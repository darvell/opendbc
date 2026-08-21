import unittest
from types import SimpleNamespace
from typing import cast
from unittest.mock import patch

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.subaru.carcontroller import CarController
from opendbc.car.subaru.carstate import CarState
from opendbc.car.subaru.fingerprints import FW_VERSIONS
from opendbc.car.subaru.interface import CarInterface
from opendbc.car.subaru.subarucan import create_es_static_1
from opendbc.car.subaru.values import CAR, CarControllerParams, SubaruFlags
from opendbc.car.vehicle_model import VehicleModel
from opendbc.sunnypilot.car.interfaces import _initialize_stop_and_go
from opendbc.sunnypilot.car.subaru.stop_and_go import SnGCarController
from opendbc.sunnypilot.car.subaru.values_ext import SubaruFlagsSP, SubaruSafetyFlagsSP


class TestSubaruFingerprint(unittest.TestCase):
  def test_fw_version_format(self):
    for platform, fws_per_ecu in FW_VERSIONS.items():
      for (ecu, _, _), fws in fws_per_ecu.items():
        fw_size = len(fws[0])
        for fw in fws:
          assert len(fw) == fw_size, f"{platform} {ecu}: {len(fw)} {fw_size}"


class TestSubaruAngleController(unittest.TestCase):
  def setUp(self):
    self.controller = CarController.__new__(CarController)
    self.controller.p = cast(CarControllerParams, SimpleNamespace(
      STEER_OVERRIDE_TORQUE_HIGH=200,
      STEER_OVERRIDE_TORQUE_LOW=150,
      ACTIVE_ANGLE_MAX=190,
      ANGLE_INACTIVE_MAX=650,
    ))
    self.controller.apply_angle_last = 0.0
    self.controller.driver_override = False
    self.controller.packer = cast(CANPacker, object())
    self.controller.VM = cast(VehicleModel, object())

  @staticmethod
  def control(lat_active=True, angle=0.0, enabled=True):
    return SimpleNamespace(enabled=enabled, latActive=lat_active, actuators=SimpleNamespace(steeringAngleDeg=angle))

  @staticmethod
  def state(measured=0.0, torque=0.0, speed=15.0):
    return SimpleNamespace(out=SimpleNamespace(steeringAngleDeg=measured, steeringTorque=torque, vEgoRaw=speed))

  @patch("opendbc.car.subaru.carcontroller.subarucan.create_steering_control_angle")
  def test_inactive_tracks_measured_angle_through_full_lock(self, create_angle_msg):
    self.controller.lateral_angle(self.control(lat_active=False, angle=0), self.state(measured=500))
    create_angle_msg.assert_called_once_with(self.controller.packer, 500.0, False)
    self.assertEqual(self.controller.apply_angle_last, 500.0)

  @patch("opendbc.car.subaru.carcontroller.subarucan.create_steering_control_angle")
  def test_mads_only_state_keeps_inactive_angle_stream(self, create_angle_msg):
    self.controller.lateral_angle(self.control(lat_active=True, angle=-20, enabled=False), self.state(measured=15))
    create_angle_msg.assert_called_once_with(self.controller.packer, 15.0, False)
    self.assertEqual(self.controller.apply_angle_last, 15.0)

  @patch("opendbc.car.subaru.carcontroller.subarucan.create_steering_control_angle")
  def test_full_lock_forces_inactive_request(self, create_angle_msg):
    self.controller.lateral_angle(self.control(lat_active=True, angle=-190), self.state(measured=-205))
    create_angle_msg.assert_called_once_with(self.controller.packer, -205.0, False)
    self.assertEqual(self.controller.apply_angle_last, -205.0)

  @patch("opendbc.car.subaru.carcontroller.apply_steer_angle_limits_vm", side_effect=lambda angle, *_: angle)
  @patch("opendbc.car.subaru.carcontroller.subarucan.create_steering_control_angle")
  def test_driver_override_hysteresis(self, create_angle_msg, _apply_limits):
    cc = self.control(lat_active=True, angle=20)

    self.controller.lateral_angle(cc, self.state(measured=15, torque=201))
    create_angle_msg.assert_called_with(self.controller.packer, 15.0, False)
    self.assertTrue(self.controller.driver_override)

    self.controller.lateral_angle(cc, self.state(measured=16, torque=175))
    create_angle_msg.assert_called_with(self.controller.packer, 16.0, False)
    self.assertTrue(self.controller.driver_override)

    self.controller.lateral_angle(cc, self.state(measured=16, torque=149))
    create_angle_msg.assert_called_with(self.controller.packer, 20, True)
    self.assertFalse(self.controller.driver_override)

  @patch("opendbc.car.subaru.carcontroller.apply_steer_angle_limits_vm", side_effect=lambda angle, *_: angle)
  @patch("opendbc.car.subaru.carcontroller.subarucan.create_steering_control_angle")
  def test_low_speed_deadzone_suppresses_small_angle_changes(self, create_angle_msg, _apply_limits):
    self.controller.apply_angle_last = 10.0
    self.controller.lateral_angle(self.control(angle=12), self.state(measured=10, speed=2))
    create_angle_msg.assert_called_once_with(self.controller.packer, 10.0, True)

  def test_speed_dependent_longitudinal_commands(self):
    low = self.controller.longitudinal_commands(1.0, 5.0)
    high = self.controller.longitudinal_commands(1.0, 25.0)
    self.assertGreater(high[0], low[0])
    self.assertGreater(high[1], low[1])
    self.assertEqual(low[2], 0)

    low_brake = self.controller.longitudinal_commands(-3.5, 5.0)
    high_brake = self.controller.longitudinal_commands(-3.5, 25.0)
    self.assertEqual(low_brake[0], CarControllerParams.THROTTLE_ENGINE_BRAKE)
    self.assertEqual(low_brake[2], 350)
    self.assertEqual(high_brake[2], 295)


class TestSubaruStopAndGo(unittest.TestCase):
  def setUp(self):
    self.sng = SnGCarController(SimpleNamespace(flags=0),
                                 SimpleNamespace(flags=SubaruFlagsSP.STOP_AND_GO))
    self.cc = SimpleNamespace(enabled=True, hudControl=SimpleNamespace(leadVisible=True),
                              cruiseControl=SimpleNamespace(resume=False))
    self.cs = SimpleNamespace(
      out=SimpleNamespace(standstill=True),
      es_distance_msg={"Close_Distance": 4.0},
      throttle_msg={"COUNTER": 0},
      brake_pedal_msg={"COUNTER": 0},
    )

  @patch("opendbc.sunnypilot.car.subaru.stop_and_go.subarucan_ext.create_brake_pedal")
  @patch("opendbc.sunnypilot.car.subaru.stop_and_go.subarucan_ext.create_throttle")
  def test_resume_pulse_when_lead_distance_increases(self, create_throttle, create_brake_pedal):
    self.sng.prev_close_distance = 3.5
    sends = self.sng.create_stop_and_go(self, self.cc, self.cs, frame=100)
    create_throttle.assert_called_once_with(self, self.sng.CP, self.cs.throttle_msg, True)
    create_brake_pedal.assert_called_once_with(self, self.sng.CP, self.cs.brake_pedal_msg, False)
    self.assertEqual(sends, [create_throttle.return_value, create_brake_pedal.return_value])

  @patch("opendbc.sunnypilot.car.subaru.stop_and_go.subarucan_ext.create_brake_pedal")
  @patch("opendbc.sunnypilot.car.subaru.stop_and_go.subarucan_ext.create_throttle")
  def test_explicit_resume_request_works_without_lead_visible(self, create_throttle, create_brake_pedal):
    self.cc.hudControl.leadVisible = False
    self.cc.cruiseControl.resume = True
    sends = self.sng.create_stop_and_go(self, self.cc, self.cs, frame=100)
    create_throttle.assert_called_once_with(self, self.sng.CP, self.cs.throttle_msg, True)
    create_brake_pedal.assert_called_once_with(self, self.sng.CP, self.cs.brake_pedal_msg, False)
    self.assertEqual(sends, [create_throttle.return_value, create_brake_pedal.return_value])

  @patch("opendbc.sunnypilot.car.subaru.stop_and_go.subarucan_ext.create_brake_pedal")
  @patch("opendbc.sunnypilot.car.subaru.stop_and_go.subarucan_ext.create_throttle")
  def test_heartbeat_frames_preserved_without_resume(self, create_throttle, create_brake_pedal):
    self.sng.prev_close_distance = 4.0
    sends = self.sng.create_stop_and_go(self, self.cc, self.cs, frame=100)
    create_throttle.assert_called_once_with(self, self.sng.CP, self.cs.throttle_msg, False)
    create_brake_pedal.assert_called_once_with(self, self.sng.CP, self.cs.brake_pedal_msg, False)
    self.assertEqual(sends, [create_throttle.return_value, create_brake_pedal.return_value])


class TestSubaruStopAndGoInitialization(unittest.TestCase):
  def test_alpha_long_crosstrek_keeps_stop_and_go_disabled(self):
    cp = SimpleNamespace(
      brand='subaru',
      carFingerprint=CAR.SUBARU_CROSSTREK_2025,
      flags=SubaruFlags.GLOBAL_GEN2 | SubaruFlags.LKAS_ANGLE,
      openpilotLongitudinalControl=True,
      autoResumeSng=False,
    )
    cp_sp = SimpleNamespace(flags=0, safetyParam=0)

    _initialize_stop_and_go(cp, cp_sp, {
      "SubaruStopAndGo": "1",
      "SubaruStopAndGoManualParkingBrake": "1",
    })

    self.assertEqual(cp_sp.flags, 0)
    self.assertEqual(cp_sp.safetyParam, 0)
    self.assertFalse(cp.autoResumeSng)

  def test_stock_long_crosstrek_enables_stop_and_go(self):
    cp = SimpleNamespace(
      brand='subaru',
      carFingerprint=CAR.SUBARU_CROSSTREK_2025,
      flags=SubaruFlags.GLOBAL_GEN2 | SubaruFlags.LKAS_ANGLE,
      openpilotLongitudinalControl=False,
      autoResumeSng=False,
    )
    cp_sp = SimpleNamespace(flags=0, safetyParam=0)

    _initialize_stop_and_go(cp, cp_sp, {
      "SubaruStopAndGo": "1",
      "SubaruStopAndGoManualParkingBrake": "0",
    })

    self.assertEqual(cp_sp.flags, SubaruFlagsSP.STOP_AND_GO)
    self.assertEqual(cp_sp.safetyParam, SubaruSafetyFlagsSP.STOP_AND_GO)
    self.assertTrue(cp.autoResumeSng)


class TestSubaruAlphaLongitudinal(unittest.TestCase):
  def test_params_use_non_pcm_cruise(self):
    candidate = CAR.SUBARU_CROSSTREK_2025
    cp = CarInterface.get_std_params(candidate)
    cp.flags = int(candidate.config.flags)
    cp = CarInterface._get_params(cp, candidate, {0: {}, 1: {}, 2: {}}, [], True, False, False)

    cp_sp = structs.CarParamsSP(pcmCruiseSpeed=True)
    cp_sp = CarInterface._get_params_sp(cp, cp_sp, candidate, {0: {}, 1: {}, 2: {}}, [], True, False, False)

    self.assertTrue(cp.openpilotLongitudinalControl)
    self.assertFalse(cp.pcmCruise)
    self.assertTrue(cp.autoResumeSng)
    self.assertFalse(cp_sp.pcmCruiseSpeed)

  def test_physical_buttons_drive_alpha_long_cruise(self):
    carstate = CarState.__new__(CarState)
    carstate.cruise_button_states = {"Set": False, "Resume": False, "Main": False}
    ret = structs.CarState()
    ret.cruiseState.standstill = True
    cp_alt = SimpleNamespace(vl={
      "CruiseControl": {"Cruise_On": 1},
      "Cruise_Buttons": {"Set": 1, "Resume": 0, "Main": 0},
    })

    carstate.update_alpha_long_cruise_state(cp_alt, ret)

    self.assertTrue(ret.cruiseState.available)
    self.assertFalse(ret.cruiseState.enabled)
    self.assertFalse(ret.cruiseState.standstill)
    self.assertEqual(len(ret.buttonEvents), 1)
    self.assertTrue(ret.buttonEvents[0].pressed)
    self.assertEqual(ret.buttonEvents[0].type, structs.CarState.ButtonEvent.Type.decelCruise)

    cp_alt.vl["Cruise_Buttons"]["Set"] = 0
    carstate.update_alpha_long_cruise_state(cp_alt, ret)
    self.assertEqual(len(ret.buttonEvents), 1)
    self.assertFalse(ret.buttonEvents[0].pressed)
    self.assertEqual(ret.buttonEvents[0].type, structs.CarState.ButtonEvent.Type.decelCruise)

    carstate.CP = SimpleNamespace(pcmCruise=False)
    self.assertTrue(carstate.update_button_enable(ret.buttonEvents))

  def test_mads_preserves_cruise_button_events(self):
    carstate = CarState.__new__(CarState)
    carstate.CP = SimpleNamespace(flags=SubaruFlags.LKAS_ANGLE)
    carstate.cruise_button_states = {"Set": False, "Resume": False, "Main": False}
    carstate.lkas_button = 0
    carstate.prev_lkas_button = 0
    ret = structs.CarState()
    cp_alt = SimpleNamespace(vl={
      "CruiseControl": {"Cruise_On": 1},
      "Cruise_Buttons": {"Set": 0, "Resume": 1, "Main": 0},
    })
    parsers = {Bus.cam: SimpleNamespace(vl={"ES_LKAS_State": {"LKAS_Dash_State": 0}})}

    carstate.update_alpha_long_cruise_state(cp_alt, ret)
    carstate.update_mads(ret, parsers)

    self.assertEqual(len(ret.buttonEvents), 1)
    self.assertEqual(ret.buttonEvents[0].type, structs.CarState.ButtonEvent.Type.accelCruise)


class TestSubaruSurroundings(unittest.TestCase):
  def setUp(self):
    self.carstate = CarState.__new__(CarState)
    self.carstate.CP = SimpleNamespace(enableBsm=True, flags=SubaruFlags.GLOBAL_GEN2)
    self.carstate.bsm_stale_frames = 0
    self.carstate.left_approaching_hold_frames = 0
    self.carstate.right_approaching_hold_frames = 0
    self.carstate.sonar_stale_frames = 0

  @staticmethod
  def parsers(bsm=None, bsm_updated=True, sonar=None, sonar_updated=True):
    bsm_values = {
      "L_ADJACENT": 0, "L_APPROACHING": 0, "R_ADJACENT": 0, "R_APPROACHING": 0,
      "L_RCTA": 0, "R_RCTA": 0,
    }
    if bsm is not None:
      bsm_values.update(bsm)
    bsm = bsm_values
    if sonar is None:
      sonar = {
        "Sonar_Available": 1, "Sonar_Off": 0, "Sonar_System_Halt": 0, "Sonar_System_Fail": 0,
        "Sonar_Warn_Left": 0, "Sonar_Warn_Center": 0, "Sonar_Warn_Right": 0,
        "RAB_Available": 1, "RAB_Off": 0, "RAB_Warn_State": 0,
      }
    bsm_all = {key: ([value] if bsm_updated else []) for key, value in bsm.items()}
    sonar_all = {key: ([value] if sonar_updated else []) for key, value in sonar.items()}
    return SimpleNamespace(vl={"BSD_RCTA": bsm}, vl_all={"BSD_RCTA": bsm_all}), \
      SimpleNamespace(vl={"ES_STATIC_1": sonar}, vl_all={"ES_STATIC_1": sonar_all})

  def update(self, **kwargs):
    cp, cp_cam = self.parsers(**kwargs)
    ret, ret_sp = structs.CarState(), structs.CarStateSP()
    self.carstate.update_subaru_surroundings(cp, cp_cam, ret, ret_sp)
    return ret, ret_sp

  def test_approaching_vehicle_is_held_after_raw_signal_clears(self):
    ret, ret_sp = self.update(bsm={"L_ADJACENT": 0, "L_APPROACHING": 1, "R_ADJACENT": 0, "R_APPROACHING": 0})
    self.assertTrue(ret.leftBlindspot)
    self.assertTrue(ret_sp.blindSpotLeftApproaching)

    ret, ret_sp = self.update()
    self.assertTrue(ret.leftBlindspot)
    self.assertFalse(ret_sp.blindSpotLeftApproaching)
    self.assertEqual(self.carstate.left_approaching_hold_frames, self.carstate.BSM_APPROACHING_HOLD_FRAMES - 1)

  def test_stale_bsm_fails_closed(self):
    self.carstate.bsm_stale_frames = self.carstate.BSM_STALE_FRAMES
    ret, ret_sp = self.update(bsm_updated=False)
    self.assertFalse(ret_sp.blindSpotMonitorValid)
    self.assertTrue(ret.leftBlindspot)
    self.assertTrue(ret.rightBlindspot)

  def test_rear_cross_traffic_is_preserved_separately(self):
    _, ret_sp = self.update(bsm={"L_RCTA": 1, "R_RCTA": 0})
    self.assertTrue(ret_sp.rearCrossTrafficLeft)
    self.assertFalse(ret_sp.rearCrossTrafficRight)

  def test_sonar_warning_and_fault_states(self):
    sonar = {
      "Sonar_Available": 1, "Sonar_Off": 0, "Sonar_System_Halt": 0, "Sonar_System_Fail": 0,
      "Sonar_Warn_Left": 2, "Sonar_Warn_Center": 4, "Sonar_Warn_Right": 1,
      "RAB_Available": 1, "RAB_Off": 0, "RAB_Warn_State": 2,
    }
    _, ret_sp = self.update(sonar=sonar)
    self.assertTrue(ret_sp.rearSonarValid)
    self.assertEqual((ret_sp.rearSonarLeft, ret_sp.rearSonarCenter, ret_sp.rearSonarRight), (2, 4, 1))
    self.assertEqual(ret_sp.rearAutomaticBrakingAlert, 2)

    sonar["Sonar_System_Fail"] = 1
    _, ret_sp = self.update(sonar=sonar)
    self.assertFalse(ret_sp.rearSonarValid)
    self.assertTrue(ret_sp.rearSonarSystemFaulted)

  def test_synthesized_rab_status_matches_normal_stock_state(self):
    _, dat, _ = create_es_static_1(CANPacker("subaru_global_2017_generated"))
    self.assertEqual(dat[2], 0xC0)

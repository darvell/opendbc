import unittest
from types import SimpleNamespace
from typing import cast
from unittest.mock import patch

from opendbc.can import CANPacker
from opendbc.car import structs
from opendbc.car.subaru.carcontroller import CarController
from opendbc.car.subaru.carstate import CarState
from opendbc.car.subaru.fingerprints import FW_VERSIONS
from opendbc.car.subaru.subarucan import create_es_static_1
from opendbc.car.subaru.values import CarControllerParams, SubaruFlags
from opendbc.car.vehicle_model import VehicleModel


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
      ANGLE_INACTIVE_MAX=650,
    ))
    self.controller.apply_angle_last = 0.0
    self.controller.driver_override = False
    self.controller.packer = cast(CANPacker, object())
    self.controller.VM = cast(VehicleModel, object())

  @staticmethod
  def control(lat_active=True, angle=0.0):
    return SimpleNamespace(latActive=lat_active, actuators=SimpleNamespace(steeringAngleDeg=angle))

  @staticmethod
  def state(measured=0.0, torque=0.0, speed=15.0):
    return SimpleNamespace(out=SimpleNamespace(steeringAngleDeg=measured, steeringTorque=torque, vEgoRaw=speed))

  @patch("opendbc.car.subaru.carcontroller.subarucan.create_steering_control_angle")
  def test_inactive_tracks_measured_angle_through_full_lock(self, create_angle_msg):
    self.controller.lateral_angle(self.control(lat_active=False, angle=0), self.state(measured=500))
    create_angle_msg.assert_called_once_with(self.controller.packer, 500.0, False)
    self.assertEqual(self.controller.apply_angle_last, 500.0)

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

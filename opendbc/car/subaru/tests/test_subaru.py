import unittest
from types import SimpleNamespace
from typing import cast
from unittest.mock import patch

from opendbc.can import CANPacker, CANParser
from opendbc.car import DT_CTRL, Bus, structs
from opendbc.car.common.filter_simple import FirstOrderFilter
from opendbc.car.subaru.carcontroller import CarController
from opendbc.car.subaru.carstate import CarState
from opendbc.car.subaru.fingerprints import FW_VERSIONS
from opendbc.car.subaru.interface import CarInterface
from opendbc.car.subaru.subarucan import create_es_lkas_alert, create_es_static_1
from opendbc.car.subaru.values import CAR, DBC, CanBus, CarControllerParams, SubaruFlags
from opendbc.car.vehicle_model import VehicleModel
from opendbc.safety import ALTERNATIVE_EXPERIENCE
from opendbc.sunnypilot.car.interfaces import _initialize_stop_and_go, _initialize_subaru_comfort
from opendbc.sunnypilot.car.subaru.values_ext import SubaruFlagsSP, SubaruSafetyFlagsSP
from opendbc.sunnypilot.car.subaru.comfort import COMFORT_BURST_LEN, COMFORT_DEADLINE_FRAMES, COMFORT_PRESS_FRAMES, \
  COMFORT_PRESS_TRIES, COMFORT_SETTLE_FRAMES, ComfortCarController


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
    self.controller.angle_filter = FirstOrderFilter(0.0, 0.0, DT_CTRL)
    self.controller.CP = SimpleNamespace(alternativeExperience=0, openpilotLongitudinalControl=False)
    self.controller.lat_eligible_last = True  # past the extra inactive frame sent after engaging

  def step(self, cc, cs):
    # These tests cover request gating and limits; the low-speed target filter is covered separately.
    self.controller.angle_filter.x = cc.actuators.steeringAngleDeg
    return self.controller.lateral_angle(cc, cs)

  @staticmethod
  def control(lat_active=True, angle=0.0, enabled=True):
    return SimpleNamespace(enabled=enabled, latActive=lat_active, actuators=SimpleNamespace(steeringAngleDeg=angle))

  @staticmethod
  def state(measured=0.0, torque=0.0, speed=15.0, brake=False, main=True, cruise=True):
    return SimpleNamespace(out=SimpleNamespace(steeringAngleDeg=measured, steeringTorque=torque, vEgoRaw=speed, brakePressed=brake,
                                               cruiseState=SimpleNamespace(available=main, enabled=cruise)))

  @patch("opendbc.car.subaru.carcontroller.subarucan.create_steering_control_angle")
  def test_inactive_tracks_measured_angle_through_full_lock(self, create_angle_msg):
    self.step(self.control(lat_active=False, angle=0), self.state(measured=500))
    create_angle_msg.assert_called_once_with(self.controller.packer, 500.0, False)
    self.assertEqual(self.controller.apply_angle_last, 500.0)

  @patch("opendbc.car.subaru.carcontroller.subarucan.create_steering_control_angle")
  def test_mads_only_state_keeps_inactive_angle_stream(self, create_angle_msg):
    self.step(self.control(lat_active=True, angle=-20, enabled=False), self.state(measured=15))
    create_angle_msg.assert_called_once_with(self.controller.packer, 15.0, False)
    self.assertEqual(self.controller.apply_angle_last, 15.0)

  @patch("opendbc.car.subaru.carcontroller.get_max_angle_vm", return_value=1000.)
  @patch("opendbc.car.subaru.carcontroller.get_max_angle_delta_vm", return_value=100.)
  @patch("opendbc.car.subaru.carcontroller.apply_steer_angle_limits_vm", side_effect=lambda angle, *_: angle)
  @patch("opendbc.car.subaru.carcontroller.subarucan.create_steering_control_angle")
  def test_extra_inactive_frame_after_engaging(self, create_angle_msg, *_):
    self.controller.p.ANGLE_LIMITS = SimpleNamespace(MAX_ANGLE_RATE=100.)
    self.controller.lat_eligible_last = False
    for measured, expected in ((10.0, (10.0, False)), (11.0, (12, True)), (12.0, (12, True))):
      self.step(self.control(angle=12), self.state(measured=measured))
      create_angle_msg.assert_called_with(self.controller.packer, *expected)
    # Any loss of eligibility re-arms the extra frame.
    self.step(self.control(angle=12), self.state(measured=12, brake=True))
    self.step(self.control(angle=13), self.state(measured=12))
    create_angle_msg.assert_called_with(self.controller.packer, 12.0, False)

  @patch("opendbc.car.subaru.carcontroller.subarucan.create_steering_control_angle")
  def test_full_lock_forces_inactive_request(self, create_angle_msg):
    self.step(self.control(lat_active=True, angle=-190), self.state(measured=-205))
    create_angle_msg.assert_called_once_with(self.controller.packer, -205.0, False)
    self.assertEqual(self.controller.apply_angle_last, -205.0)

  @patch("opendbc.car.subaru.carcontroller.get_max_angle_vm", return_value=1000.)
  @patch("opendbc.car.subaru.carcontroller.get_max_angle_delta_vm", return_value=100.)
  @patch("opendbc.car.subaru.carcontroller.apply_steer_angle_limits_vm", side_effect=lambda angle, *_: angle)
  @patch("opendbc.car.subaru.carcontroller.subarucan.create_steering_control_angle")
  def test_request_mirrors_panda_revocations(self, create_angle_msg, *_):
    self.controller.p.ANGLE_LIMITS = SimpleNamespace(MAX_ANGLE_RATE=100.)
    mads = ALTERNATIVE_EXPERIENCE.ENABLE_MADS
    cases = (
      # alternativeExperience, CarControl.enabled, CarState kwargs, LKAS_Request expected
      (0, True, {}, True),
      (0, True, {"brake": True}, False),        # Panda drops controls on the brake frame itself
      (0, True, {"cruise": False}, False),       # ES_Status disengaged on this frame
      (0, True, {"main": False}, False),         # cruise main off
      (0, False, {}, False),                      # MADS-only request without MADS armed in Panda
      (mads, False, {}, True),                    # MADS lateral-only
      (mads, False, {"main": False}, False),
      (mads, True, {"brake": True}, True),       # MADS keeps lateral through braking by default
      (mads | ALTERNATIVE_EXPERIENCE.MADS_DISENGAGE_LATERAL_ON_BRAKE, True, {"brake": True}, False),
      (mads | ALTERNATIVE_EXPERIENCE.MADS_PAUSE_LATERAL_ON_BRAKE, False, {"brake": True}, False),
    )
    for alt_exp, enabled, state, expected in cases:
      with self.subTest(alt_exp=alt_exp, enabled=enabled, **state):
        self.controller.CP.alternativeExperience = alt_exp
        self.controller.apply_angle_last = 10.0
        self.controller.lat_eligible_last = True
        self.step(self.control(angle=12, enabled=enabled), self.state(measured=10, **state))
        create_angle_msg.assert_called_with(self.controller.packer, 12 if expected else 10.0, expected)

  @patch("opendbc.car.subaru.carcontroller.get_max_angle_vm", return_value=1000.)
  @patch("opendbc.car.subaru.carcontroller.get_max_angle_delta_vm", return_value=100.)
  @patch("opendbc.car.subaru.carcontroller.apply_steer_angle_limits_vm", side_effect=lambda angle, *_: angle)
  @patch("opendbc.car.subaru.carcontroller.subarucan.create_steering_control_angle")
  def test_driver_override_hysteresis(self, create_angle_msg, _apply_limits, _max_delta, _max_angle):
    self.controller.p.ANGLE_LIMITS = SimpleNamespace(MAX_ANGLE_RATE=100.)
    cc = self.control(lat_active=True, angle=20)

    self.step(cc, self.state(measured=15, torque=201))
    create_angle_msg.assert_called_with(self.controller.packer, 15.0, False)
    self.assertTrue(self.controller.driver_override)

    self.step(cc, self.state(measured=16, torque=175))
    create_angle_msg.assert_called_with(self.controller.packer, 16.0, False)
    self.assertTrue(self.controller.driver_override)

    # Released: one extra inactive frame, then the request resumes.
    self.step(cc, self.state(measured=16, torque=149))
    create_angle_msg.assert_called_with(self.controller.packer, 16.0, False)
    self.assertFalse(self.controller.driver_override)
    self.step(cc, self.state(measured=16, torque=149))
    create_angle_msg.assert_called_with(self.controller.packer, 20, True)

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

  def test_light_deceleration_coasts_before_braking(self):
    throttle, rpm, brake = self.controller.longitudinal_commands(-0.1, 10.0)
    self.assertGreater(throttle, CarControllerParams.THROTTLE_ENGINE_BRAKE)
    self.assertLess(throttle, CarControllerParams.THROTTLE_BASE_V[2])
    self.assertGreater(rpm, CarControllerParams.BRAKE_RPM_V[2])
    self.assertLess(rpm, CarControllerParams.RPM_BASE_V[2])
    self.assertEqual(brake, 0)

  def test_braking_starts_continuously_below_coast_region(self):
    brake_start = CarControllerParams.BRAKE_START_ACCEL_V[2]
    at_threshold = self.controller.longitudinal_commands(brake_start, 10.0)
    below_threshold = self.controller.longitudinal_commands(brake_start - 0.01, 10.0)

    self.assertEqual(at_threshold, (CarControllerParams.THROTTLE_ENGINE_BRAKE, CarControllerParams.BRAKE_RPM_V[2], 0))
    self.assertEqual(below_threshold[0], CarControllerParams.THROTTLE_ENGINE_BRAKE)
    self.assertGreater(below_threshold[2], 0)

  def test_zero_accel_has_no_command_discontinuity(self):
    slightly_negative = self.controller.longitudinal_commands(-0.001, 10.0)
    zero = self.controller.longitudinal_commands(0.0, 10.0)

    self.assertLessEqual(abs(zero[0] - slightly_negative[0]), 5)
    self.assertLessEqual(abs(zero[1] - slightly_negative[1]), 5)
    self.assertEqual(slightly_negative[2], 0)
    self.assertEqual(zero[2], 0)

  def test_stop_accel_matches_observed_hold_pressure(self):
    throttle, rpm, brake = self.controller.longitudinal_commands(-2.0, 0.0)
    self.assertEqual(throttle, CarControllerParams.THROTTLE_ENGINE_BRAKE)
    self.assertEqual(rpm, CarControllerParams.BRAKE_RPM_V[0])
    self.assertGreaterEqual(brake, 225)
    self.assertLessEqual(brake, 240)


class TestSubaruLowSpeedAngleFilter(unittest.TestCase):
  """Low-speed steering is smoothed by a low pass filter instead of held still by a deadzone."""

  @staticmethod
  def make_interface(speed):
    cp = CarInterface.get_non_essential_params(CAR.SUBARU_CROSSTREK_2025)
    ci = CarInterface(cp, CarInterface.get_non_essential_params_sp(cp, CAR.SUBARU_CROSSTREK_2025))
    ci.update([])
    ci.CS.out = structs.CarState(vEgo=speed, vEgoRaw=speed)
    ci.CS.out.cruiseState.available = ci.CS.out.cruiseState.enabled = True
    ci.CC.lat_eligible_last = True  # skip the extra inactive frame sent after engaging
    return ci

  @staticmethod
  def steer(ci, target, frames, lat_active=True):
    cc = structs.CarControl(enabled=lat_active, latActive=lat_active)
    cc.actuators.steeringAngleDeg = target
    return [ci.CC.update(cc.as_reader(), structs.CarControlSP(), ci.CS, frame * 10_000_000)[0].steeringAngleDeg for frame in range(frames)]

  def test_filter_time_constant_follows_speed(self):
    for speed, tau in ((0., 0.3), (5., 0.3), (7.5, 0.2), (10., 0.1), (15., 0.05), (20., 0.), (30., 0.)):
      with self.subTest(speed=speed):
        ci = self.make_interface(speed)
        self.steer(ci, 1.0, 1)
        self.assertAlmostEqual(ci.CC.angle_filter.x, DT_CTRL / (tau + DT_CTRL))

  def test_small_request_reaches_the_wheel_at_low_speed(self):
    # The old 6 deg deadzone at 3 m/s never let a 4 deg request through.
    out = self.steer(self.make_interface(3.0), 4.0, 300)
    self.assertGreater(out[0], 0.)
    self.assertAlmostEqual(out[-1], 4.0, places=1)

  def test_low_speed_request_ramps_without_jumps(self):
    out = self.steer(self.make_interface(3.0), 4.0, 300)
    steps = [b - a for a, b in zip(out, out[1:], strict=False)]
    self.assertTrue(all(s >= 0. for s in steps))
    self.assertLess(max(steps), 0.5)

  def test_filter_updates_between_steering_messages(self):
    ci = self.make_interface(3.0)
    out = self.steer(ci, 1.0, 3)
    # Steering is sent every other frame, but the filter integrates all three control frames.
    self.assertEqual(out[0], out[1])
    self.assertAlmostEqual(out[2], 1.0 - (0.3 / 0.31) ** 3)

  def test_filter_follows_request_while_inactive(self):
    ci = self.make_interface(3.0)
    self.steer(ci, 12.0, 1, lat_active=False)
    self.assertEqual(ci.CC.angle_filter.x, 12.0)


class TestSubaruStopAndGoInitialization(unittest.TestCase):
  def test_gen2_crosstrek_never_enables_camera_side_stop_and_go(self):
    # Gen2 EyeSight reads throttle and brake pedal on the alt bus as well, so spoofing them toward the
    # camera bus cannot resume it, and blocking their forwarding only starves the camera's copy. On our
    # 2024 Crosstrek every replacement 0x40/0x139 frame was also rejected by Panda (route 1e).
    for alpha_long in (False, True):
      with self.subTest(alpha_long=alpha_long):
        cp = SimpleNamespace(brand='subaru', carFingerprint=CAR.SUBARU_CROSSTREK_2025,
                             flags=SubaruFlags.GLOBAL_GEN2 | SubaruFlags.LKAS_ANGLE,
                             openpilotLongitudinalControl=alpha_long, autoResumeSng=False)
        cp_sp = SimpleNamespace(flags=0, safetyParam=0)
        _initialize_stop_and_go(cp, cp_sp, {"SubaruStopAndGo": "1", "SubaruStopAndGoManualParkingBrake": "1"})
        self.assertEqual((cp_sp.flags, cp_sp.safetyParam), (0, 0))
        self.assertFalse(cp.autoResumeSng)

  def test_comfort_opt_in_sets_controller_and_panda_flags(self):
    params = {"SubaruAutoVehicleHold": "1", "SubaruStopStartOff": "1"}
    for platform, supported in ((CAR.SUBARU_CROSSTREK_2025, True), (CAR.SUBARU_OUTBACK, False)):
      with self.subTest(platform=platform):
        cp = CarInterface.get_non_essential_params(platform)
        cp_sp = SimpleNamespace(flags=0, safetyParam=0)
        _initialize_subaru_comfort(cp, cp_sp, params)
        expected = (SubaruFlagsSP.AUTO_VEHICLE_HOLD | SubaruFlagsSP.STOP_START_OFF,
                    SubaruSafetyFlagsSP.AUTO_VEHICLE_HOLD | SubaruSafetyFlagsSP.STOP_START_OFF) if supported else (0, 0)
        self.assertEqual((cp_sp.flags, cp_sp.safetyParam), expected)


class TestSubaruAlphaLongitudinal(unittest.TestCase):
  def test_params_keep_unverified_alpha_long_disabled(self):
    candidate = CAR.SUBARU_CROSSTREK_2025
    cp = CarInterface.get_std_params(candidate)
    cp.flags = int(candidate.config.flags)
    cp = CarInterface._get_params(cp, candidate, {0: {}, 1: {}, 2: {}}, [], True, False, False)

    cp_sp = structs.CarParamsSP(pcmCruiseSpeed=True)
    cp_sp = CarInterface._get_params_sp(cp, cp_sp, candidate, {0: {}, 1: {}, 2: {}}, [], True, False, False)

    self.assertFalse(cp.alphaLongitudinalAvailable)
    self.assertFalse(cp.openpilotLongitudinalControl)
    self.assertTrue(cp.pcmCruise)
    self.assertFalse(cp.flags & SubaruFlags.DISABLE_EYESIGHT)
    self.assertTrue(cp_sp.pcmCruiseSpeed)

  def test_gen3_main_state_uses_observed_active_low_bit(self):
    carstate = CarState.__new__(CarState)
    carstate.gen3_cruise_main_prev = None
    ret = structs.CarState()
    ret.cruiseState.standstill = True
    cp_alt = SimpleNamespace(vl={"CruiseControl": {"Gen3_Cruise_Off": 0}})

    # Never auto-engage from the initial main-on sample.
    carstate.update_alpha_long_cruise_state(cp_alt, ret)
    self.assertTrue(ret.cruiseState.available)
    self.assertFalse(ret.cruiseState.enabled)
    self.assertFalse(ret.cruiseState.standstill)
    self.assertEqual(len(ret.buttonEvents), 0)

    cp_alt.vl["CruiseControl"]["Gen3_Cruise_Off"] = 1
    carstate.update_alpha_long_cruise_state(cp_alt, ret)
    self.assertFalse(ret.cruiseState.available)
    self.assertEqual(len(ret.buttonEvents), 0)

    # A deliberate off-to-on edge is exposed as a virtual Set release.
    cp_alt.vl["CruiseControl"]["Gen3_Cruise_Off"] = 0
    carstate.update_alpha_long_cruise_state(cp_alt, ret)
    self.assertTrue(ret.cruiseState.available)
    self.assertEqual(len(ret.buttonEvents), 1)
    self.assertEqual(ret.buttonEvents[0].type, structs.CarState.ButtonEvent.Type.decelCruise)
    self.assertFalse(ret.buttonEvents[0].pressed)

  def test_gen3_main_edge_does_not_enable_with_brake(self):
    carstate = CarState.__new__(CarState)
    carstate.gen3_cruise_main_prev = False
    ret = structs.CarState(brakePressed=True)
    cp_alt = SimpleNamespace(vl={"CruiseControl": {"Gen3_Cruise_Off": 0}})

    carstate.update_alpha_long_cruise_state(cp_alt, ret)
    self.assertTrue(ret.cruiseState.available)
    self.assertEqual(len(ret.buttonEvents), 0)

  def test_gen3_main_state_decodes_captured_frames(self):
    parser = CANParser(DBC[CAR.SUBARU_CROSSTREK_2025][Bus.pt], [("CruiseControl", 0)], CanBus.alt)

    parser.update([0, [(0x240, bytes.fromhex("b30c001013008ab8"), CanBus.alt)]])
    self.assertEqual(parser.vl["CruiseControl"]["Gen3_Cruise_Off"], 1)

    parser.update([1, [(0x240, bytes.fromhex("c30d00a054408ab6"), CanBus.alt)]])
    self.assertEqual(parser.vl["CruiseControl"]["Gen3_Cruise_Off"], 0)


class TestSubaruLkasAlert(unittest.TestCase):
  """LKAS_ANGLE cameras repeat their LKAS alert on 0x3C4, which the dash also reads."""

  def setUp(self):
    self.packer = CANPacker(DBC[CAR.SUBARU_CROSSTREK_2025][Bus.pt])

  def roundtrip(self, values, visual_alert=structs.CarControl.HUDControl.VisualAlert.none):
    stock = {"CHECKSUM": 0, "Signal1": 0, "LKAS_Alert": 0, "LKAS_Alert_Msg": 0, "LKAS_Alert_State": 0, "Signal2": 0, "Signal3": 0}
    stock.update(values)
    addr, dat, bus = create_es_lkas_alert(self.packer, 0, stock, visual_alert)
    parser = CANParser(DBC[CAR.SUBARU_CROSSTREK_2025][Bus.pt], [("ES_LKAS_Alert", float("nan"))], bus)
    parser.update([(0, [(addr, dat, bus)])])
    return parser.vl["ES_LKAS_Alert"]

  def test_keep_hands_nag_is_filtered_with_its_state(self):
    for msg in (1, 7):
      with self.subTest(msg=msg):
        out = self.roundtrip({"LKAS_Alert_Msg": msg, "LKAS_Alert_State": 4})
        self.assertEqual((out["LKAS_Alert_Msg"], out["LKAS_Alert_State"]), (0, 0))

  def test_other_alerts_pass_through(self):
    out = self.roundtrip({"LKAS_Alert_Msg": 6, "LKAS_Alert_State": 4, "LKAS_Alert": 1, "Signal3": 1234})
    self.assertEqual((out["LKAS_Alert_Msg"], out["LKAS_Alert_State"], out["LKAS_Alert"], out["Signal3"]), (6, 4, 1, 1234))

  def test_audible_alerts_match_es_lkas_state_filtering(self):
    for alert, msg, expected in ((27, 0, 0), (30, 0, 0), (28, 7, 0), (28, 0, 28), (24, 0, 24)):
      with self.subTest(alert=alert, msg=msg):
        self.assertEqual(self.roundtrip({"LKAS_Alert": alert, "LKAS_Alert_Msg": msg})["LKAS_Alert"], expected)

  def test_openpilot_steer_required_is_shown(self):
    out = self.roundtrip({}, structs.CarControl.HUDControl.VisualAlert.steerRequired)
    self.assertEqual((out["LKAS_Alert_Msg"], out["LKAS_Alert_State"]), (1, 4))

  def test_missing_alert_does_not_invalidate_can(self):
    cp = CarInterface.get_non_essential_params(CAR.SUBARU_CROSSTREK_2025)
    cam = CarState.get_can_parsers(cp, None)[Bus.cam]
    _ = cam.vl["ES_LKAS_State"]

    def run(start):
      # can_valid is evaluated once per update, as card does
      valid = []
      for i in range(start, start + 30):
        cam.update([(int(i * 1e8), [self.packer.make_can_msg("ES_LKAS_State", CanBus.camera, {"COUNTER": i % 16})])])
        valid.append(cam.can_valid)
      return valid[-1]

    self.assertTrue(run(0))
    self.assertEqual(cam.ts_nanos["ES_LKAS_Alert"]["LKAS_Alert_Msg"], 0)
    # Registering it the ordinary way (by reading vl) would make an absent message invalidate CAN.
    _ = cam.vl["ES_HighBeamAssist"]
    self.assertFalse(run(30))

  def test_replacement_is_sent_only_after_the_camera_sends_one(self):
    cp = CarInterface.get_non_essential_params(CAR.SUBARU_CROSSTREK_2025)
    ci = CarInterface(cp, CarInterface.get_non_essential_params_sp(cp, CAR.SUBARU_CROSSTREK_2025))

    def sent_alerts(frames):
      cc = structs.CarControl().as_reader()
      return [addr for frame in range(frames) for addr, *_ in ci.CC.update(cc, structs.CarControlSP(), ci.CS, frame)[1] if addr == 0x3C4]

    ci.update([(0, [])])
    self.assertEqual(sent_alerts(20), [])
    ci.update([(int(1e8), [self.packer.make_can_msg("ES_LKAS_Alert", CanBus.camera, {"LKAS_Alert_Msg": 1})])])
    self.assertEqual(sent_alerts(20), [0x3C4, 0x3C4])


COMFORT_CONTROL_TEMPLATE = {"CHECKSUM": 0, "COUNTER": 3, "Signal1": 0, "AVH_REQUEST": 0, "Signal2": 1, "Signal3": 0,
                            "Signal4": 0, "Signal5": 0x0E, "Signal6": 0x0A}  # our 2024 Crosstrek, engine running
DASHLIGHTS_TEMPLATE = {"CHECKSUM": 0, "COUNTER": 3, "Signal1": 0, "Signal2": 0x99, "UNITS": 0, "Signal3": 0x23, "ICY_ROAD": 3,
                       "Signal4": 0x21, "Signal5": 0x96, "SEATBELT_FL": 1, "Signal6": 0, "LEFT_BLINKER": 0, "RIGHT_BLINKER": 0,
                       "Signal7": 0, "STOP_START": 0, "Signal8": 0, "Signal9": 0}


class FakeComfortCar:
  """The CarState fields the comfort requests read, answering like the measured car: AVH about 90 ms after a
  request, and the start-stop shutoff toggling once per gap between the car's own 10 Hz Dashlights frames that
  carries a press (so a press straddling one of the car's frames lands as two presses)."""
  AVH_ANSWER_DELAY = 9
  CAR_PERIOD = 10
  STOP_START_ANSWER_DELAY = 3

  def __init__(self, avh_active=None, stop_start_disabled=None, comfort_control=True, dashlights=True):
    self.avh_active = avh_active
    self.stop_start_disabled = stop_start_disabled
    self.comfort_control_msg = dict(COMFORT_CONTROL_TEMPLATE) if comfort_control else None
    self.dashlights_msg = dict(DASHLIGHTS_TEMPLATE) if dashlights else None
    self.avh_deaf = self.stop_start_deaf = False
    self.avh_answer_in = self.stop_start_answer_in = None
    self.presses = 0
    self.gap_pressed = False
    self.tick = 0

  def step(self, sent):
    addrs = [addr for addr, _, _ in sent]
    if 0x6BB in addrs and not self.avh_deaf and self.avh_answer_in is None:
      self.avh_answer_in = self.AVH_ANSWER_DELAY
    if self.avh_answer_in is not None:
      self.avh_answer_in -= 1
      if self.avh_answer_in == 0:
        self.avh_active, self.avh_answer_in = True, None

    self.gap_pressed |= 0x390 in addrs
    self.tick += 1
    if self.stop_start_answer_in is not None:
      self.stop_start_answer_in -= 1
      if self.stop_start_answer_in == 0:
        self.stop_start_disabled, self.stop_start_answer_in = not self.stop_start_disabled, None
    if self.dashlights_msg is not None and self.tick % self.CAR_PERIOD == 0:
      if self.gap_pressed:
        self.presses += 1
        if not self.stop_start_deaf:
          self.stop_start_answer_in = self.STOP_START_ANSWER_DELAY
        self.gap_pressed = False
      self.dashlights_msg["COUNTER"] = (self.dashlights_msg["COUNTER"] + 1) % 16


class TestSubaruComfort(unittest.TestCase):
  """Auto Vehicle Hold on and auto start-stop off, requested once at the start of a drive."""

  def setUp(self):
    self.packer = CANPacker(DBC[CAR.SUBARU_CROSSTREK_2025][Bus.pt])

  @staticmethod
  def controller(avh=False, stop_start=False, platform=CAR.SUBARU_CROSSTREK_2025):
    cp = CarInterface.get_non_essential_params(platform)
    flags = (SubaruFlagsSP.AUTO_VEHICLE_HOLD if avh else 0) | (SubaruFlagsSP.STOP_START_OFF if stop_start else 0)
    return ComfortCarController(cp, SimpleNamespace(flags=flags))

  def run_frames(self, cc, car, frames=3000, start=COMFORT_SETTLE_FRAMES):
    sent = []
    for frame in range(start, start + frames):
      step = cc.create_comfort(self.packer, car, frame)
      sent += step
      car.step(step)
    return sent

  def decode(self, sent, name):
    parser = CANParser(DBC[CAR.SUBARU_CROSSTREK_2025][Bus.pt], [(name, float("nan"))], CanBus.alt)
    out = []
    for i, msg in enumerate(sent):
      parser.update([(i, [msg])])
      out.append(dict(parser.vl[name]))
    return out

  def test_nothing_without_opt_in_or_on_other_platforms(self):
    car = FakeComfortCar(avh_active=False, stop_start_disabled=False)
    self.assertEqual(self.run_frames(self.controller(), car), [])
    self.assertEqual(self.run_frames(self.controller(True, True, CAR.SUBARU_OUTBACK), car), [])

  def test_avh_asks_once_copying_the_cars_frame(self):
    car = FakeComfortCar(avh_active=False)
    sent = self.run_frames(self.controller(avh=True), car)
    self.assertEqual(len(sent), COMFORT_BURST_LEN)
    self.assertTrue(car.avh_active)
    self.assertTrue(all(addr == 0x6BB and bus == CanBus.alt for addr, _, bus in sent))
    for msg in self.decode(sent, "Comfort_Control"):
      self.assertEqual(msg["AVH_REQUEST"], 2)
      self.assertEqual(msg["Signal6"], 0x0A)  # copied from the car, not NylonDiamond's 0x00
    self.assertTrue(all(bytes(dat)[3:] == bytes.fromhex("0100000e0a") for _, dat, _ in sent))

  def test_avh_waits_for_real_messages(self):
    self.assertEqual(self.run_frames(self.controller(avh=True), FakeComfortCar(avh_active=None)), [])
    self.assertEqual(self.run_frames(self.controller(avh=True), FakeComfortCar(avh_active=False, comfort_control=False)), [])

  def test_avh_nothing_when_already_on_and_never_fights_the_driver(self):
    self.assertEqual(self.run_frames(self.controller(avh=True), FakeComfortCar(avh_active=True)), [])
    cc, car = self.controller(avh=True), FakeComfortCar(avh_active=False)
    self.run_frames(cc, car)
    car.avh_active = False  # driver switches it off on the touchscreen
    self.assertEqual(self.run_frames(cc, car, frames=1000, start=COMFORT_SETTLE_FRAMES + 3000), [])

  def test_avh_retries_a_missed_request(self):
    car = FakeComfortCar(avh_active=False)
    car.avh_deaf = True
    sent = self.run_frames(self.controller(avh=True), car)
    self.assertGreater(len(sent), COMFORT_BURST_LEN)

  def test_stop_start_lands_as_exactly_one_press_from_any_phase(self):
    for phase in range(FakeComfortCar.CAR_PERIOD):
      with self.subTest(phase=phase):
        car = FakeComfortCar(stop_start_disabled=False)
        for _ in range(phase):
          car.step([])
        sent = self.run_frames(self.controller(stop_start=True), car)
        self.assertEqual(car.presses, 1)
        self.assertTrue(car.stop_start_disabled)
        self.assertEqual(len(sent), COMFORT_PRESS_FRAMES)
        for msg in self.decode(sent, "Dashlights"):
          self.assertEqual((msg["STOP_START"], msg["SEATBELT_FL"], msg["Signal2"]), (1, 1, 0x99))

  def test_stop_start_gives_up_and_leaves_the_driver_alone(self):
    car = FakeComfortCar(stop_start_disabled=False)
    car.stop_start_deaf = True
    sent = self.run_frames(self.controller(stop_start=True), car)
    self.assertEqual(car.presses, COMFORT_PRESS_TRIES)
    self.assertEqual(len(sent), COMFORT_PRESS_TRIES * COMFORT_PRESS_FRAMES)

    cc, car = self.controller(stop_start=True), FakeComfortCar(stop_start_disabled=False)
    self.run_frames(cc, car)
    car.stop_start_disabled, car.presses = False, 0  # driver re-arms it
    self.assertEqual(self.run_frames(cc, car, start=COMFORT_SETTLE_FRAMES + 3000), [])

  def test_settle_and_deadline(self):
    car = FakeComfortCar(avh_active=False, stop_start_disabled=False)
    self.assertEqual(self.run_frames(self.controller(True, True), car, frames=COMFORT_SETTLE_FRAMES, start=0), [])
    car.avh_deaf = car.stop_start_deaf = True
    cc = self.controller(True, True)
    self.assertNotEqual(self.run_frames(cc, car, frames=COMFORT_DEADLINE_FRAMES - COMFORT_SETTLE_FRAMES), [])
    self.assertEqual(self.run_frames(cc, car, start=COMFORT_DEADLINE_FRAMES + 1), [])

  def test_optional_messages_do_not_invalidate_can(self):
    cp = CarInterface.get_non_essential_params(CAR.SUBARU_CROSSTREK_2025)
    pt = CarState.get_can_parsers(cp, None)[Bus.pt]
    self.assertTrue({"Comfort_Control", "Comfort_Status", "Engine_Stop_Start"} <= {st.name for st in pt.message_states.values()})
    self.assertTrue(all(st.ignore_alive for st in pt.message_states.values()
                        if st.name in ("Comfort_Control", "Comfort_Status", "Engine_Stop_Start")))


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

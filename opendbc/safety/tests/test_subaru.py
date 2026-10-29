#!/usr/bin/env python3
import enum
import unittest
import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.lateral import get_max_angle_vm
from opendbc.car.subaru.carcontroller import CarController
from opendbc.car.subaru.interface import CarInterface
from opendbc.car.subaru.values import CAR, DBC, CarControllerParams, SubaruSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.car.vehicle_model import VehicleModel
from opendbc.sunnypilot.car.subaru import comfort
from opendbc.sunnypilot.car.subaru.values_ext import SubaruSafetyFlagsSP
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety, away_round, round_speed
from functools import partial


class SubaruMsg(enum.IntEnum):
  Brake_Status      = 0x13c
  Cruise_Buttons    = 0x146
  CruiseControl     = 0x240
  Throttle          = 0x40
  Brake_Pedal       = 0x139
  Steering_Torque   = 0x119
  Steering_2        = 0x11a
  Wheel_Speeds      = 0x13a
  ES_LKAS           = 0x122
  ES_LKAS_ANGLE     = 0x124
  ES_Brake          = 0x220
  ES_Distance       = 0x221
  ES_Status         = 0x222
  ES_DashStatus     = 0x321
  ES_LKAS_State     = 0x322
  ES_Infotainment   = 0x323
  ES_LKAS_Alert     = 0x3C4
  Dashlights        = 0x390
  Comfort_Control   = 0x6BB
  ES_HighBeamAssist = 0x22A
  ES_STATIC_1       = 0x325
  ES_STATIC_2       = 0x121
  ES_UDS_Request    = 0x787


SUBARU_MAIN_BUS = 0
SUBARU_ALT_BUS  = 1
SUBARU_CAM_BUS  = 2


def lkas_tx_msgs(alt_bus, lkas_msg=SubaruMsg.ES_LKAS, lkas_alert=False):
  msgs = [[lkas_msg,                    SUBARU_MAIN_BUS],
          [SubaruMsg.ES_Distance,       alt_bus],
          [SubaruMsg.ES_DashStatus,     SUBARU_MAIN_BUS],
          [SubaruMsg.ES_LKAS_State,     SUBARU_MAIN_BUS],
          [SubaruMsg.ES_Infotainment,   SUBARU_MAIN_BUS]]
  if lkas_alert:
    msgs.append([SubaruMsg.ES_LKAS_Alert, SUBARU_MAIN_BUS])
  return msgs


def long_tx_msgs(alt_bus):
  return [[SubaruMsg.ES_Brake, alt_bus],
          [SubaruMsg.ES_Status, alt_bus]]


def gen2_long_additional_tx_msgs():
  return [[SubaruMsg.ES_UDS_Request, SUBARU_CAM_BUS],
          [SubaruMsg.ES_HighBeamAssist, SUBARU_MAIN_BUS],
          [SubaruMsg.ES_STATIC_1, SUBARU_MAIN_BUS],
          [SubaruMsg.ES_STATIC_2, SUBARU_MAIN_BUS]]


def fwd_blacklisted_addr(lkas_msg=SubaruMsg.ES_LKAS, stop_and_go=False, lkas_alert=False):
  camera_addrs = [lkas_msg, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State, SubaruMsg.ES_Infotainment]
  if lkas_alert:
    camera_addrs.append(SubaruMsg.ES_LKAS_Alert)
  blacklisted = {SUBARU_CAM_BUS: camera_addrs}
  if stop_and_go:
    blacklisted[SUBARU_MAIN_BUS] = [SubaruMsg.Throttle, SubaruMsg.Brake_Pedal]
  return blacklisted


class TestSubaruSafetyBase(common.CarSafetyTest):
  FLAGS = 0
  SAFETY_PARAM_SP = 0
  RELAY_MALFUNCTION_ADDRS = {SUBARU_MAIN_BUS: (SubaruMsg.ES_LKAS, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State,
                                               SubaruMsg.ES_Infotainment)}
  FWD_BLACKLISTED_ADDRS = fwd_blacklisted_addr()

  MAX_RT_DELTA = 940

  DRIVER_TORQUE_ALLOWANCE = 60
  DRIVER_TORQUE_FACTOR = 50

  ALT_MAIN_BUS = SUBARU_MAIN_BUS
  ALT_CAM_BUS = SUBARU_CAM_BUS

  DEG_TO_CAN = 100

  INACTIVE_GAS = 1818

  def setUp(self):
    self.packer = CANPackerSafety("subaru_global_2017_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(self.SAFETY_PARAM_SP)
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, self.FLAGS)
    self.safety.init_tests()

  def _set_prev_torque(self, t):
    self.safety.set_desired_torque_last(t)
    self.safety.set_rt_torque_last(t)

  def _torque_driver_msg(self, torque):
    values = {"Steer_Torque_Sensor": torque}
    return self.packer.make_can_msg_safety("Steering_Torque", 0, values)

  def _speed_msg(self, speed):
    values = {s: speed for s in ["FR", "FL", "RR", "RL"]}
    return self.packer.make_can_msg_safety("Wheel_Speeds", self.ALT_MAIN_BUS, values)

  def _user_brake_msg(self, brake):
    values = {"Brake": brake}
    return self.packer.make_can_msg_safety("Brake_Status", self.ALT_MAIN_BUS, values)

  def _user_gas_msg(self, gas):
    values = {"Throttle_Pedal": gas}
    return self.packer.make_can_msg_safety("Throttle", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"Cruise_Activated": enable}
    return self.packer.make_can_msg_safety("CruiseControl", self.ALT_MAIN_BUS, values)

  def _lkas_button_msg(self, lkas_pressed=False, lkas_hud=0):
    # The camera reports an LKAS button press as LKAS_Dash_State 1, the only value selfdrived maps to ButtonType.lkas.
    values = {"LKAS_Dash_State": 1 if lkas_pressed else lkas_hud}
    return self.packer.make_can_msg_safety("ES_LKAS_State", SUBARU_CAM_BUS, values)

  def _acc_state_msg(self, enable):
    return self.packer.make_can_msg_safety("CruiseControl", self.ALT_MAIN_BUS, {"Cruise_On": enable})

  def test_enable_control_allowed_with_mads_button(self):
    for enable_mads in (True, False):
      with self.subTest("enable_mads", mads_enabled=enable_mads):
        for mads_button_press in range(4):
          with self.subTest("mads_button_press", button_state=mads_button_press):
            self.setUp()
            self.safety.set_mads_params(enable_mads, False, False)

            self._rx(self._lkas_button_msg(False, mads_button_press))
            self.assertEqual(enable_mads and mads_button_press == 1,
                             self.safety.get_controls_allowed_lateral())

  def test_mads_button_rearms_after_release(self):
    # selfdrived toggles MADS on every press, so Panda must see a new rising edge for each one.
    self.safety.set_mads_params(True, False, False)
    for _ in range(3):
      self._rx(self._lkas_button_msg(True))
      self._rx(self._lkas_button_msg(False))
      self.assertTrue(self.safety.get_controls_allowed_lateral())
      self.safety.set_controls_allowed_lateral(False)
    self._rx(self._lkas_button_msg(False, 2))
    self._rx(self._lkas_button_msg(True))
    self.assertTrue(self.safety.get_controls_allowed_lateral())


class TestSubaruStockLongitudinalSafetyBase(TestSubaruSafetyBase):
  def _cancel_msg(self, cancel, cruise_throttle=0):
    values = {"Cruise_Cancel": cancel, "Cruise_Throttle": cruise_throttle}
    return self.packer.make_can_msg_safety("ES_Distance", self.ALT_MAIN_BUS, values)

  def test_cancel_message(self):
    # test that we can only send the cancel message (ES_Distance) with inactive throttle (1818) and Cruise_Cancel=1
    for cancel in [True, False]:
      self._generic_limit_safety_check(partial(self._cancel_msg, cancel), self.INACTIVE_GAS, self.INACTIVE_GAS, 0, 2**12, 1, self.INACTIVE_GAS, cancel)


class TestSubaruTorqueSafetyBase(TestSubaruSafetyBase, common.DriverTorqueSteeringSafetyTest, common.SteerRequestCutSafetyTest):
  MAX_RATE_UP = 50
  MAX_RATE_DOWN = 70
  MAX_TORQUE_LOOKUP = [0], [2047]

  # Safety around steering req bit
  MIN_VALID_STEERING_FRAMES = 7
  MAX_INVALID_STEERING_FRAMES = 1
  STEER_STEP = 2

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKAS_Output": torque, "LKAS_Request": steer_req}
    return self.packer.make_can_msg_safety("ES_LKAS", SUBARU_MAIN_BUS, values)


class TestSubaruGen1TorqueStockLongitudinalSafety(TestSubaruStockLongitudinalSafetyBase, TestSubaruTorqueSafetyBase):
  FLAGS = 0
  TX_MSGS = lkas_tx_msgs(SUBARU_MAIN_BUS)


class TestSubaruGen2TorqueSafetyBase(TestSubaruTorqueSafetyBase):
  ALT_MAIN_BUS = SUBARU_ALT_BUS
  ALT_CAM_BUS = SUBARU_ALT_BUS

  MAX_RATE_UP = 35
  MAX_RATE_DOWN = 50
  MAX_TORQUE_LOOKUP = [0], [1500]


class TestSubaruGen2TorqueStockLongitudinalSafety(TestSubaruStockLongitudinalSafetyBase, TestSubaruGen2TorqueSafetyBase):
  FLAGS = SubaruSafetyFlags.GEN2
  TX_MSGS = lkas_tx_msgs(SUBARU_ALT_BUS)


class TestSubaruAngleSafetyBase(TestSubaruSafetyBase, common.AngleSteeringSafetyTest):
  STEER_ANGLE_MAX = 650
  ACTIVE_STEER_ANGLE_MAX = 190
  DEG_TO_CAN = 100

  ANGLE_RATE_BP = None
  ANGLE_RATE_UP = None
  ANGLE_RATE_DOWN = None
  LATERAL_FREQUENCY = 50
  cnt_angle_cmd = 0

  def setUp(self):
    self.__class__.cnt_angle_cmd = 0
    super().setUp()
    from opendbc.car.subaru.carcontroller import get_safety_CP
    self.VM = VehicleModel(get_safety_CP())

  def _speed_msg(self, speed):
    speed_kph = speed * 3.6
    values = {s: speed_kph for s in ["FR", "FL", "RR", "RL"]}
    return self.packer.make_can_msg_safety("Wheel_Speeds", self.ALT_MAIN_BUS, values)

  def _angle_cmd_msg(self, angle, enabled, increment_timer=True):
    values = {"LKAS_Output": angle, "LKAS_Request": enabled, "SET_3": 3}
    if increment_timer:
      self.safety.set_timer(self.cnt_angle_cmd * int(1e6 / self.LATERAL_FREQUENCY))
      self.__class__.cnt_angle_cmd += 1
    return self.packer.make_can_msg_safety("ES_LKAS_ANGLE", SUBARU_MAIN_BUS, values)

  def _angle_meas_msg(self, angle):
    return self.packer.make_can_msg_safety("Steering_2", SUBARU_MAIN_BUS, {"Steering_Angle": angle})

  def _pcm_status_msg(self, enable):
    return self.packer.make_can_msg_safety("ES_Status", self.ALT_MAIN_BUS, {"Cruise_Activated": enable})

  def test_angle_cmd_when_enabled(self):
    # VM-based accel/jerk limits and the Subaru active hard cap are tested below.
    pass

  def _find_max_allowed_angle_can(self, sign):
    lo, hi = 0, self.ACTIVE_STEER_ANGLE_MAX * self.DEG_TO_CAN + 10
    while lo < hi:
      mid = (lo + hi + 1) // 2
      self.safety.set_desired_angle_last(mid * sign)
      if self._tx(self._angle_cmd_msg(mid / self.DEG_TO_CAN * sign, True)):
        lo = mid
      else:
        hi = mid - 1
    return lo

  def test_lateral_accel_limit(self):
    for speed in np.linspace(1, 40, 100):
      speed = round_speed(away_round(speed * 3.6 / 0.057) * 0.057 / 3.6)
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)
        self._tx(self._angle_cmd_msg(0, True))
        max_angle_can = self._find_max_allowed_angle_can(sign)
        self.safety.set_desired_angle_last(max_angle_can * sign)
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_can / self.DEG_TO_CAN * sign, True)))
        above_limit_can = max_angle_can + 1
        self.safety.set_desired_angle_last(above_limit_can * sign)
        self.assertFalse(self._tx(self._angle_cmd_msg(above_limit_can / self.DEG_TO_CAN * sign, True)))

  def _find_max_allowed_delta_can(self, sign):
    lo, hi = 0, self.ACTIVE_STEER_ANGLE_MAX * self.DEG_TO_CAN
    while lo < hi:
      mid = (lo + hi + 1) // 2
      self.safety.set_desired_angle_last(0)
      if self._tx(self._angle_cmd_msg(mid / self.DEG_TO_CAN * sign, True)):
        lo = mid
      else:
        hi = mid - 1
    return lo

  def test_lateral_jerk_limit(self):
    for speed in np.linspace(1, 40, 100):
      speed = round_speed(away_round(speed * 3.6 / 0.057) * 0.057 / 3.6)
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)
        self._tx(self._angle_cmd_msg(0, True))
        max_delta_can = self._find_max_allowed_delta_can(sign)
        self.safety.set_desired_angle_last(0)
        self.assertTrue(self._tx(self._angle_cmd_msg(max_delta_can / self.DEG_TO_CAN * sign, True)))
        self.assertTrue(self._tx(self._angle_cmd_msg(max_delta_can / self.DEG_TO_CAN * sign, True)))
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))
        above_delta_can = max_delta_can + 1
        self.assertFalse(self._tx(self._angle_cmd_msg(above_delta_can / self.DEG_TO_CAN * sign, True)))
        self.safety.set_desired_angle_last(round(above_delta_can * sign))
        self.assertTrue(self._tx(self._angle_cmd_msg(above_delta_can / self.DEG_TO_CAN * sign, True)))
        self.assertFalse(self._tx(self._angle_cmd_msg(0, True)))
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))

  def test_active_angle_hard_cap(self):
    self.safety.set_controls_allowed(True)
    self._reset_speed_measurement(1)
    for sign in (-1, 1):
      self.safety.set_desired_angle_last(self.ACTIVE_STEER_ANGLE_MAX * self.DEG_TO_CAN * sign)
      self.assertTrue(self._tx(self._angle_cmd_msg(self.ACTIVE_STEER_ANGLE_MAX * sign, True)))
      self.safety.set_desired_angle_last((self.ACTIVE_STEER_ANGLE_MAX * self.DEG_TO_CAN + 1) * sign)
      self.assertFalse(self._tx(self._angle_cmd_msg((self.ACTIVE_STEER_ANGLE_MAX + 0.01) * sign, True)))

  def test_inactive_full_lock_tracks_measurement(self):
    for sign in (-1, 1):
      self._reset_angle_measurement(self.STEER_ANGLE_MAX * sign)
      self.assertTrue(self._tx(self._angle_cmd_msg(self.STEER_ANGLE_MAX * sign, False)))
      self._reset_angle_measurement((self.STEER_ANGLE_MAX + 1) * sign)
      self.assertFalse(self._tx(self._angle_cmd_msg((self.STEER_ANGLE_MAX + 1) * sign, False)))


class SubaruDynamicLongitudinalSafetyMixin:
  GAS_BP = [0., 5., 30.]
  GAS_MAX = [3100., 3410., 4000.]
  RPM_MAX = [900., 1850., 3100.]
  BRAKE_BP = [0., 10., 30.]
  BRAKE_MAX = [410., 330., 330.]

  def _send_brake_msg(self, brake):
    return self.packer.make_can_msg_safety("ES_Brake", self.ALT_MAIN_BUS, {"Brake_Pressure": brake})

  def _send_gas_msg(self, gas):
    return self.packer.make_can_msg_safety("ES_Distance", self.ALT_MAIN_BUS, {"Cruise_Throttle": gas})

  def _send_rpm_msg(self, rpm):
    return self.packer.make_can_msg_safety("ES_Status", self.ALT_MAIN_BUS, {"Cruise_RPM": rpm})

  def test_speed_dependent_longitudinal_limits(self):
    for speed in np.linspace(0., 35., 15):
      speed = round_speed(away_round(speed * 3.6 / 0.057) * 0.057 / 3.6)
      self._reset_speed_measurement(speed)
      self.safety.set_controls_allowed(True)
      max_gas = int(np.interp(speed, self.GAS_BP, self.GAS_MAX)) + 1
      max_rpm = int(np.interp(speed, self.GAS_BP, self.RPM_MAX)) + 1
      max_brake = int(np.interp(speed, self.BRAKE_BP, self.BRAKE_MAX)) + 1

      self.assertTrue(self._tx(self._send_gas_msg(max_gas)))
      self.assertFalse(self._tx(self._send_gas_msg(max_gas + 1)))
      self.assertTrue(self._tx(self._send_rpm_msg(max_rpm)))
      self.assertFalse(self._tx(self._send_rpm_msg(max_rpm + 1)))
      self.assertTrue(self._tx(self._send_brake_msg(max_brake)))
      self.assertFalse(self._tx(self._send_brake_msg(max_brake + 1)))

      self.safety.set_controls_allowed(False)
      self.assertTrue(self._tx(self._send_gas_msg(self.INACTIVE_GAS)))
      self.assertTrue(self._tx(self._send_rpm_msg(0)))
      self.assertTrue(self._tx(self._send_brake_msg(0)))
      self.assertFalse(self._tx(self._send_gas_msg(max_gas)))

  def test_controller_commands_fit_safety_envelope(self):
    for speed in np.linspace(0., 35., 71):
      max_gas = np.interp(speed, self.GAS_BP, self.GAS_MAX) + 1
      max_rpm = np.interp(speed, self.GAS_BP, self.RPM_MAX) + 1
      max_brake = np.interp(speed, self.BRAKE_BP, self.BRAKE_MAX) + 1
      throttle, rpm, _ = CarController.longitudinal_commands(2.0, speed)
      _, _, brake = CarController.longitudinal_commands(-3.5, speed)
      self.assertLessEqual(throttle, max_gas)
      self.assertLessEqual(rpm, max_rpm)
      self.assertLessEqual(brake, max_brake)

  def _rdbi_msg(self, did):
    return b'\x03\x22' + did.to_bytes(2) + b'\x00' * 4

  def _es_uds_msg(self, msg):
    return libsafety_py.make_CANPacket(SubaruMsg.ES_UDS_Request, SUBARU_CAM_BUS, msg)

  def test_es_uds_message(self):
    self.assertTrue(self._tx(self._es_uds_msg(b'\x02\x3e\x80' + b'\x00' * 5)))
    self.assertTrue(self._tx(self._es_uds_msg(self._rdbi_msg(0x1130))))
    self.assertFalse(self._tx(self._es_uds_msg(self._rdbi_msg(0x1131))))
    self.assertFalse(self._tx(self._es_uds_msg(b'\x03\xaa\xaa' + b'\x00' * 5)))


class SubaruAngleStockLongitudinalMadsMixin:
  def _acc_state_msg(self, enable):
    # Cruise main comes from the camera on angle-LKAS cars, matching CarState.cruiseState.available.
    return self.packer.make_can_msg_safety("ES_DashStatus", SUBARU_CAM_BUS, {"Cruise_On": enable})

  def test_legacy_cruise_main_bit_is_ignored(self):
    self.safety.set_mads_params(True, False, False)
    self._rx(self.packer.make_can_msg_safety("CruiseControl", self.ALT_MAIN_BUS, {"Cruise_On": 1}))
    self.assertFalse(self.safety.get_acc_main_on())
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_cruise_main_from_camera_arms_and_exits_mads(self):
    self.safety.set_mads_params(True, False, False)
    self._rx(self._acc_state_msg(True))
    self.assertTrue(self.safety.get_acc_main_on())
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self._rx(self._acc_state_msg(False))
    self.assertFalse(self.safety.get_controls_allowed_lateral())


class TestSubaruGen1AngleStockLongitudinalSafety(SubaruAngleStockLongitudinalMadsMixin, TestSubaruStockLongitudinalSafetyBase,
                                              TestSubaruAngleSafetyBase):
  FLAGS = SubaruSafetyFlags.LKAS_ANGLE
  TX_MSGS = lkas_tx_msgs(SUBARU_MAIN_BUS, SubaruMsg.ES_LKAS_ANGLE, lkas_alert=True)
  RELAY_MALFUNCTION_ADDRS = {SUBARU_MAIN_BUS: (SubaruMsg.ES_LKAS_ANGLE, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State,
                                               SubaruMsg.ES_Infotainment, SubaruMsg.ES_LKAS_Alert)}
  FWD_BLACKLISTED_ADDRS = fwd_blacklisted_addr(SubaruMsg.ES_LKAS_ANGLE, lkas_alert=True)


class TestSubaruGen2AngleStockLongitudinalSafety(SubaruAngleStockLongitudinalMadsMixin, TestSubaruStockLongitudinalSafetyBase,
                                              TestSubaruAngleSafetyBase):
  ALT_MAIN_BUS = SUBARU_ALT_BUS
  ALT_CAM_BUS = SUBARU_ALT_BUS
  FLAGS = SubaruSafetyFlags.GEN2 | SubaruSafetyFlags.LKAS_ANGLE
  TX_MSGS = lkas_tx_msgs(SUBARU_ALT_BUS, SubaruMsg.ES_LKAS_ANGLE, lkas_alert=True) + \
            [[SubaruMsg.Comfort_Control, SUBARU_ALT_BUS], [SubaruMsg.Dashlights, SUBARU_ALT_BUS]]

  # Our 2024 Crosstrek's steady-state Comfort_Control with the engine running, then the AVH on request.
  AVH_IDLE = bytes.fromhex("dd03000100000e0a")

  @staticmethod
  def _with_checksum(addr, dat):
    dat = bytearray(dat)
    dat[0] = (addr % 256 + addr // 256 + sum(dat[1:])) & 0xFF
    return libsafety_py.make_CANPacket(addr, SUBARU_ALT_BUS, bytes(dat))

  def _avh_msg(self, request=2, byte7=0x0A, static=None):
    dat = bytearray(self.AVH_IDLE)
    dat[2] = request
    dat[7] = byte7
    if static is not None:
      dat[3:7] = static
    return self._with_checksum(SubaruMsg.Comfort_Control, dat)

  def _stop_start_msg(self, pressed):
    dat = bytearray(bytes.fromhex("9405994687960000"))
    dat[6] = (dat[6] & ~0x40) | (0x40 if pressed else 0)
    return self._with_checksum(SubaruMsg.Dashlights, dat)

  def test_comfort_requests_refused_without_opt_in(self):
    self.assertFalse(self._tx(self._avh_msg()))
    self.assertFalse(self._tx(self._stop_start_msg(True)))
  RELAY_MALFUNCTION_ADDRS = {SUBARU_MAIN_BUS: (SubaruMsg.ES_LKAS_ANGLE, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State,
                                               SubaruMsg.ES_Infotainment, SubaruMsg.ES_LKAS_Alert)}
  FWD_BLACKLISTED_ADDRS = fwd_blacklisted_addr(SubaruMsg.ES_LKAS_ANGLE, lkas_alert=True)

  def test_locked_resume_research_cannot_inject_pedals_or_starve_stock_forwarding(self):
    self.safety.set_controls_allowed(True)
    for bus in (SUBARU_ALT_BUS, SUBARU_CAM_BUS):
      self.assertFalse(self._tx(self.packer.make_can_msg_safety("Throttle", bus, {"Throttle_Pedal": 5})))
      self.assertFalse(self._tx(self.packer.make_can_msg_safety("Brake_Pedal", bus, {"Speed": 3})))
    for address in (SubaruMsg.Throttle, SubaruMsg.Brake_Pedal):
      self.assertEqual(self.safety.safety_fwd_hook(SUBARU_MAIN_BUS, address), SUBARU_CAM_BUS)

  def test_controller_angle_stream_is_never_rejected(self):
    # The camera's 0x124 is blocked, so every rejected replacement frame is a gap the EPS faults on.
    # Drive the real controller through Panda safety across engagement edges, curves above the
    # lateral-accel bound, speed changes that shrink the bound, and large target steps.
    def bound(speed):
      return get_max_angle_vm(speed, self.VM, CarControllerParams)

    cases = (
      # start speed, end speed, initial measured angle, desired angle, whether every engaged frame can request
      (13.24, 13.24, 1.08 * bound(13.24), 0.97 * bound(13.24), True),  # engage in a curve above the controller's bound
      (13.0, 13.5, bound(13.0), bound(13.0), True),                     # speed increase lowers the bound at the limit
      (10.0, 14.0, bound(10.0), bound(10.0), True),                     # 4 m/s^2 (beyond this car) at the limit
      (30.0, 30.0, 10.0, -5.0, True),                                   # highway target step
      (3.0, 3.0, 0.0, 120.0, True),                                     # low-speed large target
      (13.24, 13.24, 1.3 * bound(13.24), 0.0, False),                   # engaging beyond Panda's own bound
      (27.0, 27.0, 0.0, 0.0, None),                                     # engaging while the driver turns the wheel
    )
    for start_speed, end_speed, measured, desired, always_requests in cases:
      for sign in (-1, 1):
        with self.subTest(start_speed=start_speed, end_speed=end_speed, measured=measured, desired=desired, sign=sign):
          self.setUp()
          cp = CarInterface.get_non_essential_params(CAR.SUBARU_CROSSTREK_2025)
          ci = CarInterface(cp, CarInterface.get_non_essential_params_sp(cp, CAR.SUBARU_CROSSTREK_2025))
          ci.update([])
          angle = sign * measured
          for _ in range(6):
            self._rx(self._angle_meas_msg(angle))
          angle_messages = requests = engaged_messages = 0
          # The driver turns the wheel at 100 deg/s through each engagement, and Panda has the
          # next Steering_2 sample before CarState does.
          driver_turning = always_requests is None
          for frame in range(200):
            speed = float(np.interp(frame, [20, 120], [start_speed, end_speed]))
            lat_active = frame not in (0, 100)
            cc = structs.CarControl(enabled=lat_active, latActive=lat_active)
            cc.actuators.steeringAngleDeg = sign * desired
            ci.CS.out = structs.CarState(vEgo=speed, vEgoRaw=speed, steeringAngleDeg=angle)
            ci.CS.out.cruiseState.available = ci.CS.out.cruiseState.enabled = lat_active
            turning = driver_turning and (frame < 20 or 100 <= frame < 120)
            self._rx(self._angle_meas_msg(angle + (1.0 if turning else 0.0)))
            self._reset_speed_measurement(speed)
            self.safety.set_controls_allowed(lat_active)
            self.safety.set_timer(frame * 10000)
            actuators, messages = ci.CC.update(cc.as_reader(), structs.CarControlSP(), ci.CS, frame * 10_000_000)
            for addr, dat, bus in messages:
              if addr == SubaruMsg.ES_LKAS_ANGLE:
                request = bool(dat[1] & 0x10)
                angle_messages += 1
                engaged_messages += lat_active
                requests += request
                self.assertTrue(lat_active or not request)
                self.assertTrue(self._tx(libsafety_py.make_CANPacket(addr, bus, dat)), f"frame {frame}")
            # The EPS tracks the accepted command; with the request clear the command echoes the wheel.
            angle = angle + 1.0 if turning else actuators.steeringAngleDeg
          self.assertEqual(angle_messages, 100)
          if always_requests is not None:
            # Each engagement starts with one extra inactive frame.
            self.assertEqual(requests, engaged_messages - 2 if always_requests else 0)


class TestSubaruGen2AngleComfortSafety(TestSubaruGen2AngleStockLongitudinalSafety):
  SAFETY_PARAM_SP = SubaruSafetyFlagsSP.AUTO_VEHICLE_HOLD | SubaruSafetyFlagsSP.STOP_START_OFF

  def test_comfort_requests_refused_without_opt_in(self):
    for sp in (SubaruSafetyFlagsSP.STOP_START_OFF, SubaruSafetyFlagsSP.AUTO_VEHICLE_HOLD):
      self.safety.set_current_safety_param_sp(sp)
      self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, self.FLAGS)
      self.assertEqual(bool(sp & SubaruSafetyFlagsSP.AUTO_VEHICLE_HOLD), self._tx(self._avh_msg()))
      self.assertEqual(bool(sp & SubaruSafetyFlagsSP.STOP_START_OFF), self._tx(self._stop_start_msg(True)))

  def test_avh_only_carries_the_on_request(self):
    for moving in (False, True):
      self._reset_speed_measurement(10. if moving else 0.)
      self.assertTrue(self._tx(self._avh_msg()))
      self.assertTrue(self._tx(self._avh_msg(byte7=0x00)))  # NylonDiamond's 2024 Wilderness
      for request in (0, 1, 3, 0xFF):
        self.assertFalse(self._tx(self._avh_msg(request=request)))
      self.assertFalse(self._tx(self._avh_msg(byte7=0x01)))
      self.assertFalse(self._tx(self._avh_msg(static=bytes([0x01, 0x00, 0x01, 0x0E]))))

  def test_stop_start_only_carries_a_press(self):
    self.assertTrue(self._tx(self._stop_start_msg(True)))
    self.assertFalse(self._tx(self._stop_start_msg(False)))

  def test_controller_comfort_frames_are_accepted(self):
    packer = CANPacker(DBC[CAR.SUBARU_CROSSTREK_2025][Bus.pt])
    comfort_control = {"Signal1": 0, "Signal2": 1, "Signal3": 0, "Signal4": 0, "Signal5": 0x0E, "Signal6": 0x0A}
    dashlights = {s: 0 for s in comfort.DASHLIGHTS_COPIED}
    for counter in range(16):
      for addr, dat, bus in (comfort.create_comfort_control(packer, counter, comfort_control),
                             comfort.create_stop_start_press(packer, counter, dashlights)):
        self.assertTrue(self._tx(libsafety_py.make_CANPacket(addr, bus, dat)), hex(addr))

  def test_comfort_refused_on_other_configs(self):
    for flags in (SubaruSafetyFlags.LKAS_ANGLE, SubaruSafetyFlags.GEN2, SubaruSafetyFlags.GEN2 | SubaruSafetyFlags.LKAS_ANGLE | SubaruSafetyFlags.LONG):
      self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, flags)
      self.assertFalse(self._tx(self._avh_msg()))
      self.assertFalse(self._tx(self._stop_start_msg(True)))


class TestSubaruGen2AngleLongitudinalSafety(SubaruDynamicLongitudinalSafetyMixin, TestSubaruAngleSafetyBase):
  ALT_MAIN_BUS = SUBARU_ALT_BUS
  ALT_CAM_BUS = SUBARU_ALT_BUS
  FLAGS = SubaruSafetyFlags.GEN2 | SubaruSafetyFlags.LONG | SubaruSafetyFlags.LKAS_ANGLE
  TX_MSGS = lkas_tx_msgs(SUBARU_ALT_BUS, SubaruMsg.ES_LKAS_ANGLE) + long_tx_msgs(SUBARU_ALT_BUS) + gen2_long_additional_tx_msgs()
  RELAY_MALFUNCTION_ADDRS = {
    SUBARU_MAIN_BUS: (SubaruMsg.ES_LKAS_ANGLE, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State, SubaruMsg.ES_Infotainment),
    SUBARU_ALT_BUS: (SubaruMsg.ES_Brake, SubaruMsg.ES_Distance, SubaruMsg.ES_Status),
  }
  FWD_BLACKLISTED_ADDRS = fwd_blacklisted_addr(SubaruMsg.ES_LKAS_ANGLE)

  def _cruise_control_msg(self, main):
    values = {"Gen3_Cruise_Off": not main}
    return self.packer.make_can_msg_safety("CruiseControl", SUBARU_ALT_BUS, values)

  def _acc_state_msg(self, enable):
    return self._cruise_control_msg(enable)

  # Stock EyeSight's cruise state is bypassed under openpilot longitudinal control.
  def test_disable_control_allowed_from_cruise(self):
    pass

  def test_enable_control_allowed_from_cruise(self):
    pass

  def test_cruise_engaged_prev(self):
    pass

  # EyeSight is silent in alpha long, so its LKAS HUD button is unavailable.
  def test_enable_control_allowed_with_mads_button(self):
    pass

  def test_mads_button_rearms_after_release(self):
    pass

  def test_enable_control_allowed_with_mads_button_and_disable_with_main_cruise(self):
    pass

  def test_engage_with_brake_pressed(self):
    pass

  def test_broadcast_button_bits_do_not_enable(self):
    self._rx(self._cruise_control_msg(True))
    for values in ({"Main": 1}, {"Set": 1}, {"Resume": 1}, {}):
      self._rx(self.packer.make_can_msg_safety("Cruise_Buttons", SUBARU_ALT_BUS, values))
      self.assertFalse(self.safety.get_controls_allowed())

  def test_main_switch_edge_enables(self):
    # Initial main-on state is only a baseline and never auto-engages.
    self._rx(self._cruise_control_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())

    self._rx(self._cruise_control_msg(False))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._cruise_control_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_main_switch_edge_does_not_enable_with_brake(self):
    self._rx(self._cruise_control_msg(True))
    self._rx(self._cruise_control_msg(False))
    self._rx(self._user_brake_msg(True))
    self._rx(self._cruise_control_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_main_switch_edge_does_not_enable_with_gas(self):
    self._rx(self._cruise_control_msg(True))
    self._rx(self._cruise_control_msg(False))
    self._rx(self._user_gas_msg(1))
    self._rx(self._cruise_control_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_main_switch_off_disables(self):
    self._rx(self._cruise_control_msg(True))
    self.safety.set_controls_allowed(True)
    self._rx(self._cruise_control_msg(False))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_captured_gen3_main_frames(self):
    main_off = libsafety_py.make_CANPacket(SubaruMsg.CruiseControl, SUBARU_ALT_BUS,
                                           bytes.fromhex("b30c001013008ab8"))
    main_on = libsafety_py.make_CANPacket(SubaruMsg.CruiseControl, SUBARU_ALT_BUS,
                                          bytes.fromhex("c30d00a054408ab6"))

    self.safety.set_controls_allowed(True)
    self.assertTrue(self._rx(main_off))
    self.assertFalse(self.safety.get_controls_allowed())

    self.safety.set_controls_allowed(False)
    self.assertTrue(self._rx(main_on))
    self.assertTrue(self.safety.get_controls_allowed())


if __name__ == "__main__":
  unittest.main()

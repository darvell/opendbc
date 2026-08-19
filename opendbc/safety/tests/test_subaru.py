#!/usr/bin/env python3
import enum
import unittest
import numpy as np

from opendbc.car.subaru.values import SubaruSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.car.vehicle_model import VehicleModel
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety, away_round, round_speed
from functools import partial


class SubaruMsg(enum.IntEnum):
  Brake_Status      = 0x13c
  CruiseControl     = 0x240
  Throttle          = 0x40
  Steering_Torque   = 0x119
  Steering_2        = 0x11a
  Wheel_Speeds      = 0x13a
  ES_LKAS           = 0x122
  ES_LKAS_ANGLE     = 0x124
  ES_Distance       = 0x221
  ES_Status         = 0x222
  ES_DashStatus     = 0x321
  ES_LKAS_State     = 0x322
  ES_Infotainment   = 0x323


SUBARU_MAIN_BUS = 0
SUBARU_ALT_BUS  = 1
SUBARU_CAM_BUS  = 2


def lkas_tx_msgs(alt_bus, lkas_msg=SubaruMsg.ES_LKAS):
  return [[lkas_msg,                    SUBARU_MAIN_BUS],
          [SubaruMsg.ES_Distance,       alt_bus],
          [SubaruMsg.ES_DashStatus,     SUBARU_MAIN_BUS],
          [SubaruMsg.ES_LKAS_State,     SUBARU_MAIN_BUS],
          [SubaruMsg.ES_Infotainment,   SUBARU_MAIN_BUS]]


def fwd_blacklisted_addr(lkas_msg=SubaruMsg.ES_LKAS):
  return {SUBARU_CAM_BUS: [lkas_msg, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State, SubaruMsg.ES_Infotainment]}


class TestSubaruSafetyBase(common.CarSafetyTest):
  FLAGS = 0
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
    values = {"LKAS_Dash_State": 2 if lkas_pressed else lkas_hud}
    return self.packer.make_can_msg_safety("ES_LKAS_State", SUBARU_CAM_BUS, values)

  def test_enable_control_allowed_with_mads_button(self):
    for enable_mads in (True, False):
      with self.subTest("enable_mads", mads_enabled=enable_mads):
        for mads_button_press in range(4):
          with self.subTest("mads_button_press", button_state=mads_button_press):
            self.safety.set_mads_params(enable_mads, False, False)

            self._rx(self._lkas_button_msg(False, mads_button_press))
            self.assertEqual(enable_mads and mads_button_press in range(1, 4),
                             self.safety.get_controls_allowed_lateral())


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


class TestSubaruGen1AngleStockLongitudinalSafety(TestSubaruStockLongitudinalSafetyBase, TestSubaruAngleSafetyBase):
  FLAGS = SubaruSafetyFlags.LKAS_ANGLE
  TX_MSGS = lkas_tx_msgs(SUBARU_MAIN_BUS, SubaruMsg.ES_LKAS_ANGLE)
  RELAY_MALFUNCTION_ADDRS = {SUBARU_MAIN_BUS: (SubaruMsg.ES_LKAS_ANGLE, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State,
                                               SubaruMsg.ES_Infotainment)}
  FWD_BLACKLISTED_ADDRS = fwd_blacklisted_addr(SubaruMsg.ES_LKAS_ANGLE)


class TestSubaruGen2AngleStockLongitudinalSafety(TestSubaruStockLongitudinalSafetyBase, TestSubaruAngleSafetyBase):
  ALT_MAIN_BUS = SUBARU_ALT_BUS
  ALT_CAM_BUS = SUBARU_ALT_BUS
  FLAGS = SubaruSafetyFlags.GEN2 | SubaruSafetyFlags.LKAS_ANGLE
  TX_MSGS = lkas_tx_msgs(SUBARU_ALT_BUS, SubaruMsg.ES_LKAS_ANGLE)
  RELAY_MALFUNCTION_ADDRS = {SUBARU_MAIN_BUS: (SubaruMsg.ES_LKAS_ANGLE, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State,
                                               SubaruMsg.ES_Infotainment)}
  FWD_BLACKLISTED_ADDRS = fwd_blacklisted_addr(SubaruMsg.ES_LKAS_ANGLE)


if __name__ == "__main__":
  unittest.main()

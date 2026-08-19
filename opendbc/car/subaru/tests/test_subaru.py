import unittest
from types import SimpleNamespace
from typing import cast
from unittest.mock import patch

from opendbc.can import CANPacker
from opendbc.car.subaru.carcontroller import CarController
from opendbc.car.subaru.fingerprints import FW_VERSIONS
from opendbc.car.subaru.values import CarControllerParams
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

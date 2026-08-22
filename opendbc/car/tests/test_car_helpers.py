import unittest
from unittest.mock import Mock, patch

from opendbc.car import structs
from opendbc.car import car_helpers


class FakeInterface:
  alpha_long_values = []

  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP

  @classmethod
  def get_params(cls, candidate, fingerprints, car_fw, alpha_long, is_release, docs):
    cls.alpha_long_values.append(alpha_long)
    ret = structs.CarParams()
    ret.carFingerprint = candidate
    return ret

  @staticmethod
  def get_params_sp(CP, candidate, fingerprints, car_fw, alpha_long, is_release_sp, docs):
    return structs.CarParamsSP()


class TestGetCarPostFingerprintCallback(unittest.TestCase):
  def setUp(self):
    FakeInterface.alpha_long_values = []

  @patch("opendbc.car.car_helpers.sunnypilot_interfaces")
  @patch("opendbc.car.car_helpers.fingerprint")
  def test_failed_post_fingerprint_callback_builds_stock_longitudinal_params(self, fingerprint, _setup_interfaces):
    fingerprint.return_value = (
      "TEST", {}, car_helpers.VIN_UNKNOWN, [], structs.CarParams.FingerprintSource.fixed, True,
    )
    callback = Mock(return_value=False)

    with patch.dict(car_helpers.interfaces, {"TEST": FakeInterface}, clear=True):
      CI = car_helpers.get_car(Mock(), Mock(), Mock(), True, False, post_fingerprint_callback=callback)

    callback.assert_called_once_with("TEST")
    self.assertEqual(FakeInterface.alpha_long_values, [False])
    self.assertFalse(CI.CP.openpilotLongitudinalControl)

  @patch("opendbc.car.car_helpers.sunnypilot_interfaces")
  @patch("opendbc.car.car_helpers.fingerprint")
  def test_successful_post_fingerprint_callback_preserves_alpha_long(self, fingerprint, _setup_interfaces):
    fingerprint.return_value = (
      "TEST", {}, car_helpers.VIN_UNKNOWN, [], structs.CarParams.FingerprintSource.fixed, True,
    )
    callback = Mock(return_value=True)

    with patch.dict(car_helpers.interfaces, {"TEST": FakeInterface}, clear=True):
      car_helpers.get_car(Mock(), Mock(), Mock(), True, False, post_fingerprint_callback=callback)

    callback.assert_called_once_with("TEST")
    self.assertEqual(FakeInterface.alpha_long_values, [True])

  @patch("opendbc.car.car_helpers.sunnypilot_interfaces")
  @patch("opendbc.car.car_helpers.fingerprint")
  def test_stock_long_does_not_run_post_fingerprint_callback(self, fingerprint, _setup_interfaces):
    fingerprint.return_value = (
      "TEST", {}, car_helpers.VIN_UNKNOWN, [], structs.CarParams.FingerprintSource.fixed, True,
    )
    callback = Mock(return_value=True)

    with patch.dict(car_helpers.interfaces, {"TEST": FakeInterface}, clear=True):
      car_helpers.get_car(Mock(), Mock(), Mock(), False, False, post_fingerprint_callback=callback)

    callback.assert_not_called()
    self.assertEqual(FakeInterface.alpha_long_values, [False])


if __name__ == "__main__":
  unittest.main()

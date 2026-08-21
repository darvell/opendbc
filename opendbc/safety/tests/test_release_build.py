#!/usr/bin/env python3
import unittest

from opendbc.car import structs
from opendbc.safety.tests.libsafety.libsafety_py import _build_libsafety, ffi, make_CANPacket


class TestBuild(unittest.TestCase):
  def test_development_build(self):
    _build_libsafety(release=False)

  def test_release_build(self):
    _build_libsafety(release=True)

  def test_release_build_honors_subaru_longitudinal_flag(self):
    safety = ffi.dlopen(_build_libsafety(release=True))
    subaru_gen2_angle_long = 1 | 2 | 8
    safety.set_safety_hooks(structs.CarParams.SafetyModel.subaru, subaru_gen2_angle_long)
    safety.init_tests()
    safety.set_controls_allowed(True)

    # Inactive ES_Brake is permitted only by the longitudinal allowlist. This
    # failed on the release Panda when the LONG flag was hidden by ALLOW_DEBUG.
    self.assertTrue(safety.safety_tx_hook(make_CANPacket(0x220, 1, b'\x00' * 8)))


if __name__ == "__main__":
  unittest.main()

import numpy as np
from opendbc.can import CANPacker
from opendbc.car import DT_CTRL, Bus, make_tester_present_msg, rate_limit
from opendbc.car.common.filter_simple import FirstOrderFilter
from opendbc.car.lateral import apply_driver_steer_torque_limits, apply_steer_angle_limits_vm, common_fault_avoidance, \
                                get_max_angle_delta_vm, get_max_angle_vm
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.subaru import subarucan
from opendbc.car.subaru.values import CAR, DBC, GLOBAL_ES_ADDR, CanBus, CarControllerParams, SubaruFlags
from opendbc.car.vehicle_model import VehicleModel
from opendbc.safety import ALTERNATIVE_EXPERIENCE

from opendbc.sunnypilot.car.subaru.comfort import ComfortCarController
from opendbc.sunnypilot.car.subaru.stop_and_go import SnGCarController

# FIXME: These limits aren't exact. The real limit is more than likely over a larger time period and
# involves the total steering angle change rather than rate, but these limits work well for now
MAX_STEER_RATE = 25  # deg/s
MAX_STEER_RATE_FRAMES = 7  # tx control frames needed before torque can be cut
ANGLE_REENGAGE_MAX_STEER_RATE = 2.0  # deg/s; do not re-take the wheel while the driver is still turning


def get_safety_CP():
  # Only the validated Gen3 Crosstrek angle platform is control-enabled.
  from opendbc.car.subaru.interface import CarInterface
  return CarInterface.get_non_essential_params(CAR.SUBARU_CROSSTREK_2025)


class CarController(CarControllerBase, SnGCarController, ComfortCarController):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    SnGCarController.__init__(self, CP, CP_SP)
    ComfortCarController.__init__(self, CP, CP_SP)
    self.apply_torque_last = 0
    self.apply_angle_last = 0.0
    self.driver_override = False
    self.angle_override_confirm_frames = 0
    self.angle_handoff_active = False
    self.lat_eligible_last = False

    self.cruise_button_prev = 0
    self.steer_rate_counter = 0

    self.p = CarControllerParams(CP)
    self.packer = CANPacker(DBC[CP.carFingerprint][Bus.pt])

    if CP.flags & SubaruFlags.LKAS_ANGLE:
      # Limit with the same fixed vehicle model Panda safety uses, not learned or per-car params.
      self.VM = VehicleModel(get_safety_CP())
      self.angle_filter = FirstOrderFilter(0.0, CarControllerParams.ANGLE_FILTER_TAU_V[0], DT_CTRL)

  def update_angle_filter(self, CC, CS):
    # The wheel hunts at low speed (coarse angle sensing/EPS actuation). A low pass filter on the
    # target smooths that without the hold-then-jump a center deadzone produces. Runs every control
    # frame, including frames that send no steering message. From commaai/opendbc#3795.
    if CC.latActive:
      self.angle_filter.update_alpha(float(np.interp(CS.out.vEgo, CarControllerParams.ANGLE_FILTER_SPEED_BP,
                                                     CarControllerParams.ANGLE_FILTER_TAU_V)))
      self.angle_filter.update(CC.actuators.steeringAngleDeg)
    else:
      self.angle_filter.x = CC.actuators.steeringAngleDeg

  def panda_allows_lateral(self, CC, CS) -> bool:
    # The camera's 0x124 is blocked, so a request Panda rejects is a gap the EPS faults on. carControl is
    # one frame behind CAN, while Panda revokes on the same frame it receives, so mirror Panda's immediate
    # revocations from this frame's CarState before asserting LKAS_Request.
    if not CC.latActive or not CS.out.cruiseState.available:
      return False

    braking = CS.out.brakePressed
    selfdrive_allowed = CC.enabled and not braking and (self.CP.openpilotLongitudinalControl or CS.out.cruiseState.enabled)
    if selfdrive_allowed:
      return True

    # MADS lateral-only control: Panda arms it from the same camera cruise-main and LKAS-button edges
    # selfdrived uses, and only keeps it through braking when neither brake mode is selected.
    alt_exp = self.CP.alternativeExperience
    if not alt_exp & ALTERNATIVE_EXPERIENCE.ENABLE_MADS:
      return False
    brake_modes = ALTERNATIVE_EXPERIENCE.MADS_DISENGAGE_LATERAL_ON_BRAKE | ALTERNATIVE_EXPERIENCE.MADS_PAUSE_LATERAL_ON_BRAKE
    return not (braking and alt_exp & brake_modes)

  def _angle_manual_handoff(self, CS, lat_eligible: bool) -> bool:
    """Keep LKAS inactive while the driver is taking the wheel or still unwinding the turn."""
    if not lat_eligible:
      self.angle_handoff_active = False
      return False

    steering_rate = abs(getattr(CS.out, "steeringRateDeg", 0.0))
    if self.driver_override:
      self.angle_handoff_active = True
      return True
    if self.angle_handoff_active:
      if steering_rate > ANGLE_REENGAGE_MAX_STEER_RATE:
        return True
      # Keep one explicit inactive frame after the wheel settles, anchoring Panda before re-entry.
      self.angle_handoff_active = False
      return True
    if steering_rate > ANGLE_REENGAGE_MAX_STEER_RATE:
      self.angle_handoff_active = True
      return True
    return False

  def lateral_angle(self, CC, CS):
    abs_torque = abs(CS.out.steeringTorque)
    if self.driver_override:
      if abs_torque < self.p.STEER_OVERRIDE_TORQUE_LOW:
        self.driver_override = False
    elif abs_torque > self.p.STEER_OVERRIDE_TORQUE_HIGH:
      self.angle_override_confirm_frames += 1
      if self.angle_override_confirm_frames >= 2:
        self.driver_override = True
        self.angle_override_confirm_frames = 0
    else:
      self.angle_override_confirm_frames = 0

    lat_eligible = self.panda_allows_lateral(CC, CS)
    manual_handoff = self._angle_manual_handoff(CS, lat_eligible)
    lat_active = lat_eligible and not self.driver_override and not manual_handoff
    # Do not keep LKAS_Request asserted while the wheel is already beyond the
    # active-angle ceiling. The EPS can latch a permanent fault even when the
    # requested angle itself is clamped to the ceiling.
    if lat_active and abs(CS.out.steeringAngleDeg) >= self.p.ACTIVE_ANGLE_MAX:
      lat_active = False

    # Before Panda allows control it anchors its jerk check to its newest angle sample, which can be newer
    # than CarState when the wheel is moving. Once allowed, an inactive frame anchors it to our echo instead,
    # so send one more inactive frame before requesting (commaai/opendbc#2864). Without it, engaging while the
    # wheel moves gets every frame rejected until the EPS faults (cd6dab3f3a4ab8d8/0000009b--b0f8c79194/4).
    lat_eligible = lat_active
    lat_active = lat_active and self.lat_eligible_last
    self.lat_eligible_last = lat_eligible

    if lat_active:
      apply_angle = apply_steer_angle_limits_vm(self.angle_filter.x, self.apply_angle_last, CS.out.vEgoRaw,
                                                CS.out.steeringAngleDeg, True, self.p, self.VM)
      # When the previous angle is outside the lateral-accel bound (engaging in a curve, or speed rising and
      # shrinking the bound), the clip above can move further than the jerk limit allows. Panda rejects that
      # frame and the EPS faults on the gap in the 50 Hz stream, so walk back at the jerk limit instead.
      # Panda's accel bound uses speed - 1 m/s, which leaves room for this recovery. From commaai/opendbc#3795.
      max_angle_delta = min(get_max_angle_delta_vm(max(CS.out.vEgoRaw, 1.), self.VM, self.p), self.p.ANGLE_LIMITS.MAX_ANGLE_RATE)
      apply_angle = rate_limit(apply_angle, self.apply_angle_last, -max_angle_delta, max_angle_delta)
      # Panda bounds lateral accel at (wheel speed - 1 m/s). If even the jerk-limited command is beyond that,
      # no active frame can be accepted (e.g. engaging in a curve well past 3.6 m/s^2). Hold the request off and
      # keep the stream valid rather than send a frame Panda would drop.
      if abs(apply_angle) > get_max_angle_vm(max(CS.out.vEgoRaw - 1., 1.), self.VM, self.p):
        lat_active = False

    if not lat_active:
      # LKAS_Request is clear, but the EPS still expects the angle field to track the wheel through full lock.
      apply_angle = float(np.clip(CS.out.steeringAngleDeg, -self.p.ANGLE_INACTIVE_MAX, self.p.ANGLE_INACTIVE_MAX))

    self.apply_angle_last = apply_angle
    return subarucan.create_steering_control_angle(self.packer, apply_angle, lat_active)

  def lateral_torque(self, CC, CS):
    apply_torque = int(round(CC.actuators.torque * self.p.STEER_MAX))
    apply_torque = apply_driver_steer_torque_limits(apply_torque, self.apply_torque_last, CS.out.steeringTorque, self.p)

    if not CC.latActive:
      apply_torque = 0

    self.apply_torque_last = apply_torque
    if self.CP.flags & SubaruFlags.PREGLOBAL:
      return subarucan.create_preglobal_steering_control(self.packer, self.frame // self.p.STEER_STEP, apply_torque, CC.latActive)

    apply_steer_req = CC.latActive
    if self.CP.flags & SubaruFlags.STEER_RATE_LIMITED:
      self.steer_rate_counter, apply_steer_req = \
        common_fault_avoidance(abs(CS.out.steeringRateDeg) > MAX_STEER_RATE, apply_steer_req,
                               self.steer_rate_counter, MAX_STEER_RATE_FRAMES)

    return subarucan.create_steering_control(self.packer, apply_torque, apply_steer_req)

  @staticmethod
  def longitudinal_commands(accel, v_ego):
    throttle_base = np.interp(v_ego, CarControllerParams.LONG_SPEED_BP, CarControllerParams.THROTTLE_BASE_V)
    rpm_base = np.interp(v_ego, CarControllerParams.LONG_SPEED_BP, CarControllerParams.RPM_BASE_V)

    if accel >= 0.:
      throttle_max = np.interp(v_ego, CarControllerParams.LONG_SPEED_BP, CarControllerParams.THROTTLE_MAX_V)
      rpm_max = np.interp(v_ego, CarControllerParams.LONG_SPEED_BP, CarControllerParams.RPM_MAX_V)
      throttle = np.interp(accel, [0., 2.], [throttle_base, throttle_max])
      rpm = np.interp(accel, [0., 2.], [rpm_base, rpm_max])
      brake = 0.
    else:
      brake_start_accel = np.interp(v_ego, CarControllerParams.LONG_SPEED_BP, CarControllerParams.BRAKE_START_ACCEL_V)
      brake_rpm = np.interp(v_ego, CarControllerParams.LONG_SPEED_BP, CarControllerParams.BRAKE_RPM_V)

      if accel > brake_start_accel:
        # Stock EyeSight does not jump straight from cruising torque to engine braking.
        # It first unwinds toward the inactive 1818 command, then reaches 808 at
        # the point where hydraulic braking begins.
        coast_midpoint = brake_start_accel / 2.
        if accel < coast_midpoint:
          throttle = np.interp(accel, [brake_start_accel, coast_midpoint],
                               [CarControllerParams.THROTTLE_ENGINE_BRAKE, CarControllerParams.THROTTLE_INACTIVE])
        else:
          throttle = np.interp(accel, [coast_midpoint, 0.], [CarControllerParams.THROTTLE_INACTIVE, throttle_base])
        rpm = np.interp(accel, [brake_start_accel, 0.], [brake_rpm, rpm_base])
        brake = 0.
      else:
        max_brake = np.interp(v_ego, CarControllerParams.LONG_SPEED_BP, CarControllerParams.BRAKE_MAX_V)
        throttle = CarControllerParams.THROTTLE_ENGINE_BRAKE
        rpm = brake_rpm
        brake = np.interp(accel, [-3.5, brake_start_accel], [max_brake, 0.])

    return int(round(throttle)), int(round(rpm)), int(round(brake))

  def update(self, CC, CC_SP, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    pcm_cancel_cmd = CC.cruiseControl.cancel

    can_sends = []

    # *** steering ***
    if self.CP.flags & SubaruFlags.LKAS_ANGLE:
      self.update_angle_filter(CC, CS)

    if (self.frame % self.p.STEER_STEP) == 0:
      if self.CP.flags & SubaruFlags.LKAS_ANGLE:
        can_sends.append(self.lateral_angle(CC, CS))
      else:
        can_sends.append(self.lateral_torque(CC, CS))

    # *** longitudinal ***

    if CC.longActive:
      apply_throttle, apply_rpm, apply_brake = self.longitudinal_commands(actuators.accel, CS.out.vEgoRaw)
      cruise_throttle = np.clip(apply_throttle, CarControllerParams.THROTTLE_MIN, CarControllerParams.THROTTLE_MAX)
      cruise_rpm = np.clip(apply_rpm, CarControllerParams.RPM_MIN, CarControllerParams.RPM_MAX)
      cruise_brake = np.clip(apply_brake, CarControllerParams.BRAKE_MIN, CarControllerParams.BRAKE_MAX)
    else:
      cruise_throttle = CarControllerParams.THROTTLE_INACTIVE
      cruise_rpm = CarControllerParams.RPM_MIN
      cruise_brake = CarControllerParams.BRAKE_MIN

    # *** alerts and pcm cancel ***
    if self.CP.flags & SubaruFlags.PREGLOBAL:
      if self.frame % 5 == 0:
        # 1 = main, 2 = set shallow, 3 = set deep, 4 = resume shallow, 5 = resume deep
        # disengage ACC when OP is disengaged
        if pcm_cancel_cmd:
          cruise_button = 1
        # turn main on if off and past start-up state
        elif not CS.out.cruiseState.available and CS.ready:
          cruise_button = 1
        else:
          cruise_button = CS.cruise_button

        # unstick previous mocked button press
        if cruise_button == 1 and self.cruise_button_prev == 1:
          cruise_button = 0
        self.cruise_button_prev = cruise_button

        can_sends.append(subarucan.create_preglobal_es_distance(self.packer, cruise_button, CS.es_distance_msg))

    else:
      if self.frame % 10 == 0:
        can_sends.append(subarucan.create_es_dashstatus(self.packer, self.frame // 10, CS.es_dashstatus_msg, CC.enabled,
                                                        self.CP.openpilotLongitudinalControl, CC.longActive, hud_control.leadVisible))

        can_sends.append(subarucan.create_es_lkas_state(self.packer, self.frame // 10, CS.es_lkas_state_msg, CC.enabled, hud_control.visualAlert,
                                                        hud_control.leftLaneVisible, hud_control.rightLaneVisible,
                                                        hud_control.leftLaneDepart, hud_control.rightLaneDepart))

        if self.CP.flags & SubaruFlags.SEND_INFOTAINMENT:
          can_sends.append(subarucan.create_es_infotainment(self.packer, self.frame // 10, CS.es_infotainment_msg, hud_control.visualAlert))

        # Panda blocks the camera's copy, so replace it once one has been seen, or the dash loses it.
        if CS.es_lkas_alert_msg is not None and not self.CP.openpilotLongitudinalControl:
          can_sends.append(subarucan.create_es_lkas_alert(self.packer, self.frame // 10, CS.es_lkas_alert_msg, hud_control.visualAlert))

      if self.CP.openpilotLongitudinalControl:
        if self.frame % 5 == 0:
          long_bus = CanBus.alt if self.CP.flags & SubaruFlags.GLOBAL_GEN2 else CanBus.main
          can_sends.append(subarucan.create_es_status(self.packer, self.frame // 5, CS.es_status_msg,
                                                      self.CP.openpilotLongitudinalControl, CC.longActive, cruise_rpm, long_bus))

          can_sends.append(subarucan.create_es_brake(self.packer, self.frame // 5, CS.es_brake_msg,
                                                     self.CP.openpilotLongitudinalControl, CC.longActive, cruise_brake, long_bus))

          can_sends.append(subarucan.create_es_distance(self.packer, self.frame // 5, CS.es_distance_msg, long_bus, pcm_cancel_cmd,
                                                        self.CP.openpilotLongitudinalControl, cruise_brake > 0, cruise_throttle))
      else:
        if pcm_cancel_cmd:
          if not (self.CP.flags & SubaruFlags.HYBRID):
            bus = CanBus.alt if self.CP.flags & SubaruFlags.GLOBAL_GEN2 else CanBus.main
            can_sends.append(subarucan.create_es_distance(self.packer, CS.es_distance_msg["COUNTER"] + 1, CS.es_distance_msg, bus, pcm_cancel_cmd))

      if self.CP.flags & SubaruFlags.DISABLE_EYESIGHT:
        # Tester present (keeps eyesight disabled)
        if self.frame % 100 == 0:
          can_sends.append(make_tester_present_msg(GLOBAL_ES_ADDR, CanBus.camera, suppress_response=True))

        # Create all of the other eyesight messages to keep the rest of the car happy when eyesight is disabled
        if self.frame % 5 == 0:
          can_sends.append(subarucan.create_es_highbeamassist(self.packer))

        if self.frame % 10 == 0:
          can_sends.append(subarucan.create_es_static_1(self.packer))

        if self.frame % 2 == 0:
          can_sends.append(subarucan.create_es_static_2(self.packer))

    can_sends.extend(SnGCarController.create_stop_and_go(self, self.packer, CC, CS, self.frame))
    can_sends.extend(ComfortCarController.create_comfort(self, self.packer, CS, self.frame))

    new_actuators = actuators.as_builder()
    if self.CP.flags & SubaruFlags.LKAS_ANGLE:
      new_actuators.steeringAngleDeg = self.apply_angle_last
    else:
      new_actuators.torque = self.apply_torque_last / self.p.STEER_MAX
      new_actuators.torqueOutputCan = self.apply_torque_last

    self.frame += 1
    return new_actuators, can_sends

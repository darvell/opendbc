/**
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */

#pragma once

extern bool subaru_stop_and_go;
bool subaru_stop_and_go = false;
int subaru_stop_and_go_throttle_pedal = 0;
int subaru_stop_and_go_brake_pedal_speed = 0;

void subaru_common_init(void) {
  const uint16_t SUBARU_PARAM_SP_STOP_AND_GO = 1;

  subaru_stop_and_go = GET_FLAG(current_safety_param_sp, SUBARU_PARAM_SP_STOP_AND_GO);
  subaru_stop_and_go_throttle_pedal = 0;
  subaru_stop_and_go_brake_pedal_speed = 0;
}

// Return true when the command violates the stop-and-go safety contract. The camera-bus
// heartbeat must echo the measured vehicle-side message; only the explicit resume value
// may differ, and only while the vehicle is stopped and controls are allowed.
bool subaru_common_stop_and_go_throttle_check(const int throttle_pedal) {
  const bool is_echo = throttle_pedal == subaru_stop_and_go_throttle_pedal;
  const bool is_resume = throttle_pedal == 5 && controls_allowed && !vehicle_moving;
  return subaru_stop_and_go && !(is_echo || is_resume);
}

bool subaru_common_stop_and_go_brake_pedal_check(const int speed, const bool is_preglobal) {
  // Brake_Pedal.Speed is encoded at 0.05625 kph/bit. The controller requests 1 or 3 kph.
  const int resume_speed = is_preglobal ? 18 : 53;
  const bool is_echo = speed == subaru_stop_and_go_brake_pedal_speed;
  const bool is_resume = speed == resume_speed && controls_allowed && !vehicle_moving;
  return subaru_stop_and_go && !(is_echo || is_resume);
}

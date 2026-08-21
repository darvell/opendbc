#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/subaru_common.h"

#define SUBARU_STEERING_LIMITS_GENERATOR(steer_max, rate_up, rate_down)               \
  {                                                                                   \
    .max_torque = (steer_max),                                                        \
    .max_rt_delta = 940,                                                              \
    .max_rate_up = (rate_up),                                                         \
    .max_rate_down = (rate_down),                                                     \
    .driver_torque_multiplier = 50,                                                   \
    .driver_torque_allowance = 60,                                                    \
    .type = TorqueDriverLimited,                                                      \
    /* the EPS will temporary fault if the steering rate is too high, so we cut the   \
       the steering torque every 7 frames for 1 frame if the steering rate is high */ \
    .min_valid_request_frames = 7,                                                    \
    .max_invalid_request_frames = 1,                                                  \
    .min_valid_request_rt_interval = 144000,  /* 10% tolerance */                     \
    .has_steer_req_tolerance = true,                                                  \
  }

#define MSG_SUBARU_Brake_Status          0x13cU
#define MSG_SUBARU_Cruise_Buttons        0x146U
#define MSG_SUBARU_CruiseControl         0x240U
#define MSG_SUBARU_Throttle              0x40U
#define MSG_SUBARU_Steering_Torque       0x119U
#define MSG_SUBARU_Steering_2            0x11aU
#define MSG_SUBARU_Wheel_Speeds          0x13aU
#define MSG_SUBARU_Brake_Pedal           0x139U

#define MSG_SUBARU_ES_LKAS               0x122U
#define MSG_SUBARU_ES_LKAS_ANGLE         0x124U
#define MSG_SUBARU_ES_Brake              0x220U
#define MSG_SUBARU_ES_Distance           0x221U
#define MSG_SUBARU_ES_Status             0x222U
#define MSG_SUBARU_ES_DashStatus         0x321U
#define MSG_SUBARU_ES_LKAS_State         0x322U
#define MSG_SUBARU_ES_Infotainment       0x323U
#define MSG_SUBARU_ES_HighBeamAssist     0x22AU
#define MSG_SUBARU_ES_STATIC_1           0x325U
#define MSG_SUBARU_ES_STATIC_2           0x121U
#define MSG_SUBARU_ES_UDS_Request        0x787U

#define SUBARU_MAIN_BUS 0U
#define SUBARU_ALT_BUS  1U
#define SUBARU_CAM_BUS  2U

#define SUBARU_BASE_TX_MSGS(alt_bus, lkas_msg) \
  {lkas_msg,                     SUBARU_MAIN_BUS, 8, .check_relay = true},  \
  {MSG_SUBARU_ES_DashStatus,     SUBARU_MAIN_BUS, 8, .check_relay = true},  \
  {MSG_SUBARU_ES_LKAS_State,     SUBARU_MAIN_BUS, 8, .check_relay = true},  \
  {MSG_SUBARU_ES_Infotainment,   SUBARU_MAIN_BUS, 8, .check_relay = true},  \

#define SUBARU_COMMON_TX_MSGS(alt_bus) \
  {MSG_SUBARU_ES_Distance, alt_bus, 8, .check_relay = false}, \

#define SUBARU_COMMON_LONG_TX_MSGS(alt_bus) \
  {MSG_SUBARU_ES_Distance,       alt_bus,         8, .check_relay = true}, \
  {MSG_SUBARU_ES_Brake,          alt_bus,         8, .check_relay = true}, \
  {MSG_SUBARU_ES_Status,         alt_bus,         8, .check_relay = true}, \

#define SUBARU_GEN2_LONG_ADDITIONAL_TX_MSGS() \
  {MSG_SUBARU_ES_UDS_Request,    SUBARU_CAM_BUS,  8, .check_relay = false}, \
  {MSG_SUBARU_ES_HighBeamAssist, SUBARU_MAIN_BUS, 8, .check_relay = false}, \
  {MSG_SUBARU_ES_STATIC_1,       SUBARU_MAIN_BUS, 8, .check_relay = false}, \
  {MSG_SUBARU_ES_STATIC_2,       SUBARU_MAIN_BUS, 8, .check_relay = false}, \

#define SUBARU_STOP_AND_GO_TX_MSGS \
  {MSG_SUBARU_Throttle,          SUBARU_CAM_BUS,  8, .check_relay = true}, \
  {MSG_SUBARU_Brake_Pedal,       SUBARU_CAM_BUS,  8, .check_relay = true}, \

#define SUBARU_COMMON_RX_CHECKS(alt_bus)                                                                                                         \
  {.msg = {{MSG_SUBARU_Throttle,        SUBARU_MAIN_BUS, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \
  {.msg = {{MSG_SUBARU_Steering_Torque, SUBARU_MAIN_BUS, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{MSG_SUBARU_Wheel_Speeds,    alt_bus,         8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{MSG_SUBARU_Brake_Status,    alt_bus,         8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{MSG_SUBARU_CruiseControl,   alt_bus,         8, 20U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{MSG_SUBARU_ES_LKAS_State,   SUBARU_CAM_BUS,  8, 10U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

#define SUBARU_LKAS_ANGLE_RX_CHECKS(alt_bus)                                                                                                    \
  {.msg = {{MSG_SUBARU_Throttle,        SUBARU_MAIN_BUS, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \
  {.msg = {{MSG_SUBARU_Steering_Torque, SUBARU_MAIN_BUS, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{MSG_SUBARU_Wheel_Speeds,    alt_bus,         8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{MSG_SUBARU_Brake_Status,    alt_bus,         8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{MSG_SUBARU_ES_Status,       alt_bus,         8, 20U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{MSG_SUBARU_Steering_2,      SUBARU_MAIN_BUS, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{MSG_SUBARU_ES_LKAS_State,   SUBARU_CAM_BUS,  8, 10U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

#define SUBARU_LKAS_ANGLE_LONG_RX_CHECKS(alt_bus)                                                                                               \
  {.msg = {{MSG_SUBARU_Throttle,        SUBARU_MAIN_BUS, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}}, \
  {.msg = {{MSG_SUBARU_Steering_Torque, SUBARU_MAIN_BUS, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{MSG_SUBARU_Wheel_Speeds,    alt_bus,         8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{MSG_SUBARU_Brake_Status,    alt_bus,         8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{MSG_SUBARU_CruiseControl,   alt_bus,         8, 20U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{MSG_SUBARU_Cruise_Buttons,  alt_bus,         8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \
  {.msg = {{MSG_SUBARU_Steering_2,      SUBARU_MAIN_BUS, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},  \

static bool subaru_gen2 = false;
static bool subaru_lkas_angle = false;
static bool subaru_longitudinal = false;
static bool subaru_set_button_prev = false;
static bool subaru_resume_button_prev = false;

static uint32_t subaru_get_checksum(const CANPacket_t *msg) {
  return (uint8_t)msg->data[0];
}

static uint8_t subaru_get_counter(const CANPacket_t *msg) {
  return (uint8_t)(msg->data[1] & 0xFU);
}

static uint32_t subaru_compute_checksum(const CANPacket_t *msg) {
  int len = GET_LEN(msg);
  uint8_t checksum = (uint8_t)(msg->addr) + (uint8_t)((unsigned int)(msg->addr) >> 8U);
  for (int i = 1; i < len; i++) {
    checksum += (uint8_t)msg->data[i];
  }
  return checksum;
}

static void subaru_rx_hook(const CANPacket_t *msg) {
  const unsigned int alt_main_bus = subaru_gen2 ? SUBARU_ALT_BUS : SUBARU_MAIN_BUS;

  if ((msg->addr == MSG_SUBARU_Steering_Torque) && (msg->bus == SUBARU_MAIN_BUS)) {
    int torque_driver_new;
    torque_driver_new = ((GET_BYTES(msg, 0, 4) >> 16) & 0x7FFU);
    torque_driver_new = -1 * to_signed(torque_driver_new, 11);
    update_sample(&torque_driver, torque_driver_new);
  }

  if (subaru_lkas_angle && (msg->addr == MSG_SUBARU_Steering_2) && (msg->bus == SUBARU_MAIN_BUS)) {
    int angle_meas_new = GET_BYTES(msg, 3, 3) & 0x1FFFFU;
    angle_meas_new = -1 * to_signed(angle_meas_new, 17);
    update_sample(&angle_meas, angle_meas_new);
  }

  if ((msg->addr == MSG_SUBARU_ES_LKAS_State) && (msg->bus == SUBARU_CAM_BUS)) {
    int lkas_hud = (msg->data[2] & 0x0CU) >> 2U;
    if ((lkas_hud >= 1) && (lkas_hud <= 3)) {
      mads_button_press = MADS_BUTTON_PRESSED;
    }
  }

  // Stock longitudinal enters controls from the stock ACC state. Alpha long
  // silences EyeSight, so it enters on the physical Set/Resume falling edge
  // and uses the ECM's independent main/engaged state for immediate exits.
  if (subaru_lkas_angle && !subaru_longitudinal && (msg->addr == MSG_SUBARU_ES_Status) && (msg->bus == alt_main_bus)) {
    bool cruise_engaged = (msg->data[3] >> 5) & 1U;
    pcm_cruise_check(cruise_engaged);
  }
  if ((msg->addr == MSG_SUBARU_CruiseControl) && (msg->bus == alt_main_bus)) {
    bool cruise_engaged = GET_BIT(msg, 41U);
    acc_main_on = GET_BIT(msg, 40U);

    if (subaru_longitudinal) {
      if (!acc_main_on || (cruise_engaged_prev && !cruise_engaged)) {
        controls_allowed = false;
      }
      cruise_engaged_prev = cruise_engaged;
    } else if (!subaru_lkas_angle) {
      pcm_cruise_check(cruise_engaged);
    } else {
    }
  }
  if (subaru_longitudinal && (msg->addr == MSG_SUBARU_Cruise_Buttons) && (msg->bus == alt_main_bus)) {
    bool main_button = GET_BIT(msg, 42U);
    bool set_button = GET_BIT(msg, 43U);
    bool resume_button = GET_BIT(msg, 44U);

    if ((subaru_set_button_prev && !set_button) || (subaru_resume_button_prev && !resume_button)) {
      controls_allowed = acc_main_on;
    }
    if (main_button) {
      controls_allowed = false;
    }

    subaru_set_button_prev = set_button;
    subaru_resume_button_prev = resume_button;
  }

  // update vehicle moving with any non-zero wheel speed
  if ((msg->addr == MSG_SUBARU_Wheel_Speeds) && (msg->bus == alt_main_bus)) {
    uint32_t fr = (GET_BYTES(msg, 1, 3) >> 4) & 0x1FFFU;
    uint32_t rr = (GET_BYTES(msg, 3, 3) >> 1) & 0x1FFFU;
    uint32_t rl = (GET_BYTES(msg, 4, 3) >> 6) & 0x1FFFU;
    uint32_t fl = (GET_BYTES(msg, 6, 2) >> 3) & 0x1FFFU;

    vehicle_moving = (fr > 0U) || (rr > 0U) || (rl > 0U) || (fl > 0U);

    UPDATE_VEHICLE_SPEED((fr + rr + rl + fl) / 4.0 * 0.057 * KPH_TO_MS);
  }

  if ((msg->addr == MSG_SUBARU_Brake_Status) && (msg->bus == alt_main_bus)) {
    brake_pressed = (msg->data[7] >> 6) & 1U;
  }

  if ((msg->addr == MSG_SUBARU_Throttle) && (msg->bus == SUBARU_MAIN_BUS)) {
    subaru_stop_and_go_throttle_pedal = msg->data[4];
    gas_pressed = msg->data[4] != 0U;
  }

  if ((msg->addr == MSG_SUBARU_Brake_Pedal) && (msg->bus == SUBARU_MAIN_BUS)) {
    subaru_stop_and_go_brake_pedal_speed = GET_BYTES(msg, 2, 2) & 0xFFFU;
  }
}

static bool subaru_longitudinal_cmd_checks(int value, int min_value, int max_value, int inactive_value) {
  bool active_valid = get_longitudinal_allowed() && !safety_max_limit_check(value, max_value, min_value);
  return !(active_valid || (value == inactive_value));
}

static bool subaru_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits SUBARU_STEERING_LIMITS      = SUBARU_STEERING_LIMITS_GENERATOR(2047, 50, 70);
  const TorqueSteeringLimits SUBARU_GEN2_STEERING_LIMITS = SUBARU_STEERING_LIMITS_GENERATOR(1500, 35, 50);

  const AngleSteeringLimits SUBARU_ANGLE_STEERING_LIMITS = {
    .max_angle = 650 * 100,
    .angle_deg_to_can = 100.,
    .frequency = 50U,
  };
  const int SUBARU_ACTIVE_ANGLE_MAX = 190 * 100;

  // Match the validated Gen3 Crosstrek vehicle model used by the Python controller.
  const AngleSteeringParams SUBARU_ANGLE_STEERING_PARAMS = {
    .slip_factor = -0.0006281955319491566,
    .steer_ratio = 17.0,
    .wheelbase = 2.6700000762939453,
  };

  const struct lookup_t SUBARU_MAX_GAS = {
    {0., 5., 30.},
    {3100., 3410., 4000.},
  };
  const struct lookup_t SUBARU_MAX_RPM = {
    {0., 5., 30.},
    {900., 1850., 3100.},
  };
  const struct lookup_t SUBARU_MAX_BRAKE = {
    {0., 10., 30.},
    {410., 330., 330.},
  };
  const float speed = vehicle_speed.min / VEHICLE_SPEED_FACTOR;
  const int max_gas = safety_interpolate(SUBARU_MAX_GAS, speed) + 1;
  const int max_rpm = safety_interpolate(SUBARU_MAX_RPM, speed) + 1;
  const int max_brake = safety_interpolate(SUBARU_MAX_BRAKE, speed) + 1;
  const int SUBARU_MIN_GAS = 808;
  const int SUBARU_INACTIVE_GAS = 1818;

  bool tx = true;
  bool violation = false;

  // steer cmd checks
  if (msg->addr == MSG_SUBARU_ES_LKAS) {
    int desired_torque = ((GET_BYTES(msg, 0, 4) >> 16) & 0x1FFFU);
    desired_torque = -1 * to_signed(desired_torque, 13);

    bool steer_req = (msg->data[3] >> 5) & 1U;

    const TorqueSteeringLimits limits = subaru_gen2 ? SUBARU_GEN2_STEERING_LIMITS : SUBARU_STEERING_LIMITS;
    violation |= steer_torque_cmd_checks(desired_torque, steer_req, limits);
  }

  if (msg->addr == MSG_SUBARU_ES_LKAS_ANGLE) {
    int desired_angle = GET_BYTES(msg, 5, 3) & 0x1FFFFU;
    desired_angle = -1 * to_signed(desired_angle, 17);
    bool lkas_request = GET_BIT(msg, 12U);

    violation |= steer_angle_cmd_checks_vm(desired_angle, lkas_request, SUBARU_ANGLE_STEERING_LIMITS, SUBARU_ANGLE_STEERING_PARAMS);
    if (lkas_request) {
      violation |= safety_max_limit_check(desired_angle, SUBARU_ACTIVE_ANGLE_MAX, -SUBARU_ACTIVE_ANGLE_MAX);
    }
  }

  if ((msg->addr == MSG_SUBARU_Throttle) && (msg->bus == SUBARU_CAM_BUS)) {
    const int throttle_pedal = msg->data[4];
    violation |= subaru_common_stop_and_go_throttle_check(throttle_pedal);
  }

  if ((msg->addr == MSG_SUBARU_Brake_Pedal) && (msg->bus == SUBARU_CAM_BUS)) {
    const int brake_pedal_speed = GET_BYTES(msg, 2, 2) & 0xFFFU;
    violation |= subaru_common_stop_and_go_brake_pedal_check(brake_pedal_speed, false);
  }

  if (msg->addr == MSG_SUBARU_ES_Brake) {
    int brake_pressure = GET_BYTES(msg, 2, 2);
    violation |= subaru_longitudinal_cmd_checks(brake_pressure, 0, max_brake, 0);
  }

  if (msg->addr == MSG_SUBARU_ES_Distance) {
    int cruise_throttle = GET_BYTES(msg, 2, 2) & 0x1FFFU;
    bool cruise_cancel = (msg->data[7] >> 0) & 1U;
    if (subaru_longitudinal) {
      violation |= subaru_longitudinal_cmd_checks(cruise_throttle, SUBARU_MIN_GAS, max_gas, SUBARU_INACTIVE_GAS);
    } else {
      violation |= (cruise_throttle != SUBARU_INACTIVE_GAS);
      violation |= !cruise_cancel;
    }
  }

  if (msg->addr == MSG_SUBARU_ES_Status) {
    int cruise_rpm = GET_BYTES(msg, 2, 2) & 0x1FFFU;
    violation |= subaru_longitudinal_cmd_checks(cruise_rpm, 0, max_rpm, 0);
  }

  if (msg->addr == MSG_SUBARU_ES_UDS_Request) {
    bool is_tester_present = (GET_BYTES(msg, 0, 4) == 0x00803E02U) && (GET_BYTES(msg, 4, 4) == 0x0U);
    bool is_button_rdbi = (GET_BYTES(msg, 0, 4) == 0x30112203U) && (GET_BYTES(msg, 4, 4) == 0x0U);
    violation |= !(is_tester_present || is_button_rdbi);
  }

  if (violation){
    tx = false;
  }
  return tx;
}

static safety_config subaru_init(uint16_t param) {
  static const CanMsg SUBARU_TX_MSGS[] = {
    SUBARU_BASE_TX_MSGS(SUBARU_MAIN_BUS, MSG_SUBARU_ES_LKAS)
    SUBARU_COMMON_TX_MSGS(SUBARU_MAIN_BUS)
  };

  static const CanMsg SUBARU_GEN2_TX_MSGS[] = {
    SUBARU_BASE_TX_MSGS(SUBARU_ALT_BUS, MSG_SUBARU_ES_LKAS)
    SUBARU_COMMON_TX_MSGS(SUBARU_ALT_BUS)
  };

  static const CanMsg SUBARU_LKAS_ANGLE_TX_MSGS[] = {
    SUBARU_BASE_TX_MSGS(SUBARU_MAIN_BUS, MSG_SUBARU_ES_LKAS_ANGLE)
    SUBARU_COMMON_TX_MSGS(SUBARU_MAIN_BUS)
  };

  static const CanMsg SUBARU_LKAS_ANGLE_GEN2_TX_MSGS[] = {
    SUBARU_BASE_TX_MSGS(SUBARU_ALT_BUS, MSG_SUBARU_ES_LKAS_ANGLE)
    SUBARU_COMMON_TX_MSGS(SUBARU_ALT_BUS)
  };

  static const CanMsg SUBARU_LKAS_ANGLE_GEN2_LONG_TX_MSGS[] = {
    SUBARU_BASE_TX_MSGS(SUBARU_ALT_BUS, MSG_SUBARU_ES_LKAS_ANGLE)
    SUBARU_COMMON_LONG_TX_MSGS(SUBARU_ALT_BUS)
    SUBARU_GEN2_LONG_ADDITIONAL_TX_MSGS()
  };

  static const CanMsg SUBARU_LKAS_ANGLE_GEN2_SNG_TX_MSGS[] = {
    SUBARU_BASE_TX_MSGS(SUBARU_ALT_BUS, MSG_SUBARU_ES_LKAS_ANGLE)
    SUBARU_COMMON_TX_MSGS(SUBARU_ALT_BUS)
    SUBARU_STOP_AND_GO_TX_MSGS
  };

  static const CanMsg subaru_stop_and_go_tx_msgs[] = {
    SUBARU_BASE_TX_MSGS(SUBARU_MAIN_BUS, MSG_SUBARU_ES_LKAS)
    SUBARU_COMMON_TX_MSGS(SUBARU_MAIN_BUS)
    SUBARU_STOP_AND_GO_TX_MSGS
  };

  const uint16_t SUBARU_PARAM_GEN2 = 1;
  const uint16_t SUBARU_PARAM_LKAS_ANGLE = 8;

  const uint16_t SUBARU_PARAM_LONGITUDINAL = 2;

  subaru_gen2 = GET_FLAG(param, SUBARU_PARAM_GEN2);
  subaru_lkas_angle = GET_FLAG(param, SUBARU_PARAM_LKAS_ANGLE);
  // This fork only sets LONG for the explicitly selected Gen3 Crosstrek port.
  // Keep the release Panda's safety mode consistent with CarParams so the
  // validated 0x220/0x221/0x222 checks are active on the real device.
  subaru_longitudinal = GET_FLAG(param, SUBARU_PARAM_LONGITUDINAL);
  subaru_set_button_prev = false;
  subaru_resume_button_prev = false;

  subaru_common_init();


  safety_config ret;
  if (subaru_lkas_angle) {
    if (subaru_gen2) {
      static RxCheck subaru_lkas_angle_gen2_rx_checks[] = {
        SUBARU_LKAS_ANGLE_RX_CHECKS(SUBARU_ALT_BUS)
      };
      static RxCheck subaru_lkas_angle_gen2_long_rx_checks[] = {
        SUBARU_LKAS_ANGLE_LONG_RX_CHECKS(SUBARU_ALT_BUS)
      };
      ret = subaru_longitudinal ? BUILD_SAFETY_CFG(subaru_lkas_angle_gen2_long_rx_checks, SUBARU_LKAS_ANGLE_GEN2_LONG_TX_MSGS) : \
                                  (subaru_stop_and_go ? BUILD_SAFETY_CFG(subaru_lkas_angle_gen2_rx_checks, SUBARU_LKAS_ANGLE_GEN2_SNG_TX_MSGS) : \
                                                        BUILD_SAFETY_CFG(subaru_lkas_angle_gen2_rx_checks, SUBARU_LKAS_ANGLE_GEN2_TX_MSGS));
    } else {
      static RxCheck subaru_lkas_angle_rx_checks[] = {
        SUBARU_LKAS_ANGLE_RX_CHECKS(SUBARU_MAIN_BUS)
      };
      ret = BUILD_SAFETY_CFG(subaru_lkas_angle_rx_checks, SUBARU_LKAS_ANGLE_TX_MSGS);
    }
  } else if (subaru_gen2) {
    static RxCheck subaru_gen2_rx_checks[] = {
      SUBARU_COMMON_RX_CHECKS(SUBARU_ALT_BUS)
    };
    ret = BUILD_SAFETY_CFG(subaru_gen2_rx_checks, SUBARU_GEN2_TX_MSGS);
  } else {
    static RxCheck subaru_rx_checks[] = {
      SUBARU_COMMON_RX_CHECKS(SUBARU_MAIN_BUS)
    };
    ret = subaru_stop_and_go ? BUILD_SAFETY_CFG(subaru_rx_checks, subaru_stop_and_go_tx_msgs) : \
                               BUILD_SAFETY_CFG(subaru_rx_checks, SUBARU_TX_MSGS);
  }
  return ret;
}

const safety_hooks subaru_hooks = {
  .init = subaru_init,
  .rx = subaru_rx_hook,
  .tx = subaru_tx_hook,
  .get_counter = subaru_get_counter,
  .get_checksum = subaru_get_checksum,
  .compute_checksum = subaru_compute_checksum,
};

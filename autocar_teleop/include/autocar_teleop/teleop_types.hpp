/*
 * teleop_types.hpp
 *
 * Created on 8/8/23 10:42 PM
 * Description:
 *
 * Copyright (c) 2023 Ruixiang Du (rdu)
 */

#ifndef BUILD_TELEOP_TYPES_HPP
#define BUILD_TELEOP_TYPES_HPP

#include <string>
#include <cstdint>

namespace xmotion {
struct DriverConfig {
  int js_index;
  std::string can_if_name;
  uint8_t vesc_id;
};

struct JoystickInput {
  int speed_cmd;
  double steering_cmd;
  bool deadman_switch;
};

struct CommandParams {
  double min_steer_angle = 0.0;
  double max_steer_angle = 1.0;
  double neutral_steer_angle = 0.5;
  double steer_angle_deadzone = 0.05;
  int min_motor_rpm = -2000;
  int max_motor_rpm = 3000;
  int motor_rpm_deadzone = 50;
  double rpm_ratio;
  double steer_ratio;
};

struct VescCommand {
  int motor_rpm;
  double servo_angle;
};
}  // namespace xmotion

#endif  // BUILD_TELEOP_TYPES_HPP

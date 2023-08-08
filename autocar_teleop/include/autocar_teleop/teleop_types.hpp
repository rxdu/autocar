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

struct VescCommand {
  int motor_rpm;
  double servo_angle;
};
}  // namespace xmotion

#endif  // BUILD_TELEOP_TYPES_HPP

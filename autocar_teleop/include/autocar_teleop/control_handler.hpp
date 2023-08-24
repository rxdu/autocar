/*
 * control_handler.hpp
 *
 * Created on 8/8/23 10:35 PM
 * Description:
 *
 * Copyright (c) 2023 Ruixiang Du (rdu)
 */

#ifndef CONTROL_COORDINATOR_HPP
#define CONTROL_COORDINATOR_HPP

#include <mutex>

#include "autocar_teleop/teleop_types.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace xmotion {
class ControlHandler {
  enum class ControlSource : int { kNone = 0, kJoystick, kCommand };

 public:
  ControlHandler(CommandParams params);
  ~ControlHandler() = default;

  void UpdateRcInput(const JoystickInput& input);
  void UpdateCommand(const geometry_msgs::msg::Twist& cmd);

  VescCommand GetCommand();

 private:
  VescCommand ClampCommand(const VescCommand& cmd);

  CommandParams params_;
  VescCommand vesc_cmd_;

  std::mutex js_input_mutex_;
  JoystickInput joystick_input_;

  std::mutex twist_cmd_mutex_;
  geometry_msgs::msg::Twist twist_cmd_;
};
}  // namespace xmotion

#endif  // CONTROL_COORDINATOR_HPP

/*
 * control_arbitrator.hpp
 *
 * Created on 8/8/23 10:35 PM
 * Description:
 *
 * Copyright (c) 2023 Ruixiang Du (rdu)
 */

#ifndef BUILD_CONTROL_ARBITRATOR_HPP
#define BUILD_CONTROL_ARBITRATOR_HPP

#include "autocar_teleop/teleop_types.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace xmotion {
class ControlArbitrator {
  enum class ControlSource : int { kNone = 0, kJoystick, kCommand };

 public:
  ControlArbitrator() = default;
  ~ControlArbitrator() = default;

  void UpdateRcInput(const JoystickInput& input);
  void UpdateCommand(const geometry_msgs::msg::Twist& cmd);

  VescCommand GetCommand() const;

 private:
  ControlSource control_source_ = ControlSource::kNone;
  JoystickInput joystick_input_;
  geometry_msgs::msg::Twist twist_cmd_;
};
}  // namespace xmotion

#endif  // BUILD_CONTROL_ARBITRATOR_HPP

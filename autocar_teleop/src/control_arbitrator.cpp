/*
 * control_arbitrator.cpp
 *
 * Created on 8/8/23 10:35 PM
 * Description:
 *
 * Copyright (c) 2023 Ruixiang Du (rdu)
 */

#include "autocar_teleop/control_arbitrator.hpp"

namespace xmotion {
void ControlArbitrator::UpdateRcInput(const JoystickInput& input) {}

void ControlArbitrator::UpdateCommand(const geometry_msgs::msg::Twist& cmd) {}

VescCommand ControlArbitrator::GetCommand() const {
  VescCommand vesc_cmd;
  return vesc_cmd;
}
}  // namespace xmotion
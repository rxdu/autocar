/*
 * control_handler.cpp
 *
 * Created on 8/8/23 10:35 PM
 * Description:
 *
 * Copyright (c) 2023 Ruixiang Du (rdu)
 */

#include "autocar_teleop/control_handler.hpp"

namespace xmotion {
xmotion::ControlHandler::ControlHandler(CommandParams params)
    : params_(params) {}

void ControlHandler::UpdateRcInput(const JoystickInput& input) {
  std::lock_guard<std::mutex> lock(js_input_mutex_);
  joystick_input_ = input;
}

void ControlHandler::UpdateCommand(const geometry_msgs::msg::Twist& cmd) {
  std::lock_guard<std::mutex> lock(twist_cmd_mutex_);
  twist_cmd_ = cmd;
}

VescCommand ControlHandler::ClampCommand(const VescCommand& cmd) {
  VescCommand clamped_cmd = cmd;
  if (cmd.motor_rpm > params_.max_motor_rpm)
    clamped_cmd.motor_rpm = params_.max_motor_rpm;
  if (cmd.motor_rpm < params_.min_motor_rpm)
    clamped_cmd.motor_rpm = params_.min_motor_rpm;
  if (cmd.servo_angle > params_.max_steer_angle)
    clamped_cmd.servo_angle = params_.max_steer_angle;
  if (cmd.servo_angle < params_.min_steer_angle)
    clamped_cmd.servo_angle = params_.min_steer_angle;
  return clamped_cmd;
}

VescCommand ControlHandler::GetCommand() {
  // make a local copy of the inputs
  JoystickInput js;
  {
    std::lock_guard<std::mutex> lock(js_input_mutex_);
    js = joystick_input_;
  }
  geometry_msgs::msg::Twist cmd;
  {
    std::lock_guard<std::mutex> lock(twist_cmd_mutex_);
    cmd = twist_cmd_;
  }

  // check which input is active
  vesc_cmd_.motor_rpm = 0;
  vesc_cmd_.servo_angle = 0.0;

  if (js.deadman_switch) {
    vesc_cmd_.motor_rpm = js.speed_cmd;
    vesc_cmd_.servo_angle = js.steering_cmd;
  } else {
    vesc_cmd_.motor_rpm = cmd.linear.x * params_.rpm_ratio;
    vesc_cmd_.servo_angle = params_.neutral_steer_angle + cmd.angular.z * params_.steer_ratio;
  }

  auto clamped_cmd = ClampCommand(vesc_cmd_);

  return clamped_cmd;
}
}  // namespace xmotion
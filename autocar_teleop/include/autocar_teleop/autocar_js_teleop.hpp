/*
 * autocar_js_teleop.hpp
 *
 * Created on 5/8/23 23:57 PM
 * Description:
 *
 * Copyright (c) 2023 Ruixiang Du (rdu)
 */

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "interface/driver/joystick_interface.hpp"
#include "motor_vesc/vesc_can_interface.hpp"

#include "autocar_teleop/teleop_types.hpp"
#include "autocar_teleop/control_arbitrator.hpp"

namespace xmotion {
class AutocarJsTeleop : public rclcpp::Node {
 public:
  AutocarJsTeleop(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void LoadParameters();
  void TwistCallback(const geometry_msgs::msg::Twist& msg) const;
  void OnMainTimer();

  bool Initialize();
  void VescStateUpdatedCallback(const StampedVescState& state);

  rclcpp::TimerBase::SharedPtr main_timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

  std::string cmd_topic_;

  DriverConfig driver_config_;
  std::unique_ptr<JoystickInterface> joystick_;
  std::unique_ptr<VescCanInterface> vesc_;

  JoystickInput js_input_;
  double min_steer_angle_ = 0.0;
  double max_steer_angle_ = 1.0;
  double neutral_steer_angle_ = 0.5;
  double steer_angle_deadzone_ = 0.05;
  int min_motor_rpm_ = -2000;
  int max_motor_rpm_ = 3000;
  int motor_rpm_deadzone_ = 50;
  ControlArbitrator control_arbitrator_;
};
}  // namespace xmotion
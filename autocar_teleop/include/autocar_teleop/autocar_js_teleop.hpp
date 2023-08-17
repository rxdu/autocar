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
#include "autocar_teleop/control_coordinator.hpp"

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
  CommandParams cmd_params_;
  std::unique_ptr<ControlCoordinator> control_coordinator_;
};
}  // namespace xmotion
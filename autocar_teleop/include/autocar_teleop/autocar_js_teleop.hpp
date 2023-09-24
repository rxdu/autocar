/*
 * autocar_js_teleop.hpp
 *
 * Created on 5/8/23 23:57 PM
 * Description:
 *
 * Copyright (c) 2023 Ruixiang Du (rdu)
 */

#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "interface/driver/joystick_interface.hpp"
#include "motor_vesc/vesc_can_interface.hpp"

#include "autocar_teleop/teleop_types.hpp"
#include "autocar_teleop/control_handler.hpp"

#include "autocar_teleop/rccar_bicycle_model.hpp"
#include "model/system_propagator.hpp"

namespace xmotion {
class AutocarJsTeleop : public rclcpp::Node {
  using Clock = std::chrono::steady_clock;
  using Timepoint = std::chrono::time_point<Clock>;

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
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  std::string cmd_topic_;
  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;

  DriverConfig driver_config_;
  std::unique_ptr<JoystickInterface> joystick_;
  std::unique_ptr<VescCanInterface> vesc_;

  JoystickInput js_input_;
  CommandParams cmd_params_;
  VescCommand active_cmd_;
  std::unique_ptr<ControlHandler> control_handler_;

  RccarBicycleKinematics::state_type robot_state_ {0.0,0.0,0.0};
  Timepoint t_;
  SystemPropagator<RccarBicycleKinematics> propagator_;
};
}  // namespace xmotion
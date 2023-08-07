/*
 * autocar_js_teleop.cpp
 *
 * Created on 6/8/23 00:05 PM
 * Description:
 *
 * Copyright (c) 2023 Ruixiang Du (rdu)
 */

#include "autocar_teleop/autocar_js_teleop.hpp"

#include <functional>

#include "input_joystick/joystick.hpp"

namespace xmotion {
AutocarJsTeleop::AutocarJsTeleop(const rclcpp::NodeOptions& options)
    : rclcpp::Node("autocar_teleop_node", options) {
  LoadParameters();

  // set up joystick & vesc driver
  joystick_ = std::make_unique<Joystick>(driver_config_.js_index);
  vesc_ = std::make_unique<VescCanInterface>();

  if (!Initialize()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to initialize AutocarTeleop node.");
    throw std::runtime_error("Failed to initialize AutocarTeleop drivers.");
  }

  // set up ROS stuff
  cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_topic_, 10,
      std::bind(&AutocarJsTeleop::TwistCallback, this, std::placeholders::_1));

  main_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(20),
                              std::bind(&AutocarJsTeleop::OnMainTimer, this));

  RCLCPP_INFO(this->get_logger(), "AutocarTeleop node has been created.");
}

void AutocarJsTeleop::LoadParameters() {
  this->declare_parameter("joystick_index", 0);
  this->declare_parameter("vesc_can_if_name", "can0");
  this->declare_parameter("vesc_id", 0x68);
  this->declare_parameter("cmd_topic_name", "cmd_vel");

  driver_config_.js_index =
      this->get_parameter("joystick_index").get_parameter_value().get<int>();
  driver_config_.can_if_name = this->get_parameter("vesc_can_if_name")
                                   .get_parameter_value()
                                   .get<std::string>();
  driver_config_.vesc_id =
      this->get_parameter("vesc_id").get_parameter_value().get<int>();

  cmd_topic_ = this->get_parameter("cmd_topic_name")
                   .get_parameter_value()
                   .get<std::string>();

  RCLCPP_INFO(this->get_logger(), "Joystick index: %d",
              driver_config_.js_index);
  RCLCPP_INFO(this->get_logger(), "VESC CAN interface name: %s",
              driver_config_.can_if_name.c_str());
  RCLCPP_INFO(this->get_logger(), "VESC ID: 0x%x", driver_config_.vesc_id);
  RCLCPP_INFO(this->get_logger(), "Cmd topic name: %s", cmd_topic_.c_str());
}

bool AutocarJsTeleop::Initialize() {
  if (!joystick_->Open()) {
    std::cout << "Failed to open joystick" << std::endl;
    return false;
  }
  vesc_->SetStateUpdatedCallback(std::bind(
      &AutocarJsTeleop::VescStateUpdatedCallback, this, std::placeholders::_1));
  if (!vesc_->Connect(driver_config_.can_if_name, driver_config_.vesc_id)) {
    std::cerr << "Failed to connect to " << driver_config_.can_if_name
              << std::endl;
  }
  return true;
}

void AutocarJsTeleop::VescStateUpdatedCallback(const StampedVescState& state) {
  std::cout << "voltage input: " << state.state.voltage_input << ", "
            << "temp pcb: " << state.state.temperature_pcb << ", "
            << "current motor: " << state.state.current_motor << ", "
            << "current input: " << state.state.current_input << ", "
            << "speed: " << state.state.speed << ", "
            << "duty cycle: " << state.state.duty_cycle << ", "
            << "charge drawn: " << state.state.charge_drawn << ", "
            << "charge regen: " << state.state.charge_regen << ", "
            << "charge drawn: " << state.state.energy_drawn << ", "
            << "charge regen: " << state.state.energy_regen << ", "
            << "displacement: " << state.state.displacement << ", "
            << "dist traveled: " << state.state.distance_traveled << std::endl;
}

void AutocarJsTeleop::TwistCallback(
    const geometry_msgs::msg::Twist& msg) const {
  RCLCPP_INFO(this->get_logger(), "Cmd (linear_x, angular_z): %d, %d",
              msg.linear.x, msg.angular.z);
}

void AutocarJsTeleop::OnMainTimer() {
  //  RCLCPP_INFO(this->get_logger(), "Hello World!");
  if (joystick_->IsOpened()) {
    std::cout << "Axes X: " << joystick_->GetAxisState(JsAxis::kX).value
              << ", Axes Y: " << joystick_->GetAxisState(JsAxis::kY).value
              << ", Axes Z: " << joystick_->GetAxisState(JsAxis::kZ).value
              << ", Axes RX: " << joystick_->GetAxisState(JsAxis::kRX).value
              << ", Axes RY: " << joystick_->GetAxisState(JsAxis::kRY).value
              << ", Axes RZ: " << joystick_->GetAxisState(JsAxis::kRZ).value
              << std::endl;

    if (joystick_->GetButtonState(JsButton::kMode)) {
      joystick_->SetJoystickRumble(0.1 * 0xFFFF, 0.5 * 0xFFFF);
    } else {
      joystick_->SetJoystickRumble(0.0 * 0xFFFF, 0.0 * 0xFFFF);
    }

    // set control source

    // send command to vesc
    if (control_source_ != ControlSource::kNone) {
      //    vesc_->SetServo(0.5);
    }
  }
}
}  // namespace xmotion

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(xmotion::AutocarJsTeleop)
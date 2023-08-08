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

  this->declare_parameter("min_steer_angle", 0.0);
  this->declare_parameter("max_steer_angle", 1.0);
  this->declare_parameter("neutral_steer_angle", 0.5);
  this->declare_parameter("steer_angle_deadzone", 0.05);

  this->declare_parameter("min_motor_rpm", -2000);
  this->declare_parameter("max_motor_rpm", 3000);
  this->declare_parameter("motor_rpm_deadzone", 50);

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

  min_steer_angle_ = this->get_parameter("min_steer_angle")
                         .get_parameter_value()
                         .get<double>();
  max_steer_angle_ = this->get_parameter("max_steer_angle")
                         .get_parameter_value()
                         .get<double>();
  neutral_steer_angle_ = this->get_parameter("neutral_steer_angle")
                             .get_parameter_value()
                             .get<double>();
  steer_angle_deadzone_ = this->get_parameter("steer_angle_deadzone")
                              .get_parameter_value()
                              .get<double>();

  min_motor_rpm_ =
      this->get_parameter("min_motor_rpm").get_parameter_value().get<int>();
  max_motor_rpm_ =
      this->get_parameter("max_motor_rpm").get_parameter_value().get<int>();
  motor_rpm_deadzone_ = this->get_parameter("motor_rpm_deadzone")
                            .get_parameter_value()
                            .get<int>();

  RCLCPP_INFO(this->get_logger(), "Joystick index: %d",
              driver_config_.js_index);
  RCLCPP_INFO(this->get_logger(), "VESC CAN interface name: %s",
              driver_config_.can_if_name.c_str());
  RCLCPP_INFO(this->get_logger(), "VESC ID: 0x%x", driver_config_.vesc_id);
  RCLCPP_INFO(this->get_logger(), "Cmd topic name: %s", cmd_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Min steer angle: %f", min_steer_angle_);
  RCLCPP_INFO(this->get_logger(), "Max steer angle: %f", max_steer_angle_);
  RCLCPP_INFO(this->get_logger(), "Neutral steer angle: %f",
              neutral_steer_angle_);
  RCLCPP_INFO(this->get_logger(), "Min motor RPM: %d", min_motor_rpm_);
  RCLCPP_INFO(this->get_logger(), "Max motor RPM: %d", max_motor_rpm_);
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
    //    std::cout << "Axes X: " << joystick_->GetAxisState(JsAxis::kX).value
    //              << ", Axes Y: " << joystick_->GetAxisState(JsAxis::kY).value
    //              << ", Axes Z: " << joystick_->GetAxisState(JsAxis::kZ).value
    //              << ", Axes RX: " <<
    //              joystick_->GetAxisState(JsAxis::kRX).value
    //              << ", Axes RY: " <<
    //              joystick_->GetAxisState(JsAxis::kRY).value
    //              << ", Axes RZ: " <<
    //              joystick_->GetAxisState(JsAxis::kRZ).value
    //              << std::endl;

    // only allows RC control if deadman switch is on
    if (joystick_->GetAxisState(JsAxis::kZ).value > 0) {
      js_input_.deadman_switch = true;
    } else {
      js_input_.deadman_switch = false;
    }

    if (js_input_.deadman_switch) {
      double motor_cmd = joystick_->GetAxisState(JsAxis::kY).value;
      // push up js stick to go forward, js axis value is negative
      if (motor_cmd <= 0.0) {
        js_input_.speed_cmd = -motor_cmd * max_motor_rpm_;
      } else {
        js_input_.speed_cmd = motor_cmd * min_motor_rpm_;
      }
      if (std::abs(js_input_.speed_cmd) < motor_rpm_deadzone_) {
        js_input_.speed_cmd = 0;
      }

      double servo_cmd = joystick_->GetAxisState(JsAxis::kRX).value;
      if (servo_cmd >= 0.0) {
        js_input_.steering_cmd =
            neutral_steer_angle_ +
            servo_cmd * (max_steer_angle_ - neutral_steer_angle_);
      } else {
        js_input_.steering_cmd =
            neutral_steer_angle_ +
            servo_cmd * (neutral_steer_angle_ - min_steer_angle_);
      }
      if (std::abs(js_input_.steering_cmd - neutral_steer_angle_) <
          steer_angle_deadzone_) {
        js_input_.steering_cmd = neutral_steer_angle_;
      }
    } else {
      js_input_.speed_cmd = 0;
      js_input_.steering_cmd = neutral_steer_angle_;
    }

    if (joystick_->GetButtonState(JsButton::kMode)) {
      joystick_->SetJoystickRumble(0.1 * 0xFFFF, 0.5 * 0xFFFF);
    } else {
      joystick_->SetJoystickRumble(0.0 * 0xFFFF, 0.0 * 0xFFFF);
    }

    RCLCPP_INFO(this->get_logger(),
                "Deadman switch: %d, Speed cmd: %d, Steering cmd: %f",
                js_input_.deadman_switch, js_input_.speed_cmd,
                js_input_.steering_cmd);

    // set control source
    control_arbitrator_.UpdateRcInput(js_input_);

    // send command to vesc
    auto cmd = control_arbitrator_.GetCommand();
    vesc_->SetSpeed(cmd.motor_rpm);
    vesc_->SetServo(cmd.servo_angle);
  } else {
    // no control allowed for safety
    vesc_->SetSpeed(0);
    vesc_->SetServo(neutral_steer_angle_);
  }
}
}  // namespace xmotion

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(xmotion::AutocarJsTeleop)
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

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace xmotion {
namespace {
geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw) {
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}
}

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

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  // start main timer
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

  this->declare_parameter("rpm_ratio", 1.0);
  this->declare_parameter("steer_ratio", 1.0);

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

  cmd_params_.min_steer_angle = this->get_parameter("min_steer_angle")
                                    .get_parameter_value()
                                    .get<double>();
  cmd_params_.max_steer_angle = this->get_parameter("max_steer_angle")
                                    .get_parameter_value()
                                    .get<double>();
  cmd_params_.neutral_steer_angle = this->get_parameter("neutral_steer_angle")
                                        .get_parameter_value()
                                        .get<double>();
  cmd_params_.steer_angle_deadzone = this->get_parameter("steer_angle_deadzone")
                                         .get_parameter_value()
                                         .get<double>();

  cmd_params_.min_motor_rpm =
      this->get_parameter("min_motor_rpm").get_parameter_value().get<int>();
  cmd_params_.max_motor_rpm =
      this->get_parameter("max_motor_rpm").get_parameter_value().get<int>();
  cmd_params_.motor_rpm_deadzone = this->get_parameter("motor_rpm_deadzone")
                                       .get_parameter_value()
                                       .get<int>();

  cmd_params_.rpm_ratio =
      this->get_parameter("rpm_ratio").get_parameter_value().get<double>();
  cmd_params_.steer_ratio =
      this->get_parameter("steer_ratio").get_parameter_value().get<double>();
  control_handler_ = std::make_unique<ControlHandler>(cmd_params_);

  RCLCPP_INFO(this->get_logger(), "Joystick index: %d",
              driver_config_.js_index);
  RCLCPP_INFO(this->get_logger(), "VESC CAN interface name: %s",
              driver_config_.can_if_name.c_str());
  RCLCPP_INFO(this->get_logger(), "VESC ID: 0x%x", driver_config_.vesc_id);
  RCLCPP_INFO(this->get_logger(), "Cmd topic name: %s", cmd_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Min steer angle: %f",
              cmd_params_.min_steer_angle);
  RCLCPP_INFO(this->get_logger(), "Max steer angle: %f",
              cmd_params_.max_steer_angle);
  RCLCPP_INFO(this->get_logger(), "Neutral steer angle: %f",
              cmd_params_.neutral_steer_angle);
  RCLCPP_INFO(this->get_logger(), "Min motor RPM: %d",
              cmd_params_.min_motor_rpm);
  RCLCPP_INFO(this->get_logger(), "Max motor RPM: %d",
              cmd_params_.max_motor_rpm);
  RCLCPP_INFO(this->get_logger(), "RPM ratio: %f", cmd_params_.rpm_ratio);
  RCLCPP_INFO(this->get_logger(), "Steer ratio: %f", cmd_params_.steer_ratio);
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
//   std::cout << "voltage input: " << state.state.voltage_input << ", "
//             << "temp pcb: " << state.state.temperature_pcb << ", "
//             << "current motor: " << state.state.current_motor << ", "
//             << "current input: " << state.state.current_input << ", "
//             << "speed: " << state.state.speed << ", "
//             << "duty cycle: " << state.state.duty_cycle << ", "
        //     << "charge drawn: " << state.state.charge_drawn << ", "
        //     << "charge regen: " << state.state.charge_regen << ", "
        //     << "charge drawn: " << state.state.energy_drawn << ", "
        //     << "charge regen: " << state.state.energy_regen << ", "
        //     << "displacement: " << state.state.displacement << ", "
        //     << "dist traveled: " << state.state.distance_traveled << std::endl;

   static bool first_time = true;
   if(first_time) {
      first_time = false;
      t_ = Clock::now();
      return;
   }
   double dt = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t_).count() / 1000.0;
   t_ = Clock::now();
   if(dt == 0) return;
//    std::cout << "dt: " << dt << std::endl;
   double speed = state.state.speed / cmd_params_.rpm_ratio;
   double steer_angle = (active_cmd_.servo_angle - cmd_params_.neutral_steer_angle) / cmd_params_.steer_ratio;
   auto state_tf = propagator_.Propagate(robot_state_, RccarBicycleKinematics::control_type(speed, steer_angle), 
        0, dt, dt/10);
  robot_state_ = state_tf;

  RCLCPP_INFO(this->get_logger(), "speed: %f, angle: %f, odom: (%f, %f, %f)",
        speed, steer_angle*180.0/M_PI, robot_state_[0], robot_state_[1], robot_state_[2]);

  // publish odometry
  auto current_time = this->get_clock()->now();
  geometry_msgs::msg::Quaternion odom_quat =
        createQuaternionMsgFromYaw(robot_state_[2]);

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = current_time;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_link";

  odom_msg.pose.pose.position.x = robot_state_[0];
  odom_msg.pose.pose.position.y = robot_state_[1];
  odom_msg.pose.pose.orientation = odom_quat;

  odom_msg.twist.twist.linear.x = speed;
  odom_msg.twist.twist.angular.z = steer_angle;

  odom_pub_->publish(odom_msg);
}

void AutocarJsTeleop::TwistCallback(
    const geometry_msgs::msg::Twist& msg) const {
  control_handler_->UpdateCommand(msg);
//   RCLCPP_INFO(this->get_logger(), "Twist command (linear_x, angular_z): %f, %f",
//                msg.linear.x, msg.angular.z);
}

void AutocarJsTeleop::OnMainTimer() {
  if (joystick_->IsOpened()) {
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
        js_input_.speed_cmd = -motor_cmd * cmd_params_.max_motor_rpm;
      } else {
        js_input_.speed_cmd = motor_cmd * cmd_params_.min_motor_rpm;
      }
      if (std::abs(js_input_.speed_cmd) < cmd_params_.motor_rpm_deadzone) {
        js_input_.speed_cmd = 0;
      }

      double servo_cmd = joystick_->GetAxisState(JsAxis::kRX).value;
      if (servo_cmd >= 0.0) {
        js_input_.steering_cmd = cmd_params_.neutral_steer_angle +
                                 servo_cmd * (cmd_params_.max_steer_angle -
                                              cmd_params_.neutral_steer_angle);
      } else {
        js_input_.steering_cmd = cmd_params_.neutral_steer_angle +
                                 servo_cmd * (cmd_params_.neutral_steer_angle -
                                              cmd_params_.min_steer_angle);
      }
      if (std::abs(js_input_.steering_cmd - cmd_params_.neutral_steer_angle) <
          cmd_params_.steer_angle_deadzone) {
        js_input_.steering_cmd = cmd_params_.neutral_steer_angle;
      }
    } else {
      js_input_.speed_cmd = 0;
      js_input_.steering_cmd = cmd_params_.neutral_steer_angle;
    }

    if (joystick_->GetButtonState(JsButton::kMode)) {
      joystick_->SetJoystickRumble(0.1 * 0xFFFF, 0.5 * 0xFFFF);
    } else {
      joystick_->SetJoystickRumble(0.0 * 0xFFFF, 0.0 * 0xFFFF);
    }

    //    RCLCPP_INFO(this->get_logger(),
    //                "Deadman switch: %d, Speed cmd: %d, Steering cmd: %f",
    //                js_input_.deadman_switch, js_input_.speed_cmd,
    //                js_input_.steering_cmd);

    // set control source
    control_handler_->UpdateRcInput(js_input_);

    // send command to vesc
    active_cmd_ = control_handler_->GetCommand();
  } else {
    // no control allowed for safety
    active_cmd_.motor_rpm = 0;
    active_cmd_.servo_angle = cmd_params_.neutral_steer_angle;    
    RCLCPP_INFO(this->get_logger(), "=> No RC connected, robot stopped.");
  }

//   RCLCPP_INFO(this->get_logger(), "=> Motor RPM: %d, Servo angle: %f",
//         active_cmd_.motor_rpm, active_cmd_.servo_angle);

  vesc_->SetSpeed(active_cmd_.motor_rpm);
  vesc_->SetServo(active_cmd_.servo_angle);
}
}  // namespace xmotion

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(xmotion::AutocarJsTeleop)
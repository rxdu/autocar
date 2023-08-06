/*
 * autocar_teleop.cpp
 *
 * Created on 6/8/23 00:05 PM
 * Description:
 *
 * Copyright (c) 2023 Ruixiang Du (rdu)
 */

#include "autocar_teleop/autocar_teleop.hpp"

namespace xmotion {
AutocarTeleop::AutocarTeleop(const rclcpp::NodeOptions &options)
    : rclcpp::Node("autocar_teleop_node", options) {
  RCLCPP_INFO(this->get_logger(), "Creating AutocarTeleop node ...");

  main_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(20),
                              std::bind(&AutocarTeleop::OnMainTimer, this));

  RCLCPP_INFO(this->get_logger(), "AutocarTeleop node has been created.");
}

void AutocarTeleop::OnMainTimer() {
  RCLCPP_INFO(this->get_logger(), "Hello World!");
}
}  // namespace xmotion
/*
 * autocar_teleop.hpp
 *
 * Created on 5/8/23 23:57 PM
 * Description:
 *
 * Copyright (c) 2023 Ruixiang Du (rdu)
 */

#include "rclcpp/rclcpp.hpp"

namespace xmotion {
class AutocarTeleop : public rclcpp::Node {
 public:
  AutocarTeleop(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  private:
};
}
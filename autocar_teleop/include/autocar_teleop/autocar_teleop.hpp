/*
 * autocar_teleop.hpp
 *
 * Created on 5/8/23 23:57 PM
 * Description:
 *
 * Copyright (c) 2023 Ruixiang Du (rdu)
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace xmotion {
class AutocarTeleop : public rclcpp::Node {
 public:
  AutocarTeleop(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

 private:
  void OnMainTimer();
  rclcpp::TimerBase::SharedPtr main_timer_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};
}  // namespace xmotion
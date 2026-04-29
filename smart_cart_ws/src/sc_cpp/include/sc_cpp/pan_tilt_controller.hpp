#pragma once

#include <rclcpp/rclcpp.hpp>

#include "sc_interfaces/msg/person_bbox.hpp"
#include "sc_interfaces/msg/servo_control.hpp"

namespace sc_cpp
{

class PanTiltController : public rclcpp::Node
{
public:
  explicit PanTiltController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void bbox_callback(const sc_interfaces::msg::PersonBbox::SharedPtr msg);
  void publish_servo_command(uint8_t servo_id, double angle);

  rclcpp::Subscription<sc_interfaces::msg::PersonBbox>::SharedPtr bbox_sub_;
  rclcpp::Publisher<sc_interfaces::msg::ServoControl>::SharedPtr servo_pub_;

  double current_pan_ = 90.0;
  double current_tilt_ = 90.0;

  double pan_kp_;
  double tilt_kp_;
  double pan_min_;
  double pan_max_;
  double tilt_min_;
  double tilt_max_;
  double command_speed_;
  int image_width_;
  int image_height_;
};

}  // namespace sc_cpp

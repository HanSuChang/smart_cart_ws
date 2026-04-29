#include "sc_cpp/pan_tilt_controller.hpp"

#include <algorithm>

namespace sc_cpp
{

PanTiltController::PanTiltController(const rclcpp::NodeOptions & options)
: Node("pan_tilt_controller", options)
{
  this->declare_parameter("pan_kp", 0.08);
  this->declare_parameter("tilt_kp", 0.08);
  this->declare_parameter("pan_min", 0.0);
  this->declare_parameter("pan_max", 180.0);
  this->declare_parameter("tilt_min", 45.0);
  this->declare_parameter("tilt_max", 135.0);
  this->declare_parameter("pan_center", 90.0);
  this->declare_parameter("tilt_center", 90.0);
  this->declare_parameter("image_width", 640);
  this->declare_parameter("image_height", 480);
  this->declare_parameter("command_speed", 1.0);

  pan_kp_ = this->get_parameter("pan_kp").as_double();
  tilt_kp_ = this->get_parameter("tilt_kp").as_double();
  pan_min_ = this->get_parameter("pan_min").as_double();
  pan_max_ = this->get_parameter("pan_max").as_double();
  tilt_min_ = this->get_parameter("tilt_min").as_double();
  tilt_max_ = this->get_parameter("tilt_max").as_double();
  current_pan_ = this->get_parameter("pan_center").as_double();
  current_tilt_ = this->get_parameter("tilt_center").as_double();
  image_width_ = this->get_parameter("image_width").as_int();
  image_height_ = this->get_parameter("image_height").as_int();
  command_speed_ = this->get_parameter("command_speed").as_double();

  bbox_sub_ = this->create_subscription<sc_interfaces::msg::PersonBbox>(
    "/person_bbox", 10, std::bind(&PanTiltController::bbox_callback, this, std::placeholders::_1));

  servo_pub_ = this->create_publisher<sc_interfaces::msg::ServoControl>("/servo_control", 10);

  RCLCPP_INFO(this->get_logger(), "PanTiltController started.");
}

void PanTiltController::bbox_callback(const sc_interfaces::msg::PersonBbox::SharedPtr msg)
{
  if (msg->width <= 0 || msg->height <= 0) {
    return;
  }

  const double person_center_x = msg->x + (msg->width / 2.0);
  const double person_center_y = msg->y + (msg->height / 2.0);

  const double error_x = (image_width_ / 2.0) - person_center_x;
  const double error_y = (image_height_ / 2.0) - person_center_y;

  current_pan_ += error_x * pan_kp_;
  current_tilt_ += error_y * tilt_kp_;

  current_pan_ = std::clamp(current_pan_, pan_min_, pan_max_);
  current_tilt_ = std::clamp(current_tilt_, tilt_min_, tilt_max_);

  publish_servo_command(0, current_pan_);
  publish_servo_command(1, current_tilt_);
}

void PanTiltController::publish_servo_command(uint8_t servo_id, double angle)
{
  sc_interfaces::msg::ServoControl out_msg;
  out_msg.header.stamp = this->now();
  out_msg.servo_id = servo_id;
  out_msg.angle = static_cast<float>(angle);
  out_msg.speed = static_cast<float>(command_speed_);
  servo_pub_->publish(out_msg);
}

}  // namespace sc_cpp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sc_cpp::PanTiltController)

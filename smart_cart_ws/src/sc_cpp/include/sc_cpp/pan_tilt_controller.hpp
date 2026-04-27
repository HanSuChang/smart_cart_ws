#pragma once

#include <rclcpp/rclcpp.hpp>
#include "sc_interfaces/msg/person_bbox.hpp"
#include "sc_interfaces/msg/pan_tilt_angle.hpp"

namespace sc_cpp
{

class PanTiltController : public rclcpp::Node
{
public:
  explicit PanTiltController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // 파이썬 AI로부터 좌표를 받는 콜백
  void bbox_callback(const sc_interfaces::msg::PersonBbox::SharedPtr msg);

  // ROS 2 통신 단말기
  rclcpp::Subscription<sc_interfaces::msg::PersonBbox>::SharedPtr bbox_sub_;
  rclcpp::Publisher<sc_interfaces::msg::PanTiltAngle>::SharedPtr angle_pub_;

  // 현재 목 각도 상태 (기본값: 90도 정면)
  double current_pan_ = 90.0;
  double current_tilt_ = 90.0;

  // 실시간 튜닝을 위한 파라미터 변수
  double pan_kp_, tilt_kp_;
  int center_x_ref_, center_y_ref_;
};

}  // namespace sc_cpp
#pragma once

// pan_tilt_controller
// 역할: 사람(전신) bbox 좌표를 받아서 웹캠 서보(Pan/Tilt)를 회전시킴
//       사람이 화면 중앙에 오도록 자동 추적
//       ★ /cmd_vel은 발행하지 않음 (follow_controller만 발행)

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
  // 파이썬 AI로부터 좌표를 받는 콜백
  void bbox_callback(const sc_interfaces::msg::PersonBbox::SharedPtr msg);
  void publish_servo_command(uint8_t servo_id, double angle);

  // ROS 2 통신 단말기
  rclcpp::Subscription<sc_interfaces::msg::PersonBbox>::SharedPtr bbox_sub_;
  rclcpp::Publisher<sc_interfaces::msg::ServoControl>::SharedPtr servo_pub_;

  // 현재 목 각도 상태 (기본값: 90도 정면)
  double current_pan_ = 90.0;
  double current_tilt_ = 90.0;

  // 실시간 튜닝을 위한 파라미터 변수
  int center_x_ref_, center_y_ref_;
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

} 
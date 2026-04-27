#include "sc_cpp/pan_tilt_controller.hpp"
#include <algorithm> // std::clamp 사용

namespace sc_cpp
{

PanTiltController::PanTiltController(const rclcpp::NodeOptions & options)
: Node("pan_tilt_controller", options)
{
  // 1. 파라미터 선언 (subl . 이나 ros2 param set으로 수정 가능)
  this->declare_parameter("pan_kp", 0.04);      // 좌우 추종 민감도, 로봇 고개가 너무 느리게 반응하면? : ros2 param set /pan_tilt_controller pan_kp 0.08
  this->declare_parameter("tilt_kp", 0.03);     // 상하 추종 민감도
  this->declare_parameter("center_x_ref", 320); // 화면 가로 중앙점 (640 해상도 기준)
  this->declare_parameter("center_y_ref", 240); // 화면 세로 중앙점 (480 해상도 기준)

  pan_kp_ = this->get_parameter("pan_kp").as_double();
  tilt_kp_ = this->get_parameter("tilt_kp").as_double();
  center_x_ref_ = this->get_parameter("center_x_ref").as_int();
  center_y_ref_ = this->get_parameter("center_y_ref").as_int();

  // 2. 구독(Python AI의 좌표 받기) 및 발행(OpenCR로 각도 보내기)
  bbox_sub_ = this->create_subscription<sc_interfaces::msg::PersonBbox>(
    "/person_bbox", 10, std::bind(&PanTiltController::bbox_callback, this, std::placeholders::_1));
  
  angle_pub_ = this->create_publisher<sc_interfaces::msg::PanTiltAngle>("/pan_tilt_angle", 10);

  RCLCPP_INFO(this->get_logger(), "목 관절 제어기가 준비되었습니다. 파이썬 AI의 신호를 기다립니다.");
}

void PanTiltController::bbox_callback(const sc_interfaces::msg::PersonBbox::SharedPtr msg)
{
  // 1. 사람이 인식되지 않았을 때(좌표가 0이면)는 움직이지 않음
  if (msg->width == 0) return;

  // 2. 사람의 중심점 계산
  double person_center_x = msg->x + (msg->width / 2.0);
  double person_center_y = msg->y + (msg->height / 2.0);

  // 3. 오차(Error) 계산: "화면 중앙에서 얼마나 벗어났는가?"
  double error_x = center_x_ref_ - person_center_x;
  double error_y = center_y_ref_ - person_center_y;

  // 4. 각도 업데이트 (P-제어 적용)
  // 사람이 왼쪽에 있으면(error_x > 0) 팬 각도를 키우고, 오른쪽에 있으면 줄임
  current_pan_ += error_x * pan_kp_;
  current_tilt_ += error_y * tilt_kp_;

  // 5. 서보 모터 보호 (0~180도 제한)
  current_pan_ = std::clamp(current_pan_, 0.0, 180.0);
  current_tilt_ = std::clamp(current_tilt_, 0.0, 180.0);

  // 6. 최종 각도 발행 (OpenCR로 전달)
  sc_interfaces::msg::PanTiltAngle out_msg;
  out_msg.pan = static_cast<int>(current_pan_);
  out_msg.tilt = static_cast<int>(current_tilt_);
  
  angle_pub_->publish(out_msg);
}

}  // namespace sc_cpp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sc_cpp::PanTiltController)
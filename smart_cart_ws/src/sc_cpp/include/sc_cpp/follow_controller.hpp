#pragma once
// ================================================================
// follow_controller.hpp
// [C++ 담당] 사람 추종 PID 제어 노드
//
// 역할:
//   Python(person_tracker)이 publish하는 /person_bbox를 받아서
//   Kalman Filter로 예측 → PID로 /cmd_vel 생성
//
// Subscribe: /person_bbox (sc_interfaces/msg/PersonBbox)
//            /safety_stop (std_msgs/msg/Bool)
// Publish:   /cmd_vel (geometry_msgs/msg/Twist)
//
// 파라미터 (rqt_reconfigure로 실시간 변경 가능):
//   linear_kp, linear_ki, linear_kd
//   angular_kp, angular_ki, angular_kd
//   target_bbox_height (이 픽셀 높이에서 정지)
//   image_width, max_linear_vel, max_angular_vel
//   bbox_timeout_sec, control_freq_hz
// ================================================================

// TODO: 나중에 구현 예정

#include <rclcpp/rclcpp.hpp>

namespace sc_cpp
{

class FollowController : public rclcpp::Node
{
public:
  explicit FollowController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
};

}  // namespace sc_cpp

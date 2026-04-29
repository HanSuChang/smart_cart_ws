#pragma once

// ================================================================
// follow_controller.hpp
//
// [통합 버전] PID + Kalman + LiDAR 안전거리 + GUI 모드 전환
//
// [카메라]
//   탑싱크 TS-B7WQ30 1440p USB 웹캠
//   v4l2_camera → 640x480 → /webcam/image_raw
//
// [AI]
//   YOLOv8n + DeepSORT → /person_bbox
//
// [모드 전환]
//   GUI가 /smart_cart/mode 발행 → 이 노드가 구독
//     "follow"   → 사람 추종 (이 노드가 /cmd_vel 발행)
//     "navigate" → Nav2 자동 주행 (이 노드는 cmd_vel 안 보냄)
//     "idle"     → 정지
//
// [/cmd_vel 충돌 방지]
//   - mode == "follow" 일 때만 /cmd_vel 발행
//   - 그 외엔 Nav2가 /cmd_vel 발행하도록 양보
// ================================================================

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>      // ★ 추가: GUI 모드 토픽
#include "sc_interfaces/msg/person_bbox.hpp"
#include "sc_cpp/kalman_filter.hpp"

namespace sc_cpp
{

class FollowController : public rclcpp::Node
{
public:
  explicit FollowController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── 콜백 ──
  void bbox_callback(const sc_interfaces::msg::PersonBbox::SharedPtr msg);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void safety_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void mode_callback(const std_msgs::msg::String::SharedPtr msg);  // ★ 추가
  void control_loop();

  // ── ROS2 인터페이스 ──
  rclcpp::Subscription<sc_interfaces::msg::PersonBbox>::SharedPtr bbox_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;  // ★ 추가
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ── 칼만 필터 ──
  KalmanFilter kf_;

  // ── 상태 ──
  sc_interfaces::msg::PersonBbox::SharedPtr last_bbox_;
  rclcpp::Time last_bbox_time_;
  bool is_internal_safety_stop_ = false;
  bool is_emergency_stop_ = false;
  bool target_lost_ = true;
  std::string current_mode_ = "idle";  // ★ 추가: 현재 GUI 모드 (기본 idle)

  // ── PID ──
  double lin_error_prev_ = 0.0;
  double lin_error_integral_ = 0.0;
  double ang_error_prev_ = 0.0;
  double ang_error_integral_ = 0.0;

  // ── 파라미터 ──
  double linear_kp_, linear_ki_, linear_kd_;
  double angular_kp_, angular_ki_, angular_kd_;
  double target_bbox_height_;
  int image_width_;
  double max_linear_vel_, max_angular_vel_;
  double bbox_timeout_sec_;
  double control_freq_hz_;
  double min_safe_dist_;
  double prediction_timeout_sec_;

  void stop_robot();
};

}  // namespace sc_cpp
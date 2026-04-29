#pragma once

//.hpp (Header File): 설계도 / 선언
// 역할: 클래스가 어떤 변수와 함수를 가지고 있는지 '선언'만 하는 곳.

// 비유: 식당의 '메뉴판'과 같습니다. 어떤 음식을 파는지는 써있지만, 실제로 요리가 만들어지는 곳은 아님

// 특징:
// 다른 파일들이 이 클래스를 가져다 쓸 수 있도록 인터페이스를 제공.
// private, public 처럼 접근 권한을 설정하고 파라미터 변수들을 정의

// ================================================================
// ★ 통합 버전: follow_controller + person_follower 합친 것
//
// [카메라]
//   탑싱크 TS-B7WQ30 1440p USB 웹캠
//   v4l2_camera → 640x480 리사이즈 → /webcam/image_raw 발행
//
// [AI]
//   YOLOv8n (nano) + DeepSORT → person_tracker.py → /person_bbox
//
// [이 노드의 역할]
//   - PID 제어로 부드러운 속도 조절
//   - Kalman Filter(8-state)로 사각지대/순간 끊김 예측 보정
//   - LiDAR로 직접 안전거리 판단
//   - ★ /cmd_vel은 이 노드에서만 발행 (충돌 방지)
// ================================================================

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>  // LiDAR 직접 구독
#include <std_msgs/msg/bool.hpp>           // 외부 safety_stop 신호 수신용
#include "sc_interfaces/msg/person_bbox.hpp"
#include "sc_cpp/kalman_filter.hpp"        // 8-state Kalman Filter

namespace sc_cpp
{

class FollowController : public rclcpp::Node
{
public:
  explicit FollowController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── 콜백 함수 ──
  void bbox_callback(const sc_interfaces::msg::PersonBbox::SharedPtr msg);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void safety_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void control_loop();

  // ── ROS2 인터페이스 ──
  rclcpp::Subscription<sc_interfaces::msg::PersonBbox>::SharedPtr bbox_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ── 칼만 필터 객체 (8-state: x, y, w, h, vx, vy, vw, vh) ──
  KalmanFilter kf_;

  // ── 상태 변수 ──
  sc_interfaces::msg::PersonBbox::SharedPtr last_bbox_;
  rclcpp::Time last_bbox_time_;
  bool is_internal_safety_stop_ = false; // LiDAR 직접 감지 결과
  bool is_emergency_stop_ = false;       // 외부 /safety_stop 신호
  bool target_lost_ = true;             // 타겟 일시 분실 상태

  // ── PID 변수 (Linear) ──
  double lin_error_prev_ = 0.0;
  double lin_error_integral_ = 0.0;

  // ── PID 변수 (Angular) ──
  double ang_error_prev_ = 0.0;
  double ang_error_integral_ = 0.0;

  // ── 파라미터 ──
  double linear_kp_, linear_ki_, linear_kd_;
  double angular_kp_, angular_ki_, angular_kd_;
  double target_bbox_height_;  // 이 픽셀 높이 = 적정 추종 거리 (640x480 기준)
  int image_width_;            // 640 (v4l2_camera 리사이즈 후)
  double max_linear_vel_, max_angular_vel_;
  double bbox_timeout_sec_;
  double control_freq_hz_;
  double min_safe_dist_;          // LiDAR 안전 거리 (m)
  double prediction_timeout_sec_; // Kalman 예측만으로 추격 유지할 최대 시간

  // ── 유틸리티 ──
  void stop_robot();
};

}  // namespace sc_cpp
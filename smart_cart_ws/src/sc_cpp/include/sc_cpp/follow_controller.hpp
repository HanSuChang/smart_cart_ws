#pragma once

// =====================================================================
// follow_controller.hpp
//
// [기능]
//   - /person_bbox 구독 → Kalman + PID → /cmd_vel 발행
//   - 사용자와 80cm 거리 유지 (target_distance_m)
//   - 사각지대/카메라 밖으로 나가면 Kalman 예측만으로 짧게 유지 →
//     prediction_timeout_sec 초과 시 정지 + RECOVERING 상태 발행
//   - GUI(/smart_cart/mode) 가 'follow' 일 때만 /cmd_vel 발행 (Nav2와 충돌 방지)
//
// [Topic — Pub/Sub]
//   Subscribe:
//     /person_bbox      (sc_interfaces/PersonBbox)   — Python AI
//     /scan             (sensor_msgs/LaserScan)      — LDS-02
//     /safety_stop      (std_msgs/Bool)              — safety_monitor or GUI E-Stop
//     /smart_cart/mode  (std_msgs/String)            — GUI 모드
//
//   Publish:
//     /cmd_vel          (geometry_msgs/Twist)        — 주행
//     /tracker/state    (sc_interfaces/TrackerState) — GUI 시각화용
//     /follow_status    (std_msgs/String)            — idle/following/predicting/lost/recovering
// =====================================================================

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include "sc_interfaces/msg/person_bbox.hpp"
#include "sc_interfaces/msg/tracker_state.hpp"
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
  void mode_callback(const std_msgs::msg::String::SharedPtr msg);
  void control_loop();
  rcl_interfaces::msg::SetParametersResult on_param_change(
    const std::vector<rclcpp::Parameter> & params);

  // ── ROS2 인터페이스 ──
  rclcpp::Subscription<sc_interfaces::msg::PersonBbox>::SharedPtr bbox_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<sc_interfaces::msg::TrackerState>::SharedPtr tracker_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr follow_status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // ── 칼만 필터 ──
  KalmanFilter kf_;

  // ── 상태 ──
  sc_interfaces::msg::PersonBbox::SharedPtr last_bbox_;
  rclcpp::Time last_bbox_time_;
  bool is_internal_safety_stop_ = false;
  bool is_emergency_stop_ = false;
  bool target_lost_ = true;
  std::string current_mode_ = "idle";
  std::string follow_state_ = "idle"; // idle / following / predicting / lost / recovering

  // ── PID ──
  double lin_error_prev_ = 0.0;
  double lin_error_integral_ = 0.0;
  double ang_error_prev_ = 0.0;
  double ang_error_integral_ = 0.0;

  // ── 파라미터 ──
  double linear_kp_, linear_ki_, linear_kd_;
  double angular_kp_, angular_ki_, angular_kd_;
  double target_distance_m_;        // 0.8 m (80cm)
  double bbox_height_at_target_;    // 80cm 일 때 bbox 높이 (캘리브레이션)
  int    image_width_;
  double max_linear_vel_, max_angular_vel_;
  double bbox_timeout_sec_;
  double control_freq_hz_;
  double min_safe_dist_;
  double prediction_timeout_sec_;

  void stop_robot();
  void publish_tracker_state(double dist, double time_since);
  void publish_follow_status(const std::string & st);
};

}  // namespace sc_cpp

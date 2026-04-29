#include "sc_cpp/follow_controller.hpp"
#include <algorithm>

namespace sc_cpp
{

FollowController::FollowController(const rclcpp::NodeOptions & options)
: Node("follow_controller", options), kf_()
{
  // ── 파라미터 선언 ──
  this->declare_parameter("linear_kp",  0.002);
  this->declare_parameter("linear_ki",  0.0);
  this->declare_parameter("linear_kd",  0.0005);
  this->declare_parameter("angular_kp", 0.004);
  this->declare_parameter("angular_ki", 0.0);
  this->declare_parameter("angular_kd", 0.001);
  this->declare_parameter("target_bbox_height", 200.0);
  this->declare_parameter("image_width", 640);
  this->declare_parameter("max_linear_vel",  0.22);
  this->declare_parameter("max_angular_vel", 1.0);
  this->declare_parameter("bbox_timeout_sec",      1.5);
  this->declare_parameter("control_freq_hz",       20.0);
  this->declare_parameter("min_safe_dist",         0.25);
  this->declare_parameter("prediction_timeout_sec", 0.8);

  linear_kp_              = this->get_parameter("linear_kp").as_double();
  linear_ki_              = this->get_parameter("linear_ki").as_double();
  linear_kd_              = this->get_parameter("linear_kd").as_double();
  angular_kp_             = this->get_parameter("angular_kp").as_double();
  angular_ki_             = this->get_parameter("angular_ki").as_double();
  angular_kd_             = this->get_parameter("angular_kd").as_double();
  target_bbox_height_     = this->get_parameter("target_bbox_height").as_double();
  image_width_            = this->get_parameter("image_width").as_int();
  max_linear_vel_         = this->get_parameter("max_linear_vel").as_double();
  max_angular_vel_        = this->get_parameter("max_angular_vel").as_double();
  bbox_timeout_sec_       = this->get_parameter("bbox_timeout_sec").as_double();
  control_freq_hz_        = this->get_parameter("control_freq_hz").as_double();
  min_safe_dist_          = this->get_parameter("min_safe_dist").as_double();
  prediction_timeout_sec_ = this->get_parameter("prediction_timeout_sec").as_double();

  // ── 구독 ──
  bbox_sub_ = this->create_subscription<sc_interfaces::msg::PersonBbox>(
    "/person_bbox", 10,
    std::bind(&FollowController::bbox_callback, this, std::placeholders::_1));

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(),
    std::bind(&FollowController::scan_callback, this, std::placeholders::_1));

  safety_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/safety_stop", 10,
    std::bind(&FollowController::safety_callback, this, std::placeholders::_1));

  // ★ GUI 모드 토픽 구독 (follow / navigate / idle)
  mode_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/smart_cart/mode", 10,
    std::bind(&FollowController::mode_callback, this, std::placeholders::_1));

  // ── 발행 ──
  // ★ /cmd_vel은 mode == "follow" 일 때만 발행 (Nav2와 충돌 방지)
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // ── 제어 루프 ──
  auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / control_freq_hz_));
  timer_ = this->create_wall_timer(
    period, std::bind(&FollowController::control_loop, this));

  last_bbox_time_ = this->now();
  RCLCPP_INFO(this->get_logger(),
    "FollowController 시작 — 기본 모드: %s (GUI에서 'follow' 보내야 동작)",
    current_mode_.c_str());
}

// ════════════════════════════════════════════
//  scan_callback — LiDAR 직접 감시
// ════════════════════════════════════════════
void FollowController::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  float min_range = 3.5f;
  int n = static_cast<int>(msg->ranges.size());
  for (int i = 0; i < 15 && i < n; i++) {
    if (msg->ranges[i] > 0.1f)
      min_range = std::min(min_range, msg->ranges[i]);
    int j = n - 1 - i;
    if (j >= 0 && msg->ranges[j] > 0.1f)
      min_range = std::min(min_range, msg->ranges[j]);
  }
  is_internal_safety_stop_ = (min_range < static_cast<float>(min_safe_dist_));
}

// ════════════════════════════════════════════
//  safety_callback — 외부 비상 정지 신호
// ════════════════════════════════════════════
void FollowController::safety_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  is_emergency_stop_ = msg->data;
}

// ════════════════════════════════════════════
//  mode_callback — GUI 모드 전환 (★ 신규)
//    "follow":   사람 추종 활성화
//    "navigate": Nav2 자동 주행 (이 노드는 양보)
//    "idle":     완전 정지
// ════════════════════════════════════════════
void FollowController::mode_callback(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data != current_mode_) {
    RCLCPP_INFO(this->get_logger(),
      "MODE CHANGED: %s → %s", current_mode_.c_str(), msg->data.c_str());
    current_mode_ = msg->data;

    // 모드 변경 시 PID 누적 초기화 (다음 follow 시작할 때 튀는 거 방지)
    lin_error_integral_ = 0.0;
    ang_error_integral_ = 0.0;
  }
}

// ════════════════════════════════════════════
//  bbox_callback — Kalman 보정
// ════════════════════════════════════════════
void FollowController::bbox_callback(const sc_interfaces::msg::PersonBbox::SharedPtr msg)
{
  if (!msg->is_valid || msg->width <= 0) {
    return;
  }

  kf_.predict();
  kf_.update(
    static_cast<double>(msg->x),
    static_cast<double>(msg->y),
    static_cast<double>(msg->width),
    static_cast<double>(msg->height));

  last_bbox_ = msg;
  last_bbox_time_ = this->now();
  target_lost_ = false;
}

// ════════════════════════════════════════════
//  control_loop — 20Hz
// ════════════════════════════════════════════
void FollowController::control_loop()
{
  // ★ 1순위: GUI 모드 체크
  // mode != "follow" 이면 /cmd_vel 발행 안 함 (Nav2가 발행하도록 양보)
  if (current_mode_ != "follow") {
    return;  // 정지 명령도 안 보냄 - Nav2를 방해하지 않기 위함
  }

  // 2순위: 안전 정지
  if (is_internal_safety_stop_ || is_emergency_stop_) {
    stop_robot();
    return;
  }

  // 3순위: 데이터 유효성
  if (!last_bbox_ || !kf_.isInitialized()) {
    stop_robot();
    return;
  }

  auto now = this->now();
  double elapsed = (now - last_bbox_time_).seconds();

  if (elapsed > bbox_timeout_sec_) {
    target_lost_ = true;
    stop_robot();
    return;
  }

  // 사각지대 대응
  if (elapsed > 0.1) {
    kf_.predict();
    if (elapsed > prediction_timeout_sec_) {
      stop_robot();
      return;
    }
  }

  // Kalman state
  auto state = kf_.getState();
  double pred_x = state[0];
  double pred_w = state[2];
  double pred_h = state[3];

  double dt = 1.0 / control_freq_hz_;

  // Linear
  double lin_error = target_bbox_height_ - pred_h;
  lin_error_integral_ += lin_error * dt;
  double lin_deriv = (lin_error - lin_error_prev_) / dt;
  double v = (linear_kp_ * lin_error)
           + (linear_ki_ * lin_error_integral_)
           + (linear_kd_ * lin_deriv);
  lin_error_prev_ = lin_error;

  // Angular
  double bbox_center_x = pred_x + (pred_w / 2.0);
  double ang_error = (image_width_ / 2.0) - bbox_center_x;
  ang_error_integral_ += ang_error * dt;
  double ang_deriv = (ang_error - ang_error_prev_) / dt;
  double w_val = (angular_kp_ * ang_error)
               + (angular_ki_ * ang_error_integral_)
               + (angular_kd_ * ang_deriv);
  ang_error_prev_ = ang_error;

  // 발행
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x  = std::clamp(v,     -max_linear_vel_,  max_linear_vel_);
  cmd_vel.angular.z = std::clamp(w_val, -max_angular_vel_, max_angular_vel_);
  cmd_vel_pub_->publish(cmd_vel);
}

void FollowController::stop_robot()
{
  geometry_msgs::msg::Twist stop;
  cmd_vel_pub_->publish(stop);
  lin_error_integral_ = 0.0;
  ang_error_integral_ = 0.0;
}

}  // namespace sc_cpp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sc_cpp::FollowController)
#include "sc_cpp/follow_controller.hpp"
#include <algorithm>
#include <cmath>

namespace sc_cpp
{

FollowController::FollowController(const rclcpp::NodeOptions & options)
: Node("follow_controller", options), kf_()
{
  // ── 파라미터 선언 (실시간 튜닝 가능) ──
  this->declare_parameter("linear_kp",  1.5);
  this->declare_parameter("linear_ki",  0.0);
  this->declare_parameter("linear_kd",  0.3);
  this->declare_parameter("angular_kp", 0.004);
  this->declare_parameter("angular_ki", 0.0);
  this->declare_parameter("angular_kd", 0.001);

  // ★ 80cm 거리 유지
  this->declare_parameter("target_distance_m",     0.8);
  // bbox 높이가 150px 일 때 사용자가 약 80cm 거리에 있다고 가정 (640x480, 1.7m 신장, 카메라 +20도 틸트 기준)
  this->declare_parameter("bbox_height_at_target", 150.0);

  this->declare_parameter("image_width", 640);
  this->declare_parameter("max_linear_vel",  0.22);
  this->declare_parameter("max_angular_vel", 1.0);
  this->declare_parameter("bbox_timeout_sec",      1.5);
  this->declare_parameter("control_freq_hz",       20.0);
  this->declare_parameter("min_safe_dist",         0.25);
  // 사각지대 / 카메라 밖 → Kalman 예측만 유지하는 시간
  this->declare_parameter("prediction_timeout_sec", 1.5);

  linear_kp_              = this->get_parameter("linear_kp").as_double();
  linear_ki_              = this->get_parameter("linear_ki").as_double();
  linear_kd_              = this->get_parameter("linear_kd").as_double();
  angular_kp_             = this->get_parameter("angular_kp").as_double();
  angular_ki_             = this->get_parameter("angular_ki").as_double();
  angular_kd_             = this->get_parameter("angular_kd").as_double();
  target_distance_m_      = this->get_parameter("target_distance_m").as_double();
  bbox_height_at_target_  = this->get_parameter("bbox_height_at_target").as_double();
  image_width_            = this->get_parameter("image_width").as_int();
  max_linear_vel_         = this->get_parameter("max_linear_vel").as_double();
  max_angular_vel_        = this->get_parameter("max_angular_vel").as_double();
  bbox_timeout_sec_       = this->get_parameter("bbox_timeout_sec").as_double();
  control_freq_hz_        = this->get_parameter("control_freq_hz").as_double();
  min_safe_dist_          = this->get_parameter("min_safe_dist").as_double();
  prediction_timeout_sec_ = this->get_parameter("prediction_timeout_sec").as_double();

  // 파라미터 실시간 콜백
  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&FollowController::on_param_change, this, std::placeholders::_1));

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

  mode_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/smart_cart/mode", 10,
    std::bind(&FollowController::mode_callback, this, std::placeholders::_1));

  // ── 발행 ──
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  tracker_state_pub_ = this->create_publisher<sc_interfaces::msg::TrackerState>(
    "/tracker/state", 10);
  follow_status_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/follow_status", 10);

  auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / control_freq_hz_));
  timer_ = this->create_wall_timer(
    period, std::bind(&FollowController::control_loop, this));

  last_bbox_time_ = this->now();
  RCLCPP_INFO(this->get_logger(),
    "FollowController 시작 — target_distance=%.2fm, mode=%s",
    target_distance_m_, current_mode_.c_str());
}

// =====================================================================
// 파라미터 실시간 변경
// =====================================================================
rcl_interfaces::msg::SetParametersResult FollowController::on_param_change(
  const std::vector<rclcpp::Parameter> & params)
{
  for (const auto & p : params) {
    if (p.get_name() == "linear_kp")              linear_kp_ = p.as_double();
    else if (p.get_name() == "linear_ki")         linear_ki_ = p.as_double();
    else if (p.get_name() == "linear_kd")         linear_kd_ = p.as_double();
    else if (p.get_name() == "angular_kp")        angular_kp_ = p.as_double();
    else if (p.get_name() == "angular_ki")        angular_ki_ = p.as_double();
    else if (p.get_name() == "angular_kd")        angular_kd_ = p.as_double();
    else if (p.get_name() == "target_distance_m") target_distance_m_ = p.as_double();
    else if (p.get_name() == "bbox_height_at_target")
                                                  bbox_height_at_target_ = p.as_double();
    else if (p.get_name() == "max_linear_vel")    max_linear_vel_ = p.as_double();
    else if (p.get_name() == "max_angular_vel")   max_angular_vel_ = p.as_double();
    else if (p.get_name() == "bbox_timeout_sec")  bbox_timeout_sec_ = p.as_double();
    else if (p.get_name() == "min_safe_dist")     min_safe_dist_ = p.as_double();
    else if (p.get_name() == "prediction_timeout_sec")
                                                  prediction_timeout_sec_ = p.as_double();
  }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

// =====================================================================
// scan_callback — LiDAR 직접 감시
// =====================================================================
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

void FollowController::safety_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  is_emergency_stop_ = msg->data;
}

void FollowController::mode_callback(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data != current_mode_) {
    RCLCPP_INFO(this->get_logger(),
      "MODE CHANGED: %s → %s", current_mode_.c_str(), msg->data.c_str());
    current_mode_ = msg->data;
    lin_error_integral_ = 0.0;
    ang_error_integral_ = 0.0;
  }
}

// =====================================================================
// bbox_callback — Kalman update
// =====================================================================
void FollowController::bbox_callback(const sc_interfaces::msg::PersonBbox::SharedPtr msg)
{
  if (!msg->is_valid || msg->width <= 0) {
    return;  // 측정 안 됨 → predict만 (control_loop 에서)
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

// =====================================================================
// control_loop — 20Hz 메인 제어
// =====================================================================
void FollowController::control_loop()
{
  // 모드 체크
  if (current_mode_ != "follow") {
    if (follow_state_ != "idle") {
      follow_state_ = "idle";
      publish_follow_status(follow_state_);
    }
    return;
  }

  // 안전 정지
  if (is_internal_safety_stop_ || is_emergency_stop_) {
    stop_robot();
    follow_state_ = "lost";
    publish_follow_status(follow_state_);
    publish_tracker_state(0.0, 0.0);
    return;
  }

  if (!last_bbox_ || !kf_.isInitialized()) {
    stop_robot();
    follow_state_ = "lost";
    publish_follow_status(follow_state_);
    publish_tracker_state(0.0, 0.0);
    return;
  }

  auto now = this->now();
  double elapsed = (now - last_bbox_time_).seconds();

  // ── 측정값 끊김 처리: Kalman 예측 단계로 전환 ──
  std::string state = "following";
  if (elapsed > 0.1) {
    kf_.predict();
    state = "predicting";
  }

  if (elapsed > prediction_timeout_sec_) {
    // 카메라 밖 / 사각지대 너무 오래 → 정지하고 RECOVERING 상태
    // (Python 측 BoT-SORT + HSV가 다시 잡으면 bbox_callback에서 복귀)
    stop_robot();
    follow_state_ = "recovering";
    publish_follow_status(follow_state_);
    publish_tracker_state(0.0, elapse
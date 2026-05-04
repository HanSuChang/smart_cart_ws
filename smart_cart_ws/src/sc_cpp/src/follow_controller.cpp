#include "sc_cpp/follow_controller.hpp"
#include <rclcpp_components/register_node_macro.hpp>
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
// scan_callback — LiDAR 직접 감시 (전방 좌우 15도)
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
      "MODE CHANGED: %s -> %s", current_mode_.c_str(), msg->data.c_str());
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
  // 모드 체크 — follow 일 때만 /cmd_vel 발행 (Nav2 와 충돌 방지)
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
    // (Python 측 BoT-SORT + HSV 가 다시 잡으면 bbox_callback 에서 복귀)
    stop_robot();
    follow_state_ = "recovering";
    publish_follow_status(follow_state_);
    publish_tracker_state(0.0, elapsed);
    return;
  }

  // ── Kalman 상태 추출 ──
  auto kstate = kf_.getState();   // [x(좌상단), y(좌상단), w, h, vx, vy, vw, vh]
  double bbox_cx = kstate[0] + kstate[2] * 0.5;   // 중심 X
  double bbox_h  = kstate[3];
  if (bbox_h < 1.0) bbox_h = 1.0;  // div by zero 방지

  // ── 거리 추정 (bbox 높이 기반: 80cm @ 150px 기준 1/h 비례) ──
  double current_distance = target_distance_m_ * (bbox_height_at_target_ / bbox_h);

  // ── 선속도 PID (거리 오차) ──
  double lin_error = current_distance - target_distance_m_;   // 양수 = 멀다 → 전진
  lin_error_integral_ += lin_error;
  // 적분 와인드업 방지
  lin_error_integral_ = std::max(-2.0, std::min(2.0, lin_error_integral_));
  double lin_derivative = lin_error - lin_error_prev_;
  double linear_vel =
      linear_kp_ * lin_error
    + linear_ki_ * lin_error_integral_
    + linear_kd_ * lin_derivative;
  lin_error_prev_ = lin_error;

  // 너무 가까우면 후진 안 함 (안전) — 0 으로 클램프
  if (current_distance < target_distance_m_ * 0.7) {
    linear_vel = 0.0;
    lin_error_integral_ = 0.0;
  }
  linear_vel = std::max(-max_linear_vel_,
                        std::min(max_linear_vel_, linear_vel));

  // ── 각속도 PID (이미지 중앙과 bbox_cx 오차) ──
  double image_center = static_cast<double>(image_width_) / 2.0;
  double ang_error = image_center - bbox_cx;   // 양수 = 사람이 왼쪽 → 좌회전(+)
  ang_error_integral_ += ang_error;
  ang_error_integral_ = std::max(-500.0, std::min(500.0, ang_error_integral_));
  double ang_derivative = ang_error - ang_error_prev_;
  double angular_vel =
      angular_kp_ * ang_error
    + angular_ki_ * ang_error_integral_
    + angular_kd_ * ang_derivative;
  ang_error_prev_ = ang_error;

  angular_vel = std::max(-max_angular_vel_,
                         std::min(max_angular_vel_, angular_vel));

  // ── /cmd_vel 발행 ──
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x  = linear_vel;
  cmd.angular.z = angular_vel;
  cmd_vel_pub_->publish(cmd);

  // ── 상태 발행 ──
  if (follow_state_ != state) {
    follow_state_ = state;
    publish_follow_status(follow_state_);
  }
  publish_tracker_state(current_distance, elapsed);
}

// =====================================================================
// 보조 함수
// =====================================================================
void FollowController::stop_robot()
{
  geometry_msgs::msg::Twist cmd;   // 0/0
  cmd_vel_pub_->publish(cmd);
}

void FollowController::publish_tracker_state(double dist, double time_since)
{
  sc_interfaces::msg::TrackerState m;
  m.header.stamp = this->now();
  m.header.frame_id = "base_link";

  // 0: idle, 1: following, 2: predicting, 3: lost, 4: recovering
  uint8_t s = 0;
  if      (follow_state_ == "following")  s = 1;
  else if (follow_state_ == "predicting") s = 2;
  else if (follow_state_ == "lost")       s = 3;
  else if (follow_state_ == "recovering") s = 4;
  m.state = s;

  auto kstate = kf_.getState();
  for (int i = 0; i < 8; ++i) {
    m.kalman_state[i] = static_cast<float>(kstate[i]);
  }
  m.current_distance = static_cast<float>(dist);
  m.target_distance  = static_cast<float>(target_distance_m_);
  m.track_id = (last_bbox_) ? last_bbox_->track_id : -1;
  m.time_since_measurement = static_cast<float>(time_since);

  tracker_state_pub_->publish(m);
}

void FollowController::publish_follow_status(const std::string & st)
{
  std_msgs::msg::String m;
  m.data = st;
  follow_status_pub_->publish(m);
}

}  // namespace sc_cpp

// 컴포넌트 등록 (rclcpp_components_register_node 와 매칭)
RCLCPP_COMPONENTS_REGISTER_NODE(sc_cpp::FollowController)

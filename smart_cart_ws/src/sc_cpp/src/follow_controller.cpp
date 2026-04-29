#include "sc_cpp/follow_controller.hpp"
#include <algorithm>

namespace sc_cpp
{

FollowController::FollowController(const rclcpp::NodeOptions & options)
: Node("follow_controller", options), kf_()
{
  // ── 파라미터 선언 ──
  // subl . 로 yaml 열어서 영구 저장
  // ros2 param set /follow_controller <파라미터명> <값> 으로 실시간 변경
  // ros2 run rqt_reconfigure rqt_reconfigure 로 GUI 슬라이더 조절

  // PID 제어 파라미터 의미:
  // P (kp): 현재 오차에 비례해서 움직임 → 너무 높으면 오버슈트/진동
  // I (ki): 누적 오차 보정 → 너무 높으면 진동
  // D (kd): 급격한 변화 억제 (브레이크) → 너무 높으면 반응 느림

  this->declare_parameter("linear_kp",  0.002);
  this->declare_parameter("linear_ki",  0.0);
  this->declare_parameter("linear_kd",  0.0005);

  this->declare_parameter("angular_kp", 0.004);
  this->declare_parameter("angular_ki", 0.0);
  this->declare_parameter("angular_kd", 0.001);

  // 탑싱크 TS-B7WQ30 (1440p) → v4l2_camera → 640x480 리사이즈
  // target_bbox_height: 640x480 기준에서 이 픽셀 높이 = 적정 추종 거리
  // 값 크게 → 더 가까이 붙음 / 값 작게 → 더 멀리 떨어짐
  this->declare_parameter("target_bbox_height", 200.0);
  this->declare_parameter("image_width", 640);   // v4l2_camera 리사이즈 후 해상도

  this->declare_parameter("max_linear_vel",  0.22);  // TurtleBot3 Waffle Pi 최대 0.26 m/s
  this->declare_parameter("max_angular_vel", 1.0);

  this->declare_parameter("bbox_timeout_sec",      1.5);  // 완전 분실 판단 시간
  this->declare_parameter("control_freq_hz",       20.0); // 제어 루프 20Hz
  this->declare_parameter("min_safe_dist",         0.25); // LiDAR 안전거리 25cm
  this->declare_parameter("prediction_timeout_sec", 0.8); // Kalman 예측 유지 최대 시간

  // 값 읽기
  linear_kp_             = this->get_parameter("linear_kp").as_double();
  linear_ki_             = this->get_parameter("linear_ki").as_double();
  linear_kd_             = this->get_parameter("linear_kd").as_double();
  angular_kp_            = this->get_parameter("angular_kp").as_double();
  angular_ki_            = this->get_parameter("angular_ki").as_double();
  angular_kd_            = this->get_parameter("angular_kd").as_double();
  target_bbox_height_    = this->get_parameter("target_bbox_height").as_double();
  image_width_           = this->get_parameter("image_width").as_int();
  max_linear_vel_        = this->get_parameter("max_linear_vel").as_double();
  max_angular_vel_       = this->get_parameter("max_angular_vel").as_double();
  bbox_timeout_sec_      = this->get_parameter("bbox_timeout_sec").as_double();
  control_freq_hz_       = this->get_parameter("control_freq_hz").as_double();
  min_safe_dist_         = this->get_parameter("min_safe_dist").as_double();
  prediction_timeout_sec_ = this->get_parameter("prediction_timeout_sec").as_double();

  // ── 구독 설정 ──

  // /person_bbox: YOLOv8n + DeepSORT가 인식한 사람 bbox
  // ★ is_valid=false이면 사람 못 찾은 상태
  bbox_sub_ = this->create_subscription<sc_interfaces::msg::PersonBbox>(
    "/person_bbox", 10,
    std::bind(&FollowController::bbox_callback, this, std::placeholders::_1));

  // /scan: LDS-02 LiDAR → 전방 25cm 이내 장애물 직접 감지
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(),
    std::bind(&FollowController::scan_callback, this, std::placeholders::_1));

  // /safety_stop: 외부 비상 정지 신호 (이중 안전)
  safety_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/safety_stop", 10,
    std::bind(&FollowController::safety_callback, this, std::placeholders::_1));

  // ── 발행 설정 ──
  // ★ /cmd_vel은 이 노드에서만 발행 (다른 노드와 충돌 방지)
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // ── 제어 루프 타이머 ──
  auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / control_freq_hz_));
  timer_ = this->create_wall_timer(
    period, std::bind(&FollowController::control_loop, this));

  last_bbox_time_ = this->now();
  RCLCPP_INFO(this->get_logger(),
    "FollowController 시작 (PID + Kalman 8-state + LiDAR 안전거리)");
}

// ════════════════════════════════════════════
//  scan_callback
//  LDS-02: 0°=정면, 인덱스 0~14 (전방 오른쪽), 345~359 (전방 왼쪽)
//  전방 ±15° 범위에서 min_safe_dist 이내 장애물 감지 → 즉시 정지
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
//  safety_callback
//  외부 /safety_stop 신호 수신 (이중 안전)
// ════════════════════════════════════════════
void FollowController::safety_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  is_emergency_stop_ = msg->data;
}

// ════════════════════════════════════════════
//  bbox_callback
//  YOLOv8n + DeepSORT → /person_bbox 수신
//  ★ is_valid=false이면 무시 (사람 못 찾은 상태)
//  ★ is_valid=true이면 Kalman update (보정)
// ════════════════════════════════════════════
void FollowController::bbox_callback(const sc_interfaces::msg::PersonBbox::SharedPtr msg)
{
  // 사람 못 찾은 경우 → Kalman 예측 모드로 전환 (타임아웃까지 추격 유지)
  if (!msg->is_valid || msg->width <= 0) {
    return;
  }

  // Kalman 보정 (predict → update 순서)
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
//  control_loop (20Hz)
//  PID + Kalman 예측으로 /cmd_vel 계산 및 발행
// ════════════════════════════════════════════
void FollowController::control_loop()
{
  // ── 1순위: 안전 정지 ──
  if (is_internal_safety_stop_ || is_emergency_stop_) {
    stop_robot();
    return;
  }

  // ── 2순위: 초기화 전 or 완전 분실 ──
  if (!last_bbox_ || !kf_.isInitialized()) {
    stop_robot();
    return;
  }

  auto now = this->now();
  double elapsed = (now - last_bbox_time_).seconds();

  // 완전 분실 (bbox_timeout_sec 초과) → 정지
  if (elapsed > bbox_timeout_sec_) {
    target_lost_ = true;
    stop_robot();
    return;
  }

  // ── 사각지대 대응: Kalman 예측 모드 ──
  // 0.1초 이상 bbox 없으면 predict()만 호출해서 위치 예측
  if (elapsed > 0.1) {
    kf_.predict();
    // prediction_timeout_sec 초과 → 정지
    if (elapsed > prediction_timeout_sec_) {
      stop_robot();
      return;
    }
  }

  // ★ Kalman getState() = [x, y, w, h, vx, vy, vw, vh] (8개)
  // 위치 인덱스: 0=x, 1=y, 2=w, 3=h
  auto state = kf_.getState();
  double pred_x = state[0];
  double pred_w = state[2];
  double pred_h = state[3];

  double dt = 1.0 / control_freq_hz_;

  // ── Linear 제어 (전진/후진) ──
  // lin_error > 0: bbox 작음(멀다) → 전진
  // lin_error < 0: bbox 큼(가깝다) → 후진
  double lin_error = target_bbox_height_ - pred_h;
  lin_error_integral_ += lin_error * dt;
  double lin_deriv = (lin_error - lin_error_prev_) / dt;
  double v = (linear_kp_ * lin_error)
           + (linear_ki_ * lin_error_integral_)
           + (linear_kd_ * lin_deriv);
  lin_error_prev_ = lin_error;

  // ── Angular 제어 (좌/우 회전) ──
  // ang_error > 0: 사람이 화면 왼쪽 → 왼쪽 회전
  // ang_error < 0: 사람이 화면 오른쪽 → 오른쪽 회전
  double bbox_center_x = pred_x + (pred_w / 2.0);
  double ang_error = (image_width_ / 2.0) - bbox_center_x;
  ang_error_integral_ += ang_error * dt;
  double ang_deriv = (ang_error - ang_error_prev_) / dt;
  double w_val = (angular_kp_ * ang_error)
               + (angular_ki_ * ang_error_integral_)
               + (angular_kd_ * ang_deriv);
  ang_error_prev_ = ang_error;

  // ── 속도 제한 후 발행 ──
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x  = std::clamp(v,     -max_linear_vel_,  max_linear_vel_);
  cmd_vel.angular.z = std::clamp(w_val, -max_angular_vel_, max_angular_vel_);
  cmd_vel_pub_->publish(cmd_vel);
}

// ════════════════════════════════════════════
//  stop_robot
//  PID 누적값 초기화 포함 (재출발 시 튀는 것 방지)
// ════════════════════════════════════════════
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
#include "sc_cpp/follow_controller.hpp"
#include <algorithm>

namespace sc_cpp
{

FollowController::FollowController(const rclcpp::NodeOptions & options)
: Node("follow_controller", options), kf_() // Kalman Filter 초기화
{
  // ── 파라미터 선언 및 가져오기 (yaml 파일 기반) ──, subl . 으로 실시간 튜닝 가능
  // 더 편한 방법: rqt_reconfigure
  // 터미널 명령어(ros2 param set) 대신 GUI 슬라이더로 값을 조절하고 싶다면 다음 도구를 사용할 수 있습니다.
  //    ros2 run rqt_reconfigure rqt_reconfigure

  // 1. PID 제어 파라미터 (KP, KI, KD)의 의미와 기능
  // P (Proportional, 비례 게인 - kp): "현재 오차에 비례해서 움직여라" -> 오차 크면 큰 힘, 작으면 작은 힘.
  // I (Integral, 적분 게인 - ki): "누적된 오차를 해결해라" -> 미세 오차 합쳐서 보정 (과하면 진동).
  // D (Derivative, 미분 게인 - kd): "변화 속도를 억제해라" -> 급격한 변화 막는 브레이크 역할.

  this->declare_parameter("linear_kp", 0.0025);
  this->declare_parameter("linear_ki", 0.0);
  this->declare_parameter("linear_kd", 0.0005);

  this->declare_parameter("angular_kp", 0.005);
  this->declare_parameter("angular_ki", 0.0);
  this->declare_parameter("angular_kd", 0.001);

  this->declare_parameter("target_bbox_height", 280.0); // 기준값 (가까이 붙도록 설정)
  this->declare_parameter("image_width", 640);         // 카메라 가로 해상도

  this->declare_parameter("max_linear_vel", 0.22);     // 터틀봇 최대 권장 속도
  this->declare_parameter("max_angular_vel", 1.0);

  this->declare_parameter("bbox_timeout_sec", 1.5);    // 사람 인식 실패 시 완전 정지 대기 시간
  this->declare_parameter("control_freq_hz", 20.0);    // 제어 루프 주기 (20Hz)

  this->declare_parameter("min_safe_dist", 0.25);      // ★ 로봇이 직접 판단할 최소 안전 거리 (25cm)
  this->declare_parameter("prediction_timeout_sec", 0.8); // ★ Kalman 예측 유지 시간 (사각지대 대응)

  // 값 가져오기 (실시간 반영 준비)
  linear_kp_ = this->get_parameter("linear_kp").as_double();
  linear_ki_ = this->get_parameter("linear_ki").as_double();
  linear_kd_ = this->get_parameter("linear_kd").as_double();
  angular_kp_ = this->get_parameter("angular_kp").as_double();
  angular_ki_ = this->get_parameter("angular_ki").as_double();
  angular_kd_ = this->get_parameter("angular_kd").as_double();
  target_bbox_height_ = this->get_parameter("target_bbox_height").as_double();
  image_width_ = this->get_parameter("image_width").as_int();
  max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
  max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
  bbox_timeout_sec_ = this->get_parameter("bbox_timeout_sec").as_double();
  control_freq_hz_ = this->get_parameter("control_freq_hz").as_double();
  min_safe_dist_ = this->get_parameter("min_safe_dist").as_double();
  prediction_timeout_sec_ = this->get_parameter("prediction_timeout_sec").as_double();

  // ── 구독 및 발행 설정 ──
  bbox_sub_ = this->create_subscription<sc_interfaces::msg::PersonBbox>(
    "/person_bbox", 10, std::bind(&FollowController::bbox_callback, this, std::placeholders::_1));
  
  // ★ 개조 포인트: 외부 /safety_stop 대신 직접 LiDAR 데이터를 읽어 스스로 판단합니다.
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(), std::bind(&FollowController::scan_callback, this, std::placeholders::_1));

  // GUI나 외부 노드에서 주는 비상 정지 신호도 함께 수신 (이중 안전)
  safety_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/safety_stop", 10, std::bind(&FollowController::safety_callback, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // ── 제어 루프 타이머 설정 ──
  auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / control_freq_hz_));
  timer_ = this->create_wall_timer(period, std::bind(&FollowController::control_loop, this));

  last_bbox_time_ = this->now();
  RCLCPP_INFO(this->get_logger(), "FollowController(통합 버전: PID + Kalman 예측)가 성공적으로 시작되었습니다.");
}

void FollowController::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // 정면 ±15도 범위만 직접 감시하여 진짜 위험할 때만 브레이크를 겁니다.
  float min_range = 3.5;
  for (int i = 0; i < 15; i++) {
    if (msg->ranges[i] > 0.1) min_range = std::min(min_range, msg->ranges[i]);
    if (msg->ranges[359-i] > 0.1) min_range = std::min(min_range, msg->ranges[359-i]);
  }
  is_internal_safety_stop_ = (min_range < min_safe_dist_);
}

void FollowController::safety_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  is_emergency_stop_ = msg->data;
}

void FollowController::bbox_callback(const sc_interfaces::msg::PersonBbox::SharedPtr msg)
{
  if (msg->width > 0) {
    // ★ Kalman 필터에 측정값 입력 (보정 단계) - person_follower의 핵심 기능
    kf_.predict();
    kf_.update(msg->x, msg->y, msg->width, msg->height);

    last_bbox_ = msg;
    last_bbox_time_ = this->now();
    target_lost_ = false;
  }
}

void FollowController::control_loop()
{
  geometry_msgs::msg::Twist cmd_vel;

  // 1순위: 어떤 종류든 '정지' 신호가 있으면 즉시 멈춤 (내부 LiDAR 감지 OR 외부 비상 신호)
  if (is_internal_safety_stop_ || is_emergency_stop_) {
    stop_robot();
    return;
  }

  // 2. 데이터 유효성 및 타임아웃 체크
  auto now = this->now();
  double elapsed_since_bbox = (now - last_bbox_time_).seconds();

  // 한 번도 사람을 본 적 없으면 정지
  if (!last_bbox_) {
    stop_robot();
    return;
  }

  // ★ 너무 오래 안 보이면 완전 분실 → 정지
  if (elapsed_since_bbox > bbox_timeout_sec_) {
    target_lost_ = true;
    stop_robot();
    return;
  }

  // ★ 잠깐 안 보이는 사각지대 상황 → Kalman 예측만으로 추격 유지
  if (elapsed_since_bbox > 0.1) { // 0.1초 이상 지연 시 예측 모드 활성화
    kf_.predict();  // 보정 없이 예측만 계속 (사람이 가려졌을 때)
    if (elapsed_since_bbox > prediction_timeout_sec_) {
        stop_robot();
        return;
    }
  }

  // ★ Kalman으로 보정/예측된 좌표 사용
  auto state = kf_.getState();  // [x, y, w, h]
  double pred_x = state[0];
  double pred_w = state[2];
  double pred_h = state[3];

  double dt = 1.0 / control_freq_hz_;

  // 3. Linear 제어 (전진/후진)
  double lin_error = target_bbox_height_ - pred_h;
  
  // PID 계산: 현재 오차(P), 오차 누적(I), 오차 변화량(D)
  lin_error_integral_ += lin_error * dt;
  double lin_derivative = (lin_error - lin_error_prev_) / dt;
  double v = (linear_kp_ * lin_error) + (linear_ki_ * lin_error_integral_) + (linear_kd_ * lin_derivative);
  
  cmd_vel.linear.x = std::clamp(v, -max_linear_vel_, max_linear_vel_);
  lin_error_prev_ = lin_error;

  // 4. Angular 제어 (좌/우 회전)
  double bbox_center_x = pred_x + (pred_w / 2.0);
  double ang_error = (image_width_ / 2.0) - bbox_center_x;
  ang_error_integral_ += ang_error * dt;
  double ang_derivative = (ang_error - ang_error_prev_) / dt;

  double w_val = (angular_kp_ * ang_error) + (angular_ki_ * ang_error_integral_) + (angular_kd_ * ang_derivative);
  cmd_vel.angular.z = std::clamp(w_val, -max_angular_vel_, max_angular_vel_);
  ang_error_prev_ = ang_error;

  // 5. 속도 발행
  cmd_vel_pub_->publish(cmd_vel);
}

void FollowController::stop_robot()
{
  geometry_msgs::msg::Twist stop_msg;
  cmd_vel_pub_->publish(stop_msg);
  // 에러 누적값 초기화 (다시 시작할 때 로봇이 튀지 않도록 함)
  lin_error_integral_ = 0.0;
  ang_error_integral_ = 0.0;
}

}  // namespace sc_cpp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sc_cpp::FollowController)
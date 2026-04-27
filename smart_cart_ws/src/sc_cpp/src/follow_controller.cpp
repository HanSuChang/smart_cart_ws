#include "sc_cpp/follow_controller.hpp"
#include <algorithm>

namespace sc_cpp
{

FollowController::FollowController(const rclcpp::NodeOptions & options)
: Node("follow_controller", options)
{
  // ── 파라미터 선언 및 가져오기 (yaml 파일 기반) ──, subl . 으로 실시간 튜닝 가능
  // 더 편한 방법: rqt_reconfigure
  // 터미널 명령어(ros2 param set) 대신 GUI 슬라이더로 값을 조절하고 싶다면 다음 도구를 사용할 수 있습니다.
  //   ros2 run rqt_reconfigure rqt_reconfigure

// 1. PID 제어 파라미터 (KP, KI, KD)의 의미와 기능
// PID 제어는 '목표'와 '현재 상태'의 차이(오차)를 줄여나가는 알고리즘입니다.


// P (Proportional, 비례 게인 - kp): "현재 오차에 비례해서 움직여라"

// 기능: 오차가 크면 큰 힘을, 작으면 작은 힘을 줍니다.

// 영향: 이 값이 너무 낮으면 로봇이 굼뜨게 반응하고, 너무 높으면 목표물을 지나치거나 좌우로 심하게 흔들립니다(오버슈트).
//===============================================================================================================================

// I (Integral, 적분 게인 - ki): "누적된 오차를 해결해라"

// 기능: 시간이 지나도 사라지지 않는 미세한 오차를 합쳐서 힘을 보탭니다.

// 영향: 로봇이 목표 거리 근처에서 멈추지 못하고 미세하게 떨어져 있을 때 이를 끝까지 밀어주는 역할을 하지만, 과하면 로봇이 목표물 주변에서 계속 진동할 수 있습니다.

//===============================================================================================================================
// D (Derivative, 미분 게인 - kd): "변화 속도를 억제해라"

// 기능: 오차가 빠르게 줄어들면 브레이크를 걸어 급격한 변화를 막습니다.

// 영향: 로봇의 출렁거림(진동)을 억제하고 부드럽게 멈추도록 돕습니다.

// declare_parameter (subl . 로 실시간 속도, 해상도, 높이 등등등 PID 수정 가능)
// 실시간으로 로봇 움직임 고치는 법 : 터미널에 ros2 param set /follow_controller linear_kp 0.003
// 라고 예로들어서 뒤에 값만 고쳐서 쓰고 엔터를 치면 linear_kp_ 변수값이 즉시 0.003으로 바뀜. (코드 수정 X, 빌드 X, 재시작 X)

// 여러 값을 편하게 마우스로 조절하고 싶을 때 (rqt)
// 방법: ros2 run rqt_reconfigure rqt_reconfigure를 실행
// 결과: 슬라이더가 나타납니다. 마우스로 드래그하면 로봇의 속도나 민감도가 실시간으로 변함

// 찾아낸 최적의 값을 '영구히' 저장하고 싶을 때 (YAML)
// 방법: subl로 follow_params.yaml을 열어 숫자를 0.0025로 고치고 저장합니다.
// 결과: 다음에 로봇을 켤 때 자동으로 이 최적의 값으로 시작

// 기능 자체가 없을 때는 VS CODE 들어와서  원하는 declare_parameter 추가

//===============================================================================================================================
  this->declare_parameter("linear_kp", 0.0025); // PID 할 때 P - p (살짝 상향)
  this->declare_parameter("linear_ki", 0.0);    // PID 할 때 I - i
  this->declare_parameter("linear_kd", 0.0005); // PID 할 때 D -d

  this->declare_parameter("angular_kp", 0.005); // PID 할 때 P - p (살짝 상향)
  this->declare_parameter("angular_ki", 0.0);    // PID 할 때 I - i
  this->declare_parameter("angular_kd", 0.001); // PID 할 때 D -d

  this->declare_parameter("target_bbox_height", 280.0); // 기준값 (조금 더 가까이 붙도록 수정)
  this->declare_parameter("image_width", 640); // 카메라의 가로 해상도

  this->declare_parameter("max_linear_vel", 0.22); // 최대 속도 제한
  this->declare_parameter("max_angular_vel", 1.0); // 최대 속도 제한

  this->declare_parameter("bbox_timeout_sec", 1.5); // 사람 인식 실패 시 대기 시간
  this->declare_parameter("control_freq_hz", 20.0); // 제어 루프 주기

  this->declare_parameter("min_safe_dist", 0.25);  // ★ 신규: 로봇이 직접 판단할 최소 안전 거리 (25cm)

  this->declare_parameter("prediction_timeout_sec", 0.8);  // ★ 신규: Kalman 예측만으로 추격 유지할 최대 시간 (사각지대 대응)

  // 값 가져오기
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

  // 1. 데이터 유효성 및 타임아웃 체크
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
  bool using_prediction = (elapsed_since_bbox > prediction_timeout_sec_);
  if (using_prediction) {
    kf_.predict();  // 보정 없이 예측만 계속 (사람이 가려졌을 때)
  }

  // ★ Kalman으로 보정/예측된 좌표 사용
  auto state = kf_.getState();  // [x, y, w, h]
  double pred_x = state[0];
  double pred_w = state[2];
  double pred_h = state[3];

  double dt = 1.0 / control_freq_hz_;

  // 2. Linear 제어 (전진/후진) - 안전 거리 체크 포함
  double lin_error = target_bbox_height_ - pred_h;
  
  // ★ 외부 간섭 없이 스스로 판단: 진짜 부딪힐 것 같지 않으면 전진!
  if (!is_internal_safety_stop_) {
    lin_error_integral_ += lin_error * dt;
    double lin_derivative = (lin_error - lin_error_prev_) / dt;
    double v = (linear_kp_ * lin_error) + (linear_ki_ * lin_error_integral_) + (linear_kd_ * lin_derivative);
    cmd_vel.linear.x = std::clamp(v, -max_linear_vel_, max_linear_vel_);
  } else {
    cmd_vel.linear.x = 0.0; // 최소 안전 거리 확보 시 정지
  }
  lin_error_prev_ = lin_error;

  // 3. Angular 제어 (좌/우 회전)
  double bbox_center_x = pred_x + (pred_w / 2.0);
  double ang_error = (image_width_ / 2.0) - bbox_center_x;
  ang_error_integral_ += ang_error * dt;
  double ang_derivative = (ang_error - ang_error_prev_) / dt;

  double w = (angular_kp_ * ang_error) + (angular_ki_ * ang_error_integral_) + (angular_kd_ * ang_derivative);
  cmd_vel.angular.z = std::clamp(w, -max_angular_vel_, max_angular_vel_);
  ang_error_prev_ = ang_error;

  // 4. 속도 발행
  cmd_vel_pub_->publish(cmd_vel);
}

void FollowController::stop_robot()
{
  geometry_msgs::msg::Twist stop_msg;
  cmd_vel_pub_->publish(stop_msg);
  lin_error_integral_ = 0.0;
  ang_error_integral_ = 0.0;
}

}  // namespace sc_cpp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sc_cpp::FollowController)
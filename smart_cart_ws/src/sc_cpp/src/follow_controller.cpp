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
  this->declare_parameter("linear_kp", 0.002); // PID 할 때 P - p
  this->declare_parameter("linear_ki", 0.0);   // PID 할 때 I - i
  this->declare_parameter("linear_kd", 0.0005); // PID 할 때 D -d

  this->declare_parameter("angular_kp", 0.004); // PID 할 때 P - p
  this->declare_parameter("angular_ki", 0.0);   // PID 할 때 I - i
  this->declare_parameter("angular_kd", 0.001); // PID 할 때 D -d

  this->declare_parameter("target_bbox_height", 300.0); // 로봇과 사람 사이의 거리 기준값
  this->declare_parameter("image_width", 640); // 카메라의 가로 해상도

  this->declare_parameter("max_linear_vel", 0.22); // 최대 속도 제한 너무 빠르게 달려 사람과 충돌하거나 급회전하여 넘어지는 것을 방지하는 안전장치
  this->declare_parameter("max_angular_vel", 1.0); // 최대 속도 제한

  this->declare_parameter("bbox_timeout_sec", 1.5); // 사람 인식 실패 시 대기 시간
  this->declare_parameter("control_freq_hz", 20.0); // 제어 루프 주기입니다. 초당 20번(20Hz)씩 현재 위치를 확인하고 속도 명령을 계산하라는 뜻



  // declare_parameter로 등록했던 파라미터들의 실제 값을 가져와서 C++ 변수에 할당하는 과정
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


  // ── 구독 및 발행 설정 ──
  bbox_sub_ = this->create_subscription<sc_interfaces::msg::PersonBbox>(
    "/person_bbox", 10, std::bind(&FollowController::bbox_callback, this, std::placeholders::_1));
  
  safety_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/safety_stop", 10, std::bind(&FollowController::safety_callback, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // ── 제어 루프 타이머 설정 ──
  auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / control_freq_hz_));
  timer_ = this->create_wall_timer(period, std::bind(&FollowController::control_loop, this));

  last_bbox_time_ = this->now();
  RCLCPP_INFO(this->get_logger(), "FollowController가 성공적으로 시작되었습니다.");
}

void FollowController::bbox_callback(const sc_interfaces::msg::PersonBbox::SharedPtr msg)
{
  last_bbox_ = msg;
  last_bbox_time_ = this->now();
}

void FollowController::safety_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  is_safety_stop_ = msg->data;
}

void FollowController::control_loop()
{
  geometry_msgs::msg::Twist cmd_vel;

  // 1. 긴급 정지 상태 체크
  if (is_safety_stop_) {
    stop_robot();
    return;
  }

  // 2. 데이터 유효성 및 타임아웃 체크
  auto now = this->now();
  if (!last_bbox_ || (now - last_bbox_time_).seconds() > bbox_timeout_sec_) {
    stop_robot();
    return;
  }

  double dt = 1.0 / control_freq_hz_;

  // 3. Linear 제어 (전진/후진)
  // 에러 = 목표 높이 - 현재 높이 (높이가 작을수록 멀리 있는 것)
  double lin_error = target_bbox_height_ - static_cast<double>(last_bbox_->height);
  lin_error_integral_ += lin_error * dt;
  double lin_derivative = (lin_error - lin_error_prev_) / dt;
  
  double v = (linear_kp_ * lin_error) + (linear_ki_ * lin_error_integral_) + (linear_kd_ * lin_derivative);
  cmd_vel.linear.x = std::clamp(v, -max_linear_vel_, max_linear_vel_);
  lin_error_prev_ = lin_error;

  // 4. Angular 제어 (좌/우 회전)
  // 에러 = 이미지 중앙 x - bbox 중앙 x
  double bbox_center_x = last_bbox_->x + (last_bbox_->width / 2.0);
  double ang_error = (image_width_ / 2.0) - bbox_center_x;
  ang_error_integral_ += ang_error * dt;
  double ang_derivative = (ang_error - ang_error_prev_) / dt;

  double w = (angular_kp_ * ang_error) + (angular_ki_ * ang_error_integral_) + (angular_kd_ * ang_derivative);
  cmd_vel.angular.z = std::clamp(w, -max_angular_vel_, max_angular_vel_);
  ang_error_prev_ = ang_error;

  // 5. 속도 발행
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
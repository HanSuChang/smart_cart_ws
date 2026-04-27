//.hpp (Header File): 설계도 / 선언
// 역할: 클래스가 어떤 변수와 함수를 가지고 있는지 '선언'만 하는 곳.

// 비유: 식당의 '메뉴판'과 같습니다. 어떤 음식을 파는지는 써있지만, 실제로 요리가 만들어지는 곳은 아님

// 특징:
// 다른 파일들이 이 클래스를 가져다 쓸 수 있도록 인터페이스를 제공.
// private, public 처럼 접근 권한을 설정하고 파라미터 변수들을 정의



#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include "sc_interfaces/msg/person_bbox.hpp"

namespace sc_cpp
{

class FollowController : public rclcpp::Node
{
public:
  explicit FollowController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── 콜백 함수 ──
  void bbox_callback(const sc_interfaces::msg::PersonBbox::SharedPtr msg);
  void safety_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void control_loop();

  // ── ROS2 인터페이스 ──
  rclcpp::Subscription<sc_interfaces::msg::PersonBbox>::SharedPtr bbox_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ── 상태 변수 ──
  sc_interfaces::msg::PersonBbox::SharedPtr last_bbox_;
  rclcpp::Time last_bbox_time_;
  bool is_safety_stop_ = false;

  // ── PID 변수 (Linear) ──
  double lin_error_prev_ = 0.0;
  double lin_error_integral_ = 0.0;

  // ── PID 변수 (Angular) ──
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

  // ── 유틸리티 ──
  void stop_robot();
};

}  // namespace sc_cpp
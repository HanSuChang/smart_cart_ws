#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>

namespace sc_cpp
{

class SafetyMonitor : public rclcpp::Node
{
public:
  explicit SafetyMonitor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── 콜백 및 로직 ──
  // 콜백: "데이터가 도착하면(이벤트), 내가 정해둔 함수를 실행해라" 라는 예약 시스템
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  
  // ── ROS2 인터페이스 ──
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_; // scan_sub : 구독자: 귀(듣는 역할)
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_pub_; // safety_pub_ : 발행자: 입(말하는 역할)

  // ── 파라미터 ──
  double stop_distance_m_;  // 멈출 거리 (예: 0.3m)
  int scan_angle_range_;    // 전방 감시 각도 (예: 30도 -> 전방 -15 ~ +15도) , -15도라는 수치는 360도 체계에서 345도와 같은 곳
};

}  // namespace sc_cpp
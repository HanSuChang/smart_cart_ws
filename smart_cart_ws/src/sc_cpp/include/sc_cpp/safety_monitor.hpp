#pragma once
// ================================================================
// safety_monitor.hpp
// [C++ 담당] LiDAR(LDS-02) 장애물 감지 + 긴급정지
//
// Subscribe: /scan (sensor_msgs/msg/LaserScan)
// Publish:   /cmd_vel (zero 강제), /safety_stop (std_msgs/msg/Bool)
//
// 파라미터:
//   stop_distance_m (0.35m 이내 → 정지)
//   resume_distance_m (0.50m 이상 → 재출발, 히스테리시스)
//   front_angle_deg (전방 ±30° 범위만 감시)
// ================================================================

// TODO: 나중에 구현 예정

#include <rclcpp/rclcpp.hpp>

namespace sc_cpp
{

class SafetyMonitor : public rclcpp::Node
{
public:
  explicit SafetyMonitor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
};

}  // namespace sc_cpp

// ================================================================
// safety_monitor.cpp
// [C++ 담당] LiDAR 장애물 긴급정지 구현
// TODO: 전체 로직 구현 예정
// ================================================================

#include "sc_cpp/safety_monitor.hpp"

namespace sc_cpp
{

SafetyMonitor::SafetyMonitor(const rclcpp::NodeOptions & options)
: Node("safety_monitor", options)
{
  // TODO: declare_parameter (stop_distance_m, front_angle_deg, resume_distance_m)
  // TODO: /scan subscriber
  // TODO: /cmd_vel publisher (긴급정지용)
  // TODO: /safety_stop publisher (follow_controller에 정지 신호)
  RCLCPP_INFO(get_logger(), "SafetyMonitor 노드 생성됨 (TODO: 구현 필요)");
}

}  // namespace sc_cpp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sc_cpp::SafetyMonitor)

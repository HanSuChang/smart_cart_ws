// ================================================================
// follow_controller.cpp
// [C++ 담당] 사람 추종 PID 제어 노드 구현
// TODO: 전체 로직 구현 예정
// ================================================================

#include "sc_cpp/follow_controller.hpp"

namespace sc_cpp
{

FollowController::FollowController(const rclcpp::NodeOptions & options)
: Node("follow_controller", options)
{
  // TODO: declare_parameter (PID 게인, target_bbox_height 등)
  // TODO: /person_bbox subscriber
  // TODO: /safety_stop subscriber
  // TODO: /cmd_vel publisher
  // TODO: 제어 루프 타이머 (20Hz)
  // TODO: Kalman Filter 초기화
  RCLCPP_INFO(get_logger(), "FollowController 노드 생성됨 (TODO: 구현 필요)");
}

}  // namespace sc_cpp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sc_cpp::FollowController)

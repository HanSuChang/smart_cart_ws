// ================================================================
// status_publisher.cpp
// [C++ 담당] 카트 전체 상태 종합 발행 구현
// TODO: 전체 로직 구현 예정
// ================================================================

#include "sc_cpp/status_publisher.hpp"

namespace sc_cpp
{

StatusPublisher::StatusPublisher(const rclcpp::NodeOptions & options)
: Node("status_publisher", options)
{
  // TODO: 각 토픽 subscriber
  // TODO: /cart_status publisher
  // TODO: 주기적 상태 종합 타이머 (5Hz)
  RCLCPP_INFO(get_logger(), "StatusPublisher 노드 생성됨 (TODO: 구현 필요)");
}

}  // namespace sc_cpp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sc_cpp::StatusPublisher)

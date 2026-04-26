// ================================================================
// lid_controller.cpp
// [C++ 담당] 바구니 뚜껑 서보 제어 구현
// TODO: 전체 로직 구현 예정
// ================================================================

#include "sc_cpp/lid_controller.hpp"

namespace sc_cpp
{

LidController::LidController(const rclcpp::NodeOptions & options)
: Node("lid_controller", options)
{
  // TODO: declare_parameter (confidence_threshold, confirm_frames, open_angle 등)
  // TODO: /item_detected subscriber
  // TODO: /servo_control publisher (servo_id=2)
  // TODO: /item_confirm publisher
  // TODO: 연속 감지 카운터 로직
  // TODO: 뚜껑 자동 닫기 타이머
  RCLCPP_INFO(get_logger(), "LidController 노드 생성됨 (TODO: 구현 필요)");
}

}  // namespace sc_cpp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sc_cpp::LidController)

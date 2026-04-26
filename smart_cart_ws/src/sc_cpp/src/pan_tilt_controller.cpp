// ================================================================
// pan_tilt_controller.cpp
// [C++ 담당] USB 웹캠 Pan/Tilt 서보 제어 구현
// TODO: 전체 로직 구현 예정
// ================================================================

#include "sc_cpp/pan_tilt_controller.hpp"

namespace sc_cpp
{

PanTiltController::PanTiltController(const rclcpp::NodeOptions & options)
: Node("pan_tilt_controller", options)
{
  // TODO: declare_parameter (pan_kp, tilt_kp, center각도, min/max 등)
  // TODO: /person_bbox subscriber
  // TODO: /servo_control publisher (servo_id=0,1)
  // TODO: bbox 중심 → 화면 중앙 오차 → 서보 각도 계산
  RCLCPP_INFO(get_logger(), "PanTiltController 노드 생성됨 (TODO: 구현 필요)");
}

}  // namespace sc_cpp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sc_cpp::PanTiltController)

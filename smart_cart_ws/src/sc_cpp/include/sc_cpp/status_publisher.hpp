#pragma once
// ================================================================
// status_publisher.hpp
// [C++ 담당] 카트 전체 상태 종합 발행 → GUI(sc_gui)에서 표시
//
// Subscribe: /person_bbox, /item_detected, /scan, /safety_stop
// Publish:   /cart_status (sc_interfaces/msg/CartStatus)
// ================================================================

// TODO: 나중에 구현 예정

#include <rclcpp/rclcpp.hpp>

namespace sc_cpp
{

class StatusPublisher : public rclcpp::Node
{
public:
  explicit StatusPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
};

}  // namespace sc_cpp

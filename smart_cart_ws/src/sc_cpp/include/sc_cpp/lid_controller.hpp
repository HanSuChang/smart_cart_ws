#pragma once
// ================================================================
// lid_controller.hpp
// [C++ 담당] 바구니 뚜껑 서보(MG996R) 제어
//
// 동작 시나리오:
//   1. RPi 카메라가 Roboflow 학습 물체를 인식 → /item_detected publish
//   2. 이 노드가 수신 → 연속 N프레임 확정 → 뚜껑 열기 신호
//   3. /servo_control publish (servo_id=2, angle=open_angle)
//   4. open_duration_sec 후 자동으로 뚜껑 닫기
//   5. 확정된 물체명을 /item_confirm publish
//
//   ※ 결제 시스템:
//      현재 계획: 안드로이드 앱에서 상품 QR 스캔 → 결제
//      ROS2와는 별도 프로젝트. 추후 연동 방식은 변경될 수 있음.
//      변경 시 이 노드에 결제 확인 subscriber 추가 가능.
//
// Subscribe: /item_detected (sc_interfaces/msg/ItemDetected)
// Publish:   /servo_control (sc_interfaces/msg/ServoControl)
//            /item_confirm (std_msgs/msg/String)
//
// 파라미터:
//   confidence_threshold, confirm_frames
//   open_angle, close_angle, open_duration_sec
// ================================================================

// TODO: 나중에 구현 예정

#include <rclcpp/rclcpp.hpp>

namespace sc_cpp
{

class LidController : public rclcpp::Node
{
public:
  explicit LidController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
};

}  // namespace sc_cpp

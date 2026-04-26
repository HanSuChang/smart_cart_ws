#pragma once
// ================================================================
// pan_tilt_controller.hpp
// [C++ 담당] USB 웹캠 Pan/Tilt 서보(MG996R x2 + 브라켓) 제어
//
// 하드웨어:
//   - MG996R 서보 2개 + MG995/MG996용 메탈 브라켓 1개
//   - servo_id 0: Pan (좌우 회전)
//   - servo_id 1: Tilt (상하 회전)
//   - OpenCR GPIO에서 PWM 제어
//
// 동작:
//   /person_bbox의 중심 좌표를 받아서
//   화면 중앙과의 오차 → Pan/Tilt 각도 보정
//   사람이 화면 중앙에 오도록 서보가 물리적으로 웹캠을 회전시킴
//
// Subscribe: /person_bbox (sc_interfaces/msg/PersonBbox)
// Publish:   /servo_control (sc_interfaces/msg/ServoControl)
//
// 파라미터:
//   pan_kp, tilt_kp (비례 게인)
//   pan_center, tilt_center (서보 중립 각도)
//   pan_min, pan_max, tilt_min, tilt_max (각도 제한)
//   image_width, image_height
// ================================================================

// TODO: 나중에 구현 예정

#include <rclcpp/rclcpp.hpp>

namespace sc_cpp
{

class PanTiltController : public rclcpp::Node
{
public:
  explicit PanTiltController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
};

}  // namespace sc_cpp

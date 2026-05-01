#pragma once
// =====================================================================
// lid_controller.hpp
//
// [기능]
//   1. /item_detected 연속 confirm_frames 이상 감지 → 뚜껑 OPEN
//      open_duration_sec 후 자동 CLOSE
//   2. /smart_cart/destination "toilet" 수신 시 즉시 뚜껑 CLOSE
//      (화장실 이동 시 안전을 위해 뚜껑이 닫혀야 함)
//      ※ 하드웨어 미연결 — 일단 토픽 발행만 (ServoControl)
//   3. /payment/event "paid" 수신 시 뚜껑 CLOSE (결제 완료 후 잠금)
//
// [Topic — Pub/Sub]
//   Subscribe:
//     /item_detected           (sc_interfaces/ItemDetected)
//     /basket/event            (sc_interfaces/BasketEvent)   바구니 비전 노드
//     /smart_cart/destination  (std_msgs/String)             GUI 이동 목적지
//     /payment/event           (sc_interfaces/PaymentEvent)
//
//   Publish:
//     /servo_control           (sc_interfaces/ServoControl, servo_id=2)
//     /item_confirm            (std_msgs/String)             확정된 물체명
//     /lid_state               (std_msgs/String)             "open" | "closed"
// =====================================================================

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "sc_interfaces/msg/item_detected.hpp"
#include "sc_interfaces/msg/servo_control.hpp"
#include "sc_interfaces/msg/payment_event.hpp"
#include "sc_interfaces/msg/basket_event.hpp"

namespace sc_cpp
{

class LidController : public rclcpp::Node
{
public:
  explicit LidController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void item_callback(const sc_interfaces::msg::ItemDetected::SharedPtr msg);
  void basket_callback(const sc_interfaces::msg::BasketEvent::SharedPtr msg);
  void destination_callback(const std_msgs::msg::String::SharedPtr msg);
  void payment_callback(const sc_interfaces::msg::PaymentEvent::SharedPtr msg);

  void open_lid(const std::string & reason);
  void close_lid(const std::string & reason);
  void publish_servo(double angle);
  void publish_lid_state(const std::string & st);
  void on_close_timer();

  // ── Sub ──
  rclcpp::Subscription<sc_interfaces::msg::ItemDetected>::SharedPtr item_sub_;
  rclcpp::Subscription<sc_interfaces::msg::BasketEvent>::SharedPtr basket_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dest_sub_;
  rclcpp::Subscription<sc_interfaces::msg::PaymentEvent>::SharedPtr payment_sub_;

  // ── Pub ──
  rclcpp::Publisher<sc_interfaces::msg::ServoControl>::SharedPtr servo_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr confirm_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lid_state_pub_;

  rclcpp::TimerBase::SharedPtr close_timer_;

  // ── 파라미터 ──
  double confidence_threshold_;
  int    confirm_frames_;
  double open_angle_;
  double close_angle_;
  double open_duration_sec_;
  double command_speed_;

  // ── 상태 ──
  std::string last_item_;
  int  consecutive_count_ = 0;
  bool lid_open_ = false;
};

}  // namespace sc_cpp

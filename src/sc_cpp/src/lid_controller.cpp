// =====================================================================
// lid_controller.cpp
//
// ※ 하드웨어 미연결: MG996R 서보 제어용 servo_id=2 ServoControl 메시지를
//    /servo_control 로 발행하기만 함. 실제 서보 동작은 OpenCR/Arduino 펌웨어
//    또는 micro-ROS 노드가 /servo_control 을 구독해서 실행.
// =====================================================================

#include "sc_cpp/lid_controller.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace sc_cpp
{

LidController::LidController(const rclcpp::NodeOptions & options)
: Node("lid_controller", options)
{
  // ── 파라미터 ──
  this->declare_parameter("confidence_threshold", 0.70);
  this->declare_parameter("confirm_frames", 5);
  this->declare_parameter("open_angle", 120.0);
  this->declare_parameter("close_angle", 0.0);
  this->declare_parameter("open_duration_sec", 5.0);
  this->declare_parameter("command_speed", 1.0);

  confidence_threshold_ = this->get_parameter("confidence_threshold").as_double();
  confirm_frames_       = this->get_parameter("confirm_frames").as_int();
  open_angle_           = this->get_parameter("open_angle").as_double();
  close_angle_          = this->get_parameter("close_angle").as_double();
  open_duration_sec_    = this->get_parameter("open_duration_sec").as_double();
  command_speed_        = this->get_parameter("command_speed").as_double();

  // ── Sub ──
  item_sub_ = this->create_subscription<sc_interfaces::msg::ItemDetected>(
    "/item_detected", 10,
    std::bind(&LidController::item_callback, this, std::placeholders::_1));

  basket_sub_ = this->create_subscription<sc_interfaces::msg::BasketEvent>(
    "/basket/event", 10,
    std::bind(&LidController::basket_callback, this, std::placeholders::_1));

  dest_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/smart_cart/destination", 10,
    std::bind(&LidController::destination_callback, this, std::placeholders::_1));

  payment_sub_ = this->create_subscription<sc_interfaces::msg::PaymentEvent>(
    "/payment/event", 10,
    std::bind(&LidController::payment_callback, this, std::placeholders::_1));

  // ── Pub ──
  servo_pub_ = this->create_publisher<sc_interfaces::msg::ServoControl>(
    "/servo_control", 10);
  confirm_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/item_confirm", 10);
  lid_state_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/lid_state", 10);

  // 시작 시 닫힘 상태 초기화
  close_lid("init");

  RCLCPP_INFO(this->get_logger(),
    "LidController 시작 — confirm=%d, open=%.1f°, close=%.1f°",
    confirm_frames_, open_angle_, close_angle_);
}

// ─────────────────────────────────────────────────────────
// 1. /item_detected — 연속 N프레임 확정 시 OPEN
// ─────────────────────────────────────────────────────────
void LidController::item_callback(const sc_interfaces::msg::ItemDetected::SharedPtr msg)
{
  if (!msg->in_basket_zone) return;
  if (msg->confidence < confidence_threshold_) {
    consecutive_count_ = 0;
    return;
  }

  if (msg->item_name == last_item_) {
    consecutive_count_++;
  } else {
    last_item_ = msg->item_name;
    consecutive_count_ = 1;
  }

  if (consecutive_count_ >= confirm_frames_ && !lid_open_) {
    open_lid("item_confirm:" + last_item_);
    std_msgs::msg::String confirm_msg;
    confirm_msg.data = last_item_;
    confirm_pub_->publish(confirm_msg);
    consecutive_count_ = 0;

    auto period = std::chrono::milliseconds(
      static_cast<int>(open_duration_sec_ * 1000));
    close_timer_ = this->create_wall_timer(
      period, std::bind(&LidController::on_close_timer, this));
  }
}

// ─────────────────────────────────────────────────────────
// 2. /basket/event — 바구니 비전 노드의 OpenCV 이벤트
//    분류기 미정 — 일단 'insert' 시 뚜껑 임시 OPEN 가능 (옵션)
// ─────────────────────────────────────────────────────────
void LidController::basket_callback(const sc_interfaces::msg::BasketEvent::SharedPtr msg)
{
  // 분류기가 정해지면 msg->item_name + confidence 로 추가 확정 가능
  RCLCPP_DEBUG(this->get_logger(),
    "[basket] event=%s motion=%d", msg->event_type.c_str(), msg->motion_pixels);
  // (현재는 로그만, 분류 모델 결정되면 여기서도 OPEN 트리거)
}

// ─────────────────────────────────────────────────────────
// 3. /smart_cart/destination — GUI 이동 목적지
//    "toilet"  → 즉시 뚜껑 CLOSE (안전)
//    "charger" → CLOSE (선택)
// ─────────────────────────────────────────────────────────
void LidController::destination_callback(const std_msgs::msg::String::SharedPtr msg)
{
  const std::string & dest = msg->data;
  RCLCPP_INFO(this->get_logger(), "[destination] %s 수신", dest.c_str());
  if (dest == "toilet" || dest == "charger") {
    if (close_timer_) {
      close_timer_->cancel();
      close_timer_.reset();
    }
    close_lid("destination:" + dest);
  }
}

// ─────────────────────────────────────────────────────────
// 4. /payment/event — 결제 완료 시 뚜껑 잠금
// ─────────────────────────────────────────────────────────
void LidController::payment_callback(const sc_interfaces::msg::PaymentEvent::SharedPtr msg)
{
  if (msg->event == "paid") {
    if (close_timer_) {
      close_timer_->cancel();
      close_timer_.reset();
    }
    close_lid("payment_paid");
    RCLCPP_INFO(this->get_logger(),
      "결제 완료 — 뚜껑 잠금 (총액=%d원)", msg->total_price);
  }
}

// ─────────────────────────────────────────────────────────
// 보조 함수
// ─────────────────────────────────────────────────────────
void LidController::open_lid(const std::string & reason)
{
  publish_servo(open_angle_);
  lid_open_ = true;
  publish_lid_state("open");
  RCLCPP_INFO(this->get_logger(), ">>> 뚜껑 OPEN [%s]", reason.c_str());
}

void LidController::close_lid(const std::string & reason)
{
  publish_servo(close_angle_);
  lid_open_ = false;
  publish_lid_state("closed");
  RCLCPP_INFO(this->get_logger(), ">>> 뚜껑 CLOSE [%s]", reason.c_str());
}

void LidController::on_close_timer()
{
  if (close_timer_) {
    close_timer_->cancel();
    close_timer_.reset();
  }
  close_lid("auto_close_timer");
}

void LidController::publish_servo(double angle)
{
  sc_interfaces::msg::ServoControl m;
  m.header.stamp = this->now();
  m.servo_id = 2;
  m.angle = static_cast<float>(angle);
  m.speed = static_cast<float>(command_speed_);
  servo_pub_->publish(m);
}

void LidController::publish_lid_state(const std::string & st)
{
  std_msgs::msg::String m;
  m.data = st;
  lid_state_pub_->publish(m);
}

}  // namespace sc_cpp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sc_cpp::LidController)

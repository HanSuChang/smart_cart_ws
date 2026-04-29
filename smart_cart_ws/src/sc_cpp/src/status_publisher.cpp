#include "sc_cpp/status_publisher.hpp"
#include <algorithm>

namespace sc_cpp
{

StatusPublisher::StatusPublisher(const rclcpp::NodeOptions & options)
: Node("status_publisher", options)
{
  // 상태 초기화
  current_status_.is_following = false;
  current_status_.item_count = 0;
  current_status_.emergency_stop = false;

// ================================================================
// 데이터 흐름 설명
// ================================================================
//
// 영상 획득: USB 웹캠 (/dev/video0) → v4l2_camera → /webcam/image_raw
//
// 사람 인식: person_tracker.py (YOLOv8n + DeepSORT)
//           → /person_bbox 토픽 발행
//
// 물체 인식: item_classifier.py (Roboflow 모델)
//           → /item_detected 토픽 발행
//
// 이 노드는 위 토픽들을 모아서 GUI(/cart_status)에 통합 발행
//
// ================================================================

  // ── 구독 설정 ──

  // 사람 추적 정보: YOLOv8n + DeepSORT가 인식한 사람 bbox
  // is_valid=false이면 사람 못 찾은 상태
  bbox_sub_ = this->create_subscription<sc_interfaces::msg::PersonBbox>(
    "/person_bbox", 10,
    std::bind(&StatusPublisher::bbox_callback, this, std::placeholders::_1));

  // 물체 인식 정보: Roboflow 모델이 인식한 물체 이름 + 개수
  item_sub_ = this->create_subscription<sc_interfaces::msg::ItemDetected>(
    "/item_detected", 10,
    std::bind(&StatusPublisher::item_callback, this, std::placeholders::_1));

  // 장애물 거리: LDS-02 LiDAR 데이터 → 주변 최소 거리 계산
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(),
    std::bind(&StatusPublisher::scan_callback, this, std::placeholders::_1));

  // 안전 상태: 비상 정지 신호 모니터링
  safety_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/safety_stop", 10,
    std::bind(&StatusPublisher::safety_callback, this, std::placeholders::_1));

  // 현재 속도: follow_controller가 발행하는 /cmd_vel
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    std::bind(&StatusPublisher::cmd_vel_callback, this, std::placeholders::_1));

  // ── 발행 설정 ──
  status_pub_ = this->create_publisher<sc_interfaces::msg::CartStatus>("/cart_status", 10);

  // 5Hz 주기로 상태 통합 발행
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&StatusPublisher::publish_status, this));

  RCLCPP_INFO(this->get_logger(), "StatusPublisher: GUI용 통합 상태 노드가 시작되었습니다.");
}

void StatusPublisher::bbox_callback(const sc_interfaces::msg::PersonBbox::SharedPtr msg)
{
  // ★ is_valid=false이면 사람 못 찾은 상태 → 추종 중단 표시
  if (!msg->is_valid) {
    current_status_.is_following = false;
    return;
  }

  // ★ track_id 사용 (PersonBbox.msg 필드명)
  current_status_.tracked_person_id = msg->track_id;

  // bbox 높이 기반 거리 추정 (bbox 높이 클수록 가까움)
  // 640x480 해상도 기준 보정값 300.0
  if (msg->height > 0) {
    current_status_.person_distance = 300.0 / static_cast<float>(msg->height);
  }

  last_bbox_time_ = this->now();
  current_status_.is_following = true;
}

void StatusPublisher::item_callback(const sc_interfaces::msg::ItemDetected::SharedPtr msg)
{
  current_status_.last_item_name = msg->item_name;
  current_status_.item_count++;
}

void StatusPublisher::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // 전방 장애물 최소 거리 계산
  float min_dist = msg->range_max;
  for (auto dist : msg->ranges) {
    if (dist > msg->range_min && dist < min_dist) min_dist = dist;
  }
  current_status_.obstacle_distance = min_dist;
}

void StatusPublisher::safety_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  current_status_.emergency_stop = msg->data;
}

void StatusPublisher::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  current_status_.linear_vel = msg->linear.x;
  current_status_.angular_vel = msg->angular.z;
}

void StatusPublisher::publish_status()
{
  // 사람 인식 타임아웃 체크 (1초 이상 안 보이면 추종 중단 표시)
  if ((this->now() - last_bbox_time_).seconds() > 1.0) {
    current_status_.is_following = false;
  }

  current_status_.header.stamp = this->now();
  status_pub_->publish(current_status_);
}

}  // namespace sc_cpp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sc_cpp::StatusPublisher)
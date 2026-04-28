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

  // ── 구독 설정 ──
  bbox_sub_ = this->create_subscription<sc_interfaces::msg::PersonBbox>(
    "/person_bbox", 10, std::bind(&StatusPublisher::bbox_callback, this, std::placeholders::_1));
  
  item_sub_ = this->create_subscription<sc_interfaces::msg::ItemDetected>(
    "/item_detected", 10, std::bind(&StatusPublisher::item_callback, this, std::placeholders::_1));

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(), std::bind(&StatusPublisher::scan_callback, this, std::placeholders::_1));

  safety_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/safety_stop", 10, std::bind(&StatusPublisher::safety_callback, this, std::placeholders::_1));

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10, std::bind(&StatusPublisher::cmd_vel_callback, this, std::placeholders::_1));

  // ── 발행 설정 ──
  status_pub_ = this->create_publisher<sc_interfaces::msg::CartStatus>("/cart_status", 10);

  // 5Hz 주기로 상태 통합 발행
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(200), std::bind(&StatusPublisher::publish_status, this));

  RCLCPP_INFO(this->get_logger(), "StatusPublisher: GUI용 통합 상태 노드가 시작되었습니다.");
}

void StatusPublisher::bbox_callback(const sc_interfaces::msg::PersonBbox::SharedPtr msg)
{
  current_status_.tracked_person_id = msg->id;
  // 간단한 거리 추정 (bbox 높이가 클수록 가까움)
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
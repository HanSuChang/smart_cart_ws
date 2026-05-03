#include "sc_cpp/status_publisher.hpp"
#include <algorithm>

namespace sc_cpp
{

StatusPublisher::StatusPublisher(const rclcpp::NodeOptions & options)
: Node("status_publisher", options)
{
  // 1. 상태 및 시간 초기화
  current_status_.is_following = false;
  current_status_.item_count = 0;
  current_status_.emergency_stop = false;
  
  // ★ 중요: 노드 시작 시점을 마지막 인식 시간으로 초기화하여 에러 방지
  last_bbox_time_ = this->now();

  // 2. 구독 설정
  bbox_sub_ = this->create_subscription<sc_interfaces::msg::PersonBbox>(
    "/person_bbox", 10,
    std::bind(&StatusPublisher::bbox_callback, this, std::placeholders::_1));

  item_sub_ = this->create_subscription<sc_interfaces::msg::ItemDetected>(
    "/item_detected", 10,
    std::bind(&StatusPublisher::item_callback, this, std::placeholders::_1));

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(),
    std::bind(&StatusPublisher::scan_callback, this, std::placeholders::_1));

  safety_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/safety_stop", 10,
    std::bind(&StatusPublisher::safety_callback, this, std::placeholders::_1));

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    std::bind(&StatusPublisher::cmd_vel_callback, this, std::placeholders::_1));

  // 3. 발행 설정
  status_pub_ = this->create_publisher<sc_interfaces::msg::CartStatus>("/cart_status", 10);

  // 5Hz 주기로 상태 통합 발행 (200ms)
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&StatusPublisher::publish_status, this));

  RCLCPP_INFO(this->get_logger(), "StatusPublisher: GUI용 통합 상태 노드가 시작되었습니다.");
}

void StatusPublisher::bbox_callback(const sc_interfaces::msg::PersonBbox::SharedPtr msg)
{
  // is_valid=false이면 사람을 놓친 상태임
  if (!msg->is_valid) {
    current_status_.is_following = false;
    return;
  }

  current_status_.tracked_person_id = msg->track_id;

  // bbox 높이 기반 거리 추정
  if (msg->height > 0) {
    current_status_.person_distance = 300.0 / static_cast<float>(msg->height);
  }

  // ★ 현재 시간을 저장하여 타임아웃 체크에 사용
  last_bbox_time_ = this->now(); 
  current_status_.is_following = true;
}

void StatusPublisher::item_callback(const sc_interfaces::msg::ItemDetected::SharedPtr msg)
{
  current_status_.last_item_name = msg->item_name;
  current_status_.item_count++; // 인식될 때마다 카운트 증가
}

void StatusPublisher::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
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
  // 사람 인식 타임아웃 체크 (마지막으로 본 지 1초 넘으면 추종 중단)
  if ((this->now() - last_bbox_time_).seconds() > 1.0) {
    current_status_.is_following = false;
  }

  current_status_.header.stamp = this->now();
  status_pub_->publish(current_status_);
}

}  // namespace sc_cpp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sc_cpp::StatusPublisher)
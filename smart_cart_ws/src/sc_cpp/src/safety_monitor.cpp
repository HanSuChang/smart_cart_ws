#include "sc_cpp/safety_monitor.hpp"

namespace sc_cpp
{

SafetyMonitor::SafetyMonitor(const rclcpp::NodeOptions & options)
: Node("safety_monitor", options)
{
  // 파라미터 선언 (실시간 튜닝 가능)
  this->declare_parameter("stop_distance_m", 0.4);   // 40cm 앞 장애물 감지
  this->declare_parameter("scan_angle_range", 40);   // 전방 40도 범위 감시

  stop_distance_m_ = this->get_parameter("stop_distance_m").as_double();
  scan_angle_range_ = this->get_parameter("scan_angle_range").as_int();

  // LDS-02 데이터 구독 및 정지 신호 발행
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(), std::bind(&SafetyMonitor::scan_callback, this, std::placeholders::_1));
  
  safety_pub_ = this->create_publisher<std_msgs::msg::Bool>("/safety_stop", 10);

  RCLCPP_INFO(this->get_logger(), "Safety Monitor가 가동되었습니다. (감시 거리: %.2fm)", stop_distance_m_);
}

void SafetyMonitor::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std_msgs::msg::Bool stop_msg;
  stop_msg.data = false;  // 기본값은 '주행 가능'

  // LDS-02는 0도가 정면입니다.
  // 전방 범위를 확인 (예: 340도~360도 및 0도~20도)
  int half_range = scan_angle_range_ / 2;
  int num_points = msg->ranges.size();

  for (int i = 0; i < num_points; ++i) {
    // 현재 인덱스를 각도로 변환 (LDS-02 인덱스 1개가 보통 1도)
    if (i <= half_range || i >= (num_points - half_range)) {
      double distance = msg->ranges[i];

      // 유효한 데이터이면서 정지 거리보다 가까운 경우
      if (distance > msg->range_min && distance < stop_distance_m_) {
        stop_msg.data = true;
        break;
      }
    }
  }

  // 결과 발행 (follow_controller가 이를 구독함)
  safety_pub_->publish(stop_msg);
}

}  // namespace sc_cpp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sc_cpp::SafetyMonitor)
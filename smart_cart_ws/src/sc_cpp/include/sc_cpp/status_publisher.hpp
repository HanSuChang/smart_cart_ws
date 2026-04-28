#pragma once

#include <rclcpp/rclcpp.hpp> // ROS2 C++ 개발의 가장 핵심이 되는 헤더입니다. 노드 생성, 퍼블리셔/서브스크라이버 선언, 로깅 등 모든 기본 기능을 포함
#include <std_msgs/msg/bool.hpp> // 표준 불리언(True/False) 메시지 타입입니다. 현재 코드에서는 긴급 정지(safety_stop) 여부를 확인
#include <geometry_msgs/msg/twist.hpp> // 로봇의 속도(선속도, 각속도)를 나타내는 표준 메시지입니다. /cmd_vel 토픽을 통해 로봇이 실제로 어떻게 움직이는지 데이터를 가져올 때 사용
#include <sensor_msgs/msg/laser_scan.hpp> // LiDAR 센서 데이터의 표준 형식 (전방 장애물과의 거리를 계산하기 위해 이 데이터를 읽어옴)

#include "sc_interfaces/msg/person_bbox.hpp" // YOLO가 인식한 사람의 좌표와 ID 정보를 담고 있는 메시지 헤더입니다. PersonBbox.msg 파일을 기반으로 생성
#include "sc_interfaces/msg/item_detected.hpp" // 바구니에 어떤 물건이 담겼는지(이름, 개수 등) 정보를 전달받기 위한 헤더

#include "sc_interfaces/msg/cart_status.hpp" // 노드의 최종 결과물입니다. 위의 모든 정보(속도, 사람 위치, 물체 정보, 안전 상태)를 하나로 합친 통합 상태 메시지 헤더입니다. CartStatus.msg 파일을 기반으로 생성


namespace sc_cpp
{

class StatusPublisher : public rclcpp::Node
{
public:
  explicit StatusPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── 콜백 함수 ──
  void bbox_callback(const sc_interfaces::msg::PersonBbox::SharedPtr msg);
  void item_callback(const sc_interfaces::msg::ItemDetected::SharedPtr msg);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void safety_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void publish_status();

  // ── ROS2 인터페이스 ──
  rclcpp::Subscription<sc_interfaces::msg::PersonBbox>::SharedPtr bbox_sub_;
  rclcpp::Subscription<sc_interfaces::msg::ItemDetected>::SharedPtr item_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  
  rclcpp::Publisher<sc_interfaces::msg::CartStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ── 내부 상태 저장 변수 ──
  sc_interfaces::msg::CartStatus current_status_;
  rclcpp::Time last_bbox_time_;
};

}  // namespace sc_cpp
#pragma once
// ================================================================
// obstacle_avoidance.hpp
// [C++ 담당] LiDAR 기반 'ㄷ'자 장애물 회피 노드
//
// ────────────────────────────────────────────────────
//  전체 동작 시나리오
// ────────────────────────────────────────────────────
//
//  1. Nav2로 목적지(초록 화살표) 설정 → 터틀봇이 직진 주행
//  2. 직진 도중 LiDAR(/scan)로 전방 장애물 감지
//  3. 좌/우 공간 비교 → 넓은 쪽으로 'ㄷ'자 회피
//
//     [오른쪽이 넓을 때 — 기본 'ㄷ'자]
//
//         장애물
//       ┌──────┐
//       │      │
//   ──→ │      ↓       → 직진 복귀
//       │      │
//       └──────┘
//
//     [오른쪽이 막혀있을 때 — 반전 'ㄷ'자]
//
//       ┌──────┐
//       │      │
//       ↑      │  ←──
//       │      │
//       └──────┘
//
//  4. 회피 완료 후 Nav2 목적지로 자동 복귀
//
// ────────────────────────────────────────────────────
//  나중에 사람 추종으로 확장할 때
// ────────────────────────────────────────────────────
//
//  - 현재: Nav2 goal → 직진 → 장애물 회피 → 목적지 도착
//  - 나중: YOLO+DeepSORT → ARQ 사람 추종 → 장애물 회피 → 사람 복귀
//  - 이 노드의 회피 로직은 그대로 재사용 가능
//  - follow_controller에서 이 노드의 회피 상태를 체크하면 됨
//
// ────────────────────────────────────────────────────
//  Subscribe / Publish
// ────────────────────────────────────────────────────
//
//  Subscribe:
//    /scan (sensor_msgs/msg/LaserScan)  — LDS-02 LiDAR 360° 데이터
//    /odom (nav_msgs/msg/Odometry)      — 현재 위치/방향 (회피 거리 계산용)
//
//  Publish:
//    /cmd_vel (geometry_msgs/msg/Twist) — 회피 주행 명령
//
//  ※ Nav2와 공존: 평소에는 Nav2가 /cmd_vel 제어
//     장애물 감지 시 이 노드가 /cmd_vel을 override하여 회피
//     회피 완료 후 Nav2에게 제어권 반환
//
// ────────────────────────────────────────────────────
//  파라미터 (rqt_reconfigure로 실시간 변경 가능)
// ────────────────────────────────────────────────────
//
//  obstacle_threshold_m    — 이 거리 이내에 장애물 있으면 회피 시작 (기본 0.5m)
//  side_clear_distance_m   — 좌/우 이 거리 이상 비어있어야 통과 가능 (기본 0.5m)
//  avoidance_speed         — 회피 시 직진 속도 (m/s, 기본 0.15)
//  avoidance_turn_speed    — 회피 시 회전 속도 (rad/s, 기본 0.8)
//  forward_distance_m      — 'ㄷ'자 직진 거리 (기본 0.5m)
//  side_distance_m         — 'ㄷ'자 옆으로 이동 거리 (기본 0.6m)
//  front_angle_deg         — 전방 감지 각도 범위 (기본 ±25°)
//  control_freq_hz         — 제어 루프 주기 (기본 10Hz)
//
// ================================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <cmath>
#include <string>

namespace sc_cpp
{

class ObstacleAvoidance : public rclcpp::Node
{
public:
  explicit ObstacleAvoidance(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ────────────────────────────────────────────
  //  회피 상태 머신 (State Machine)
  // ────────────────────────────────────────────
  //
  //  IDLE         — 평소 상태. Nav2가 주행 제어. 이 노드는 감시만 함.
  //  STOP         — 장애물 발견! 일단 정지하고 좌/우 공간 판단.
  //  TURN_SIDE    — 'ㄷ'자 1단계: 옆으로 90° 회전
  //  MOVE_SIDE    — 'ㄷ'자 2단계: 옆으로 side_distance_m 만큼 직진
  //  TURN_FORWARD — 'ㄷ'자 3단계: 원래 진행방향으로 90° 회전
  //  MOVE_FORWARD — 'ㄷ'자 4단계: 장애물 옆을 지나쳐서 forward_distance_m 직진
  //  TURN_RETURN  — 'ㄷ'자 5단계: 원래 경로쪽으로 90° 회전
  //  MOVE_RETURN  — 'ㄷ'자 6단계: 원래 경로로 side_distance_m 복귀 직진
  //  TURN_RESUME  — 'ㄷ'자 7단계: 원래 진행방향으로 90° 회전
  //  DONE         — 회피 완료 → IDLE로 복귀, Nav2에게 제어권 반환
  //
  enum class State
  {
    IDLE,
    STOP,
    TURN_SIDE,
    MOVE_SIDE,
    TURN_FORWARD,
    MOVE_FORWARD,
    TURN_RETURN,
    MOVE_RETURN,
    TURN_RESUME,
    DONE
  };

  // ── 회피 방향 ──
  enum class AvoidDirection
  {
    RIGHT,  // 기본 'ㄷ'자 (오른쪽으로 회피)
    LEFT    // 반전 'ㄷ'자 (왼쪽으로 회피)
  };

  // ── Subscriber / Publisher ──
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // ── 제어 루프 타이머 ──
  rclcpp::TimerBase::SharedPtr control_timer_;

  // ── 파라미터 콜백 ──
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  rcl_interfaces::msg::SetParametersResult onParamChange(
    const std::vector<rclcpp::Parameter> & params);

  // ── 콜백 ──
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void controlLoop();

  // ── LiDAR 분석 함수 ──
  // 전방 장애물까지 최소 거리
  double getMinDistance(const sensor_msgs::msg::LaserScan::SharedPtr & scan,
                        double angle_min_deg, double angle_max_deg);
  // 좌/우 공간 여유 확인
  double getLeftClearance(const sensor_msgs::msg::LaserScan::SharedPtr & scan);
  double getRightClearance(const sensor_msgs::msg::LaserScan::SharedPtr & scan);

  // ── 주행 명령 함수 ──
  void publishCmd(double linear, double angular);
  void stopRobot();

  // ── 회전/직진 완료 판단 ──
  bool hasRotated(double target_angle_rad);
  bool hasMoved(double target_distance_m);
  void saveCurrentPose();  // 회전/직진 시작 시점의 pose 저장

  // ── 오일러 각도 변환 ──
  double quaternionToYaw(double qx, double qy, double qz, double qw);
  double normalizeAngle(double angle);

  // ── 내부 상태 ──
  State state_ = State::IDLE;
  AvoidDirection avoid_dir_ = AvoidDirection::RIGHT;

  // 최신 센서 데이터
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;

  // 회전/직진 시작 시점 기록
  double saved_yaw_ = 0.0;     // 회전 시작 yaw
  double saved_x_ = 0.0;       // 직진 시작 x 좌표
  double saved_y_ = 0.0;       // 직진 시작 y 좌표

  // ── 파라미터 ──
  double obstacle_threshold_m_;   // 전방 이 거리 이내 → 회피 시작
  double side_clear_distance_m_;  // 좌/우 이 거리 이상 비어야 통과 가능
  double avoidance_speed_;        // 회피 직진 속도
  double avoidance_turn_speed_;   // 회피 회전 속도
  double forward_distance_m_;     // 'ㄷ'자 앞으로 이동 거리 (장애물 옆을 지나가는 거리)
  double side_distance_m_;        // 'ㄷ'자 옆으로 이동 거리
  double front_angle_deg_;        // 전방 감지 각도 ±N도
  double control_freq_hz_;        // 제어 루프 주기
};

}  // namespace sc_cpp
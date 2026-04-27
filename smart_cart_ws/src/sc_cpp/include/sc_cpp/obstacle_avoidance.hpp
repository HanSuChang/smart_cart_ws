#pragma once
// ================================================================
// obstacle_avoidance.hpp
// [C++ 담당] LiDAR 기반 동적 'ㄷ'자 장애물 회피 노드
//
// ────────────────────────────────────────────────────
//  핵심 동작 시나리오
// ────────────────────────────────────────────────────
//
//  Nav2로 목적지 주행 중 → 장애물 발견 → 'ㄷ'자 회피 → 목적지 복귀
//
//  ★ 핵심: 장애물 길이를 모르더라도 LiDAR로 실시간 감시하면서
//          장애물 끝을 자동 감지한 후 안전하게 회피
//
// ────────────────────────────────────────────────────
//  'ㄷ'자 회피 동적 알고리즘 (장애물 길이 자동 인식)
// ────────────────────────────────────────────────────
//
//  ① 장애물 감지 (전방 거리 < threshold)
//  ② 정지 후 좌/우 LiDAR 공간 비교 → 더 넓은 쪽 선택
//  ③ 옆으로 90° 회전
//  ④ 옆으로 직진 (장애물과 안전거리 확보될 때까지)
//  ⑤ 원래 진행 방향으로 90° 회전
//  ⑥ 직진하며 LiDAR로 옆면 계속 감시
//     ↓ 옆에 장애물 보이는 동안 → 계속 직진
//     ↓ 옆이 비어짐 → 장애물 끝 통과!
//  ⑦ 안전 마진을 위해 추가로 조금 더 직진
//  ⑧ 원래 경로 쪽으로 90° 회전
//  ⑨ 원래 경로로 복귀 직진
//  ⑩ 진행 방향으로 90° 회전
//  → 회피 완료, Nav2에게 제어권 반환
//
//  [장애물이 길든 짧든 자동 대응]
//   - 짧은 장애물: ⑥에서 빨리 옆이 비어서 금방 회피 완료
//   - 긴 장애물:   ⑥에서 옆이 계속 차있어서 끝까지 직진
//
//  [좌/우 공간 비교]
//   - 오른쪽이 더 넓음 → 오른쪽 'ㄷ'자
//   - 왼쪽이 더 넓음   → 왼쪽 반전 'ㄷ'자
//   - 양쪽 다 좁음     → 후진 후 재시도
//
// ────────────────────────────────────────────────────
//  나중에 사람 추종으로 확장할 때
// ────────────────────────────────────────────────────
//
//  현재: Nav2 goal → 직진 → 장애물 회피 → 목적지 도착
//  나중: YOLO+DeepSORT → ARQ 사람 추종 → 장애물 회피 → 사람 복귀
//  이 노드의 회피 로직은 그대로 재사용 가능
//
// ────────────────────────────────────────────────────
//  Subscribe / Publish
// ────────────────────────────────────────────────────
//
//  Subscribe:
//    /scan  — LDS-02 LiDAR 360° 거리 데이터
//    /odom  — 현재 위치/방향 (회전 각도, 직진 거리 계산)
//
//  Publish:
//    /cmd_vel — 회피 주행 속도 명령 (Nav2 override)
//
// ────────────────────────────────────────────────────
//  파라미터 (ros2 param set / rqt_reconfigure 실시간 변경)
// ────────────────────────────────────────────────────
//
//  obstacle_threshold_m    전방 이 거리 이내 → 회피 시작 (0.5m)
//  side_safe_distance_m    장애물과의 측면 안전거리 (0.4m)
//  side_clear_threshold_m  측면이 이 거리 이상 비면 "장애물 끝" (0.7m)
//  side_distance_m         'ㄷ'자 옆으로 이동 거리 (0.6m)
//  pass_margin_m           장애물 끝 통과 후 추가 안전 마진 (0.3m)
//  avoidance_speed         회피 직진 속도 (0.15 m/s)
//  avoidance_turn_speed    회피 회전 속도 (0.6 rad/s)
//  front_angle_deg         전방 감지 각도 ±N° (25°)
//  control_freq_hz         제어 루프 주기 (10Hz)
//
// ================================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <cmath>

namespace sc_cpp
{

class ObstacleAvoidance : public rclcpp::Node
{
public:
  explicit ObstacleAvoidance(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ────────────────────────────────────────────
  //  상태 머신 — 'ㄷ'자 회피 단계별
  // ────────────────────────────────────────────
  //
  //  IDLE           평소: Nav2가 제어. 장애물만 감시.
  //  STOP           장애물 감지 → 정지 → 좌/우 비교
  //  TURN_SIDE      ③ 옆으로 90° 회전
  //  MOVE_SIDE      ④ 옆으로 직진 (안전거리 확보)
  //  TURN_FORWARD   ⑤ 원래 방향 90° 회전
  //  PASS_OBSTACLE  ⑥ 직진 + LiDAR 옆면 감시 (장애물 끝 감지)
  //  EXTRA_FORWARD  ⑦ 장애물 끝 통과 후 안전 마진 직진
  //  TURN_RETURN    ⑧ 경로 복귀 방향 90° 회전
  //  MOVE_RETURN    ⑨ 경로 복귀 직진
  //  TURN_RESUME    ⑩ 원래 방향 90° 회전
  //  DONE           회피 완료 → IDLE 복귀
  //
  enum class State
  {
    IDLE,
    STOP,
    TURN_SIDE,
    MOVE_SIDE,
    TURN_FORWARD,
    PASS_OBSTACLE,
    EXTRA_FORWARD,
    TURN_RETURN,
    MOVE_RETURN,
    TURN_RESUME,
    DONE
  };

  // 회피 방향
  enum class AvoidDirection
  {
    RIGHT,  // 오른쪽 'ㄷ'자
    LEFT    // 왼쪽 반전 'ㄷ'자
  };

  // ── ROS2 인터페이스 ──
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // ── 콜백 ──
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void controlLoop();
  rcl_interfaces::msg::SetParametersResult onParamChange(
    const std::vector<rclcpp::Parameter> & params);

  // ── LiDAR 분석 함수 ──
  double getMinDistanceInRange(double angle_min_deg, double angle_max_deg);
  double getFrontDistance();
  double getSideClearance();
  double getLeftSpace();
  double getRightSpace();

  // ── 주행 제어 ──
  void publishCmd(double linear, double angular);
  void stopRobot();

  // ── Odometry 기반 판단 ──
  void savePoseSnapshot();
  bool hasRotatedBy(double target_angle_rad);
  bool hasMovedBy(double target_distance_m);

  // ── 유틸 ──
  double quaternionToYaw(double qx, double qy, double qz, double qw);
  double normalizeAngle(double angle);

  // ── 내부 상태 ──
  State state_ = State::IDLE;
  AvoidDirection avoid_dir_ = AvoidDirection::RIGHT;

  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;

  double saved_yaw_ = 0.0;
  double saved_x_   = 0.0;
  double saved_y_   = 0.0;

  bool obstacle_was_seen_ = false;  // PASS_OBSTACLE에서 "옆에 장애물 봤었는지" 기록

  // ── 파라미터 ──
  double obstacle_threshold_m_;
  double side_safe_distance_m_;
  double side_clear_threshold_m_;
  double side_distance_m_;
  double pass_margin_m_;
  double avoidance_speed_;
  double avoidance_turn_speed_;
  double front_angle_deg_;
  double control_freq_hz_;
};

}  // namespace sc_cpp
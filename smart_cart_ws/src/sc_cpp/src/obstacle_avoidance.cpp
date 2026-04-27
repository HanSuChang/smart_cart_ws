// ================================================================
// obstacle_avoidance.cpp
// [C++ 담당] LiDAR 기반 동적 'ㄷ'자 장애물 회피 — 전체 구현
//
// ────────────────────────────────────────────────────
//  'ㄷ'자 회피 동작 순서 (오른쪽 회피 기준)
// ────────────────────────────────────────────────────
//
//   진행방향 →    장애물
//
//   ③ 우로 90° 회전
//   ④ 옆으로 직진 (안전거리 확보)        ↓
//   ⑤ 좌로 90° 회전 (원래 방향 복원)     ↓
//   ⑥ 직진 + LiDAR 옆면 감시            ↓
//      옆에 장애물 보임 → 계속 직진       ↓
//      옆이 비어짐 → 장애물 끝 통과!      ↓
//   ⑦ 안전 마진 추가 직진                ↓
//   ⑧ 좌로 90° 회전 (경로 복귀)          ↓
//   ⑨ 원래 경로로 직진 복귀              ↓
//   ⑩ 우로 90° 회전 (진행 방향 복원)     ↓
//   → 회피 완료!
//
//   [왼쪽 회피는 좌/우 반대]
//
// ================================================================

#include "sc_cpp/obstacle_avoidance.hpp"
#include <algorithm>
#include <chrono>
#include <limits>

using namespace std::chrono_literals;

namespace sc_cpp
{

// ════════════════════════════════════════════════════
//  생성자
//  - 파라미터 선언 및 읽기
//  - Subscriber, Publisher, 타이머 생성
// ════════════════════════════════════════════════════
ObstacleAvoidance::ObstacleAvoidance(const rclcpp::NodeOptions & options)
: Node("obstacle_avoidance", options)
{
  // ── 파라미터 선언 ──
  declare_parameter("obstacle_threshold_m",    0.5);   // 전방 이 거리 이내 → 회피
  declare_parameter("side_safe_distance_m",    0.4);   // 측면 안전거리
  declare_parameter("side_clear_threshold_m",  0.7);   // 측면 이 이상 비면 = 장애물 끝
  declare_parameter("side_distance_m",         0.6);   // 옆으로 이동 거리
  declare_parameter("pass_margin_m",           0.3);   // 장애물 끝 후 추가 마진
  declare_parameter("avoidance_speed",         0.15);  // 직진 속도 (m/s)
  declare_parameter("avoidance_turn_speed",    0.6);   // 회전 속도 (rad/s)
  declare_parameter("front_angle_deg",         25.0);  // 전방 감지 각도 ±25°
  declare_parameter("control_freq_hz",         10.0);  // 제어 루프 10Hz

  // 파라미터 읽기
  obstacle_threshold_m_    = get_parameter("obstacle_threshold_m").as_double();
  side_safe_distance_m_    = get_parameter("side_safe_distance_m").as_double();
  side_clear_threshold_m_  = get_parameter("side_clear_threshold_m").as_double();
  side_distance_m_         = get_parameter("side_distance_m").as_double();
  pass_margin_m_           = get_parameter("pass_margin_m").as_double();
  avoidance_speed_         = get_parameter("avoidance_speed").as_double();
  avoidance_turn_speed_    = get_parameter("avoidance_turn_speed").as_double();
  front_angle_deg_         = get_parameter("front_angle_deg").as_double();
  control_freq_hz_         = get_parameter("control_freq_hz").as_double();

  // 파라미터 변경 콜백
  param_cb_handle_ = add_on_set_parameters_callback(
    std::bind(&ObstacleAvoidance::onParamChange, this, std::placeholders::_1));

  // ── Subscriber ──
  auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());
scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
  "/scan", qos,
  std::bind(&ObstacleAvoidance::scanCallback, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10,
    std::bind(&ObstacleAvoidance::odomCallback, this, std::placeholders::_1));

  // ── Publisher ──
  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // ── 제어 루프 타이머 ──
  auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / control_freq_hz_));
  control_timer_ = create_wall_timer(period_ns,
    std::bind(&ObstacleAvoidance::controlLoop, this));

  RCLCPP_INFO(get_logger(),
    "ObstacleAvoidance 시작 — threshold=%.2fm, side_safe=%.2fm, side_clear=%.2fm",
    obstacle_threshold_m_, side_safe_distance_m_, side_clear_threshold_m_);
}

// ════════════════════════════════════════════════════
//  파라미터 실시간 변경 콜백
//  ros2 param set /obstacle_avoidance obstacle_threshold_m 0.4
//  rqt_reconfigure 에서도 변경 가능
// ════════════════════════════════════════════════════
rcl_interfaces::msg::SetParametersResult ObstacleAvoidance::onParamChange(
  const std::vector<rclcpp::Parameter> & params)
{
  for (const auto & p : params) {
    const auto & n = p.get_name();
    if (n == "obstacle_threshold_m")    obstacle_threshold_m_   = p.as_double();
    if (n == "side_safe_distance_m")    side_safe_distance_m_   = p.as_double();
    if (n == "side_clear_threshold_m")  side_clear_threshold_m_ = p.as_double();
    if (n == "side_distance_m")         side_distance_m_        = p.as_double();
    if (n == "pass_margin_m")           pass_margin_m_          = p.as_double();
    if (n == "avoidance_speed")         avoidance_speed_        = p.as_double();
    if (n == "avoidance_turn_speed")    avoidance_turn_speed_   = p.as_double();
    if (n == "front_angle_deg")         front_angle_deg_        = p.as_double();
    RCLCPP_INFO(get_logger(), "파라미터 변경: %s", n.c_str());
  }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

// ════════════════════════════════════════════════════
//  /scan 콜백 — 최신 LiDAR 데이터 저장
// ════════════════════════════════════════════════════
void ObstacleAvoidance::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  latest_scan_ = msg;
}

// ════════════════════════════════════════════════════
//  /odom 콜백 — 현재 위치/방향 저장
// ════════════════════════════════════════════════════
void ObstacleAvoidance::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_ = msg;
}

// ════════════════════════════════════════════════════
//
//  ★ 메인 제어 루프 — 10Hz 호출
//     상태 머신(State Machine)으로 'ㄷ'자 회피 실행
//
// ════════════════════════════════════════════════════
void ObstacleAvoidance::controlLoop()
{
  if (!latest_scan_ || !latest_odom_) return;

  switch (state_)
  {
    // ──────────────────────────────────────────────
    //  IDLE: 평소 상태
    //  - Nav2가 주행 제어 중. 이 노드는 /cmd_vel 발행 안 함.
    //  - LiDAR로 전방만 계속 감시.
    //  - 전방에 장애물 감지되면 → STOP으로 전환.
    // ──────────────────────────────────────────────
    case State::IDLE:
    {
      double front = getFrontDistance();
      if (front < obstacle_threshold_m_) {
        RCLCPP_WARN(get_logger(),
          "=== 전방 장애물 감지! 거리=%.2fm ===", front);
        state_ = State::STOP;
      }
      break;
    }

    // ──────────────────────────────────────────────
    //  STOP: 정지 → 좌/우 공간 비교 → 회피 방향 결정
    //
    //  LiDAR로 왼쪽(60~120°)과 오른쪽(-120~-60°) 공간을 측정.
    //  더 넓은 쪽으로 'ㄷ'자 회피 방향 결정.
    //  양쪽 다 좁으면 후진 후 재시도.
    // ──────────────────────────────────────────────
    case State::STOP:
    {
      stopRobot();

      double left_space  = getLeftSpace();
      double right_space = getRightSpace();

      RCLCPP_INFO(get_logger(),
        "좌측=%.2fm, 우측=%.2fm", left_space, right_space);

      if (right_space >= side_safe_distance_m_ &&
          right_space >= left_space) {
        // 오른쪽이 더 넓거나 같음 → 오른쪽 'ㄷ'자
        avoid_dir_ = AvoidDirection::RIGHT;
        RCLCPP_INFO(get_logger(), "→ 오른쪽 'ㄷ'자 회피 선택");
      } else if (left_space >= side_safe_distance_m_) {
        // 왼쪽이 더 넓음 → 왼쪽 반전 'ㄷ'자
        avoid_dir_ = AvoidDirection::LEFT;
        RCLCPP_INFO(get_logger(), "← 왼쪽 반전 'ㄷ'자 회피 선택");
      } else {
        // 양쪽 다 좁음 → 후진
        RCLCPP_WARN(get_logger(),
          "양쪽 다 막힘! 후진 (좌=%.2f, 우=%.2f)", left_space, right_space);
        publishCmd(-avoidance_speed_, 0.0);
        state_ = State::IDLE;
        break;
      }

      obstacle_was_seen_ = false;
      savePoseSnapshot();
      state_ = State::TURN_SIDE;
      break;
    }

    // ──────────────────────────────────────────────
    //  ③ TURN_SIDE: 옆으로 90° 회전
    //
    //  오른쪽 회피 → 시계 방향(-90°) 회전
    //  왼쪽 회피  → 반시계 방향(+90°) 회전
    //
    //  odom의 yaw 변화량으로 90° 회전 완료 판단.
    // ──────────────────────────────────────────────
    case State::TURN_SIDE:
    {
      double target = (avoid_dir_ == AvoidDirection::RIGHT) ? -M_PI_2 : M_PI_2;

      if (!hasRotatedBy(target)) {
        double turn = (avoid_dir_ == AvoidDirection::RIGHT)
                      ? -avoidance_turn_speed_ : avoidance_turn_speed_;
        publishCmd(0.0, turn);
      } else {
        stopRobot();
        savePoseSnapshot();
        state_ = State::MOVE_SIDE;
        RCLCPP_INFO(get_logger(), "③ 옆 회전 완료 → ④ 옆 직진");
      }
      break;
    }

    // ──────────────────────────────────────────────
    //  ④ MOVE_SIDE: 옆으로 side_distance_m 직진
    //
    //  장애물과 안전거리를 확보하기 위해 옆으로 비킴.
    //  odom으로 이동 거리 측정.
    //
    //  ★ 직진 도중에도 전방 장애물 체크
    //     혹시 옆에도 장애물이 있으면 충돌 방지
    // ──────────────────────────────────────────────
    case State::MOVE_SIDE:
    {
      // 전방에 또 장애물 있으면 멈춤 (2차 충돌 방지)
      double front = getFrontDistance();
      if (front < side_safe_distance_m_) {
        stopRobot();
        RCLCPP_WARN(get_logger(), "④ 옆 직진 중 전방 장애물! 추가 회피 필요");
        // side_distance를 더 넓혀서 재시도
        savePoseSnapshot();
        break;
      }

      if (!hasMovedBy(side_distance_m_)) {
        publishCmd(avoidance_speed_, 0.0);
      } else {
        stopRobot();
        savePoseSnapshot();
        state_ = State::TURN_FORWARD;
        RCLCPP_INFO(get_logger(), "④ 옆 직진 완료 (%.2fm) → ⑤ 원래 방향 회전",
                    side_distance_m_);
      }
      break;
    }

    // ──────────────────────────────────────────────
    //  ⑤ TURN_FORWARD: 원래 진행 방향으로 90° 회전
    //
    //  오른쪽 회피였으면 → 왼쪽 90° (원래 방향 복원)
    //  왼쪽 회피였으면  → 오른쪽 90°
    // ──────────────────────────────────────────────
    case State::TURN_FORWARD:
    {
      double target = (avoid_dir_ == AvoidDirection::RIGHT) ? M_PI_2 : -M_PI_2;

      if (!hasRotatedBy(target)) {
        double turn = (avoid_dir_ == AvoidDirection::RIGHT)
                      ? avoidance_turn_speed_ : -avoidance_turn_speed_;
        publishCmd(0.0, turn);
      } else {
        stopRobot();
        savePoseSnapshot();
        obstacle_was_seen_ = false;
        state_ = State::PASS_OBSTACLE;
        RCLCPP_INFO(get_logger(), "⑤ 방향 복원 완료 → ⑥ 장애물 옆 직진 (LiDAR 감시)");
      }
      break;
    }

    // ──────────────────────────────────────────────
    //  ⑥ PASS_OBSTACLE: 핵심 단계!
    //     직진하면서 LiDAR로 옆면을 계속 감시
    //
    //  동작 방식:
    //    1. 옆면(회피 방향 반대쪽)을 LiDAR로 계속 측정
    //    2. 측면 거리 < side_clear_threshold → 장애물 아직 있음 → 계속 직진
    //    3. 측면 거리 >= side_clear_threshold → 장애물 끝 통과!
    //
    //  ★ obstacle_was_seen_ 플래그:
    //    처음에 옆에 장애물이 보여야 "장애물이 있었다"고 기록.
    //    그 후에 안 보이면 "장애물 끝"으로 판단.
    //    이렇게 해야 장애물 시작 전에 빈 공간을 "끝"으로 오판 안 함.
    //
    //  ★ 전방 충돌도 동시 체크:
    //    직진 도중 전방에 또 다른 장애물이 있으면 정지.
    // ──────────────────────────────────────────────
    case State::PASS_OBSTACLE:
    {
      // 전방 충돌 체크 (직진 도중 또 다른 장애물)
      double front = getFrontDistance();
      if (front < side_safe_distance_m_) {
        stopRobot();
        RCLCPP_WARN(get_logger(),
          "⑥ 직진 중 전방 장애물(%.2fm)! 감속 대기", front);
        break;
      }

      // 옆면 거리 측정 (회피 방향 반대쪽 = 장애물이 있는 쪽)
      double side = getSideClearance();

      if (side < side_clear_threshold_m_) {
        // 옆에 장애물이 아직 있음 → 계속 직진
        obstacle_was_seen_ = true;
        publishCmd(avoidance_speed_, 0.0);
      } else {
        if (obstacle_was_seen_) {
          // 장애물이 있었는데 이제 안 보임 → 장애물 끝 통과!
          RCLCPP_INFO(get_logger(),
            "⑥ 장애물 끝 감지! (측면=%.2fm) → ⑦ 마진 직진", side);
          stopRobot();
          savePoseSnapshot();
          state_ = State::EXTRA_FORWARD;
        } else {
          // 아직 장애물 시작 전 → 계속 직진
          publishCmd(avoidance_speed_, 0.0);
        }
      }
      break;
    }

    // ──────────────────────────────────────────────
    //  ⑦ EXTRA_FORWARD: 안전 마진 추가 직진
    //
    //  장애물 끝을 통과했지만 바로 꺾으면
    //  터틀봇 뒷부분이 장애물 모서리에 부딪힐 수 있음.
    //  pass_margin_m 만큼 더 직진해서 안전하게 지나감.
    // ──────────────────────────────────────────────
    case State::EXTRA_FORWARD:
    {
      if (!hasMovedBy(pass_margin_m_)) {
        publishCmd(avoidance_speed_, 0.0);
      } else {
        stopRobot();
        savePoseSnapshot();
        state_ = State::TURN_RETURN;
        RCLCPP_INFO(get_logger(), "⑦ 마진 직진 완료 (%.2fm) → ⑧ 경로 복귀 회전",
                    pass_margin_m_);
      }
      break;
    }

    // ──────────────────────────────────────────────
    //  ⑧ TURN_RETURN: 원래 경로 쪽으로 90° 회전
    //
    //  오른쪽으로 비켜갔었으면 → 왼쪽으로 90° (경로로 돌아감)
    //  왼쪽으로 비켜갔었으면  → 오른쪽으로 90°
    // ──────────────────────────────────────────────
    case State::TURN_RETURN:
    {
      double target = (avoid_dir_ == AvoidDirection::RIGHT) ? M_PI_2 : -M_PI_2;

      if (!hasRotatedBy(target)) {
        double turn = (avoid_dir_ == AvoidDirection::RIGHT)
                      ? avoidance_turn_speed_ : -avoidance_turn_speed_;
        publishCmd(0.0, turn);
      } else {
        stopRobot();
        savePoseSnapshot();
        state_ = State::MOVE_RETURN;
        RCLCPP_INFO(get_logger(), "⑧ 복귀 회전 완료 → ⑨ 경로 복귀 직진");
      }
      break;
    }

    // ──────────────────────────────────────────────
    //  ⑨ MOVE_RETURN: 원래 경로로 side_distance_m 복귀 직진
    //
    //  ④에서 옆으로 비켜간 만큼 다시 돌아옴.
    //  이걸로 원래 주행 라인에 복귀.
    // ──────────────────────────────────────────────
    case State::MOVE_RETURN:
    {
      if (!hasMovedBy(side_distance_m_)) {
        publishCmd(avoidance_speed_, 0.0);
      } else {
        stopRobot();
        savePoseSnapshot();
        state_ = State::TURN_RESUME;
        RCLCPP_INFO(get_logger(), "⑨ 경로 복귀 완료 → ⑩ 방향 복원");
      }
      break;
    }

    // ──────────────────────────────────────────────
    //  ⑩ TURN_RESUME: 원래 진행 방향으로 90° 회전 (최종)
    //
    //  이 회전이 끝나면 터틀봇은 원래 진행 방향을 보고 있음.
    //  Nav2가 다시 목적지를 향해 주행 재개.
    // ──────────────────────────────────────────────
    case State::TURN_RESUME:
    {
      double target = (avoid_dir_ == AvoidDirection::RIGHT) ? -M_PI_2 : M_PI_2;

      if (!hasRotatedBy(target)) {
        double turn = (avoid_dir_ == AvoidDirection::RIGHT)
                      ? -avoidance_turn_speed_ : avoidance_turn_speed_;
        publishCmd(0.0, turn);
      } else {
        stopRobot();
        state_ = State::DONE;
        RCLCPP_INFO(get_logger(), "⑩ 방향 복원 완료!");
      }
      break;
    }

    // ──────────────────────────────────────────────
    //  DONE: 'ㄷ'자 회피 완료 → IDLE 복귀
    //  Nav2에게 제어권 반환.
    //  Nav2가 자동으로 목적지 경로를 재계산하여 주행 재개.
    // ──────────────────────────────────────────────
    case State::DONE:
    {
      RCLCPP_INFO(get_logger(),
        "=== 'ㄷ'자 회피 완료! Nav2 목적지 주행 재개 ===");
      state_ = State::IDLE;
      break;
    }
  }  // switch
}

// ════════════════════════════════════════════════════
//  LiDAR: 지정 각도 범위 내 최소 거리
//
//  angle_min_deg ~ angle_max_deg 사이에 있는
//  LiDAR 레이저 빔 중 가장 가까운 거리를 반환.
//
//  예: getMinDistanceInRange(-25, 25) → 전방 ±25° 최소 거리
//  예: getMinDistanceInRange(60, 120) → 왼쪽 옆 최소 거리
// ════════════════════════════════════════════════════
double ObstacleAvoidance::getMinDistanceInRange(
  double angle_min_deg, double angle_max_deg)
{
  if (!latest_scan_) return 999.0;

  double min_dist = std::numeric_limits<double>::max();
  const double amin = angle_min_deg * M_PI / 180.0;
  const double amax = angle_max_deg * M_PI / 180.0;

  int n = static_cast<int>(latest_scan_->ranges.size());
  for (int i = 0; i < n; ++i) {
    float r = latest_scan_->ranges[i];
    if (!std::isfinite(r) ||
        r < latest_scan_->range_min ||
        r > latest_scan_->range_max) continue;

    double angle = latest_scan_->angle_min + i * latest_scan_->angle_increment;

    // 각도 정규화 (-π ~ π)
    angle = normalizeAngle(angle);

    if (angle >= amin && angle <= amax) {
      min_dist = std::min(min_dist, static_cast<double>(r));
    }
  }
  return min_dist;
}

// ════════════════════════════════════════════════════
//  전방 거리 — ±front_angle_deg° 범위
//  장애물 감지의 기본 입력값
// ════════════════════════════════════════════════════
double ObstacleAvoidance::getFrontDistance()
{
  // LDS-02는 0°=정면, 양방향으로 전방 범위를 나눠서 측정
  double front_right = getMinDistanceInRange(0.0, front_angle_deg_);
  double front_left  = getMinDistanceInRange(360.0 - front_angle_deg_, 360.0);
  return std::min(front_right, front_left);
}

// ════════════════════════════════════════════════════
//  측면 클리어런스 — 장애물 끝 감지용
//
//  회피 방향에 따라 감시하는 쪽이 다름:
//  - 오른쪽으로 비켜갔으면 → 왼쪽(장애물 쪽)을 감시
//  - 왼쪽으로 비켜갔으면  → 오른쪽(장애물 쪽)을 감시
//
//  이 값이 side_clear_threshold_m 이상이면 → 장애물 끝 통과
// ════════════════════════════════════════════════════
double ObstacleAvoidance::getSideClearance()
{
  if (avoid_dir_ == AvoidDirection::RIGHT) {
    // 오른쪽으로 비켜감 → 왼쪽(장애물 쪽) 감시
    return getMinDistanceInRange(60.0, 120.0);
  } else {
    // 왼쪽으로 비켜감 → 오른쪽(장애물 쪽) 감시
    return getMinDistanceInRange(240.0, 300.0);
  }
}

// ════════════════════════════════════════════════════
//  왼쪽 전체 공간 — 회피 방향 결정에 사용
//  30°~150° 범위의 최소 거리
// ════════════════════════════════════════════════════
double ObstacleAvoidance::getLeftSpace()
{
  return getMinDistanceInRange(60.0, 120.0);
}

// ════════════════════════════════════════════════════
//  오른쪽 전체 공간 — 회피 방향 결정에 사용
//  -150°~-30° 범위의 최소 거리
// ════════════════════════════════════════════════════
double ObstacleAvoidance::getRightSpace()
{
  return getMinDistanceInRange(240.0, 300.0);
}

// ════════════════════════════════════════════════════
//  /cmd_vel 발행
//
//  linear  > 0: 전진
//  linear  < 0: 후진
//  angular > 0: 반시계 회전 (왼쪽)
//  angular < 0: 시계 회전 (오른쪽)
// ════════════════════════════════════════════════════
void ObstacleAvoidance::publishCmd(double linear, double angular)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x  = linear;
  msg.angular.z = angular;
  cmd_vel_pub_->publish(msg);
}

// ════════════════════════════════════════════════════
//  정지 명령 — linear=0, angular=0
// ════════════════════════════════════════════════════
void ObstacleAvoidance::stopRobot()
{
  publishCmd(0.0, 0.0);
}

// ════════════════════════════════════════════════════
//  현재 pose 스냅샷 저장
//
//  회전/직진 시작 시점의 위치와 방향을 기록.
//  이 기준점으로부터 "얼마나 회전했는지" "얼마나 이동했는지" 계산.
// ════════════════════════════════════════════════════
void ObstacleAvoidance::savePoseSnapshot()
{
  if (!latest_odom_) return;

  auto & pos = latest_odom_->pose.pose.position;
  auto & ori = latest_odom_->pose.pose.orientation;

  saved_x_   = pos.x;
  saved_y_   = pos.y;
  saved_yaw_ = quaternionToYaw(ori.x, ori.y, ori.z, ori.w);
}

// ════════════════════════════════════════════════════
//  회전 완료 판단
//
//  saved_yaw_에서 target_angle_rad만큼 회전했으면 true.
//  양수 = 반시계(왼쪽), 음수 = 시계(오른쪽)
//
//  ★ 90% 도달 시 완료로 판단 (오차 허용)
//     odom에 미세한 노이즈가 있어서 정확히 90°는 어려움
// ════════════════════════════════════════════════════
bool ObstacleAvoidance::hasRotatedBy(double target_angle_rad)
{
  if (!latest_odom_) return false;

  auto & ori = latest_odom_->pose.pose.orientation;
  double current_yaw = quaternionToYaw(ori.x, ori.y, ori.z, ori.w);
  double diff = normalizeAngle(current_yaw - saved_yaw_);

  return std::abs(diff) >= std::abs(target_angle_rad) * 0.90;
}

// ════════════════════════════════════════════════════
//  직진 완료 판단
//
//  saved_x_, saved_y_에서 target_distance_m만큼 이동했으면 true.
//  유클리드 거리로 계산.
//
//  ★ 95% 도달 시 완료 (바퀴 미끄러짐 오차 허용)
// ════════════════════════════════════════════════════
bool ObstacleAvoidance::hasMovedBy(double target_distance_m)
{
  if (!latest_odom_) return false;

  auto & pos = latest_odom_->pose.pose.position;
  double dx = pos.x - saved_x_;
  double dy = pos.y - saved_y_;
  double dist = std::sqrt(dx * dx + dy * dy);

  return dist >= target_distance_m * 0.95;
}

// ════════════════════════════════════════════════════
//  쿼터니언 → yaw(평면 회전각) 변환
//
//  ROS2 odom은 쿼터니언(4원수)으로 3D 방향을 표현.
//  터틀봇은 2D 평면 로봇이라 yaw(Z축 회전)만 필요.
//  이 함수로 쿼터니언 → yaw 각도(라디안) 추출.
// ════════════════════════════════════════════════════
double ObstacleAvoidance::quaternionToYaw(
  double qx, double qy, double qz, double qw)
{
  double siny = 2.0 * (qw * qz + qx * qy);
  double cosy = 1.0 - 2.0 * (qy * qy + qz * qz);
  return std::atan2(siny, cosy);
}

// ════════════════════════════════════════════════════
//  각도 정규화 — -π ~ +π 범위로 맞춤
//
//  회전 오차 계산 시 360°를 넘어가면 (예: 350° → 10°)
//  오차가 340°로 잘못 계산되는 문제 방지.
// ════════════════════════════════════════════════════
double ObstacleAvoidance::normalizeAngle(double angle)
{
  while (angle >  M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

}  // namespace sc_cpp

// ════════════════════════════════════════════════════
//  ROS2 컴포넌트 등록
//  이걸로 launch 파일에서 노드를 실행할 수 있음
//  ros2 run sc_cpp obstacle_avoidance
// ════════════════════════════════════════════════════
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sc_cpp::ObstacleAvoidance)
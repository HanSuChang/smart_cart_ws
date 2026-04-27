#include "obstacle_avoidance.hpp" // 프로젝트의 구조에 맞게 경로 수정 필요
#include <rclcpp/qos.hpp>
#include <algorithm>
#include <limits>

using namespace std::chrono_literals;

namespace sc_cpp
{

ObstacleAvoidance::ObstacleAvoidance(const rclcpp::NodeOptions & options)
: Node("obstacle_avoidance", options)
{
  // 1. 파라미터 선언 및 초기화
  obstacle_threshold_m_ = this->declare_parameter("obstacle_threshold_m", 0.5);
  side_clear_distance_m_ = this->declare_parameter("side_clear_distance_m", 0.5);
  avoidance_speed_ = this->declare_parameter("avoidance_speed", 0.15);
  avoidance_turn_speed_ = this->declare_parameter("avoidance_turn_speed", 0.8);
  forward_distance_m_ = this->declare_parameter("forward_distance_m", 0.5);
  side_distance_m_ = this->declare_parameter("side_distance_m", 0.6);
  front_angle_deg_ = this->declare_parameter("front_angle_deg", 25.0);
  control_freq_hz_ = this->declare_parameter("control_freq_hz", 10.0);

  // 2. 파라미터 동적 변경 콜백 등록 (rqt_reconfigure 지원)
  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&ObstacleAvoidance::onParamChange, this, std::placeholders::_1));

  // 3. QoS 설정 (LiDAR는 SensorDataQoS가 유리함)
  rclcpp::QoS sensor_qos(rclcpp::KeepLast(5));
  sensor_qos.best_effort();
  rclcpp::QoS default_qos(10);

  // 4. Sub / Pub 생성
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", sensor_qos, std::bind(&ObstacleAvoidance::scanCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", default_qos, std::bind(&ObstacleAvoidance::odomCallback, this, std::placeholders::_1));
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", default_qos);

  // 5. 제어 루프 타이머 생성
  auto period = std::chrono::duration<double>(1.0 / control_freq_hz_);
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&ObstacleAvoidance::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "Obstacle Avoidance Node Initialized (Ready for Nav2 override)");
}

rcl_interfaces::msg::SetParametersResult ObstacleAvoidance::onParamChange(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & param : params) {
    if (param.get_name() == "obstacle_threshold_m") obstacle_threshold_m_ = param.as_double();
    else if (param.get_name() == "side_clear_distance_m") side_clear_distance_m_ = param.as_double();
    else if (param.get_name() == "avoidance_speed") avoidance_speed_ = param.as_double();
    else if (param.get_name() == "avoidance_turn_speed") avoidance_turn_speed_ = param.as_double();
    else if (param.get_name() == "forward_distance_m") forward_distance_m_ = param.as_double();
    else if (param.get_name() == "side_distance_m") side_distance_m_ = param.as_double();
    else if (param.get_name() == "front_angle_deg") front_angle_deg_ = param.as_double();
    else if (param.get_name() == "control_freq_hz") {
      control_freq_hz_ = param.as_double();
      // 주기가 바뀌면 타이머 리셋 필요 (생략 가능, 통상 재시작 권장)
    }
  }
  return result;
}

void ObstacleAvoidance::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  latest_scan_ = msg;
}

void ObstacleAvoidance::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_ = msg;
}

void ObstacleAvoidance::controlLoop()
{
  if (!latest_scan_ || !latest_odom_) return;

  double min_front_dist = getMinDistance(latest_scan_, -front_angle_deg_, front_angle_deg_);
  const double TARGET_TURN_RAD = M_PI / 2.0; // 90도

  switch (state_) {
    case State::IDLE:
      if (min_front_dist < obstacle_threshold_m_) {
        RCLCPP_WARN(this->get_logger(), "Obstacle detected at %.2fm! Overriding Nav2...", min_front_dist);
        state_ = State::STOP;
      }
      break;

    case State::STOP:
      stopRobot();
      {
        double left_space = getLeftClearance(latest_scan_);
        double right_space = getRightClearance(latest_scan_);
        
        // 오른쪽이 더 넓거나 같으면 기본 회피(우측 이동), 아니면 반전 회피(좌측 이동)
        if (right_space >= left_space && right_space >= side_clear_distance_m_) {
          avoid_dir_ = AvoidDirection::RIGHT;
          RCLCPP_INFO(this->get_logger(), "Decided to avoid RIGHT. Left: %.2f, Right: %.2f", left_space, right_space);
        } else {
          avoid_dir_ = AvoidDirection::LEFT;
          RCLCPP_INFO(this->get_logger(), "Decided to avoid LEFT. Left: %.2f, Right: %.2f", left_space, right_space);
        }
        saveCurrentPose();
        state_ = State::TURN_SIDE;
      }
      break;

    case State::TURN_SIDE:
      {
        // RIGHT: 오른쪽(-90도) 회전, LEFT: 왼쪽(+90도) 회전
        double sign = (avoid_dir_ == AvoidDirection::RIGHT) ? -1.0 : 1.0;
        publishCmd(0.0, sign * avoidance_turn_speed_);
        if (hasRotated(TARGET_TURN_RAD)) {
          stopRobot();
          saveCurrentPose();
          state_ = State::MOVE_SIDE;
        }
      }
      break;

    case State::MOVE_SIDE:
      publishCmd(avoidance_speed_, 0.0);
      if (hasMoved(side_distance_m_)) {
        stopRobot();
        saveCurrentPose();
        state_ = State::TURN_FORWARD;
      }
      break;

    case State::TURN_FORWARD:
      {
        // 원래 앞을 보려면 반대로 회전
        double sign = (avoid_dir_ == AvoidDirection::RIGHT) ? 1.0 : -1.0;
        publishCmd(0.0, sign * avoidance_turn_speed_);
        if (hasRotated(TARGET_TURN_RAD)) {
          stopRobot();
          saveCurrentPose();
          state_ = State::MOVE_FORWARD;
        }
      }
      break;

    case State::MOVE_FORWARD:
      publishCmd(avoidance_speed_, 0.0);
      if (hasMoved(forward_distance_m_)) {
        stopRobot();
        saveCurrentPose();
        state_ = State::TURN_RETURN;
      }
      break;

    case State::TURN_RETURN:
      {
        // 원래 경로 쪽으로 바라보기 위해 한 번 더 회전
        double sign = (avoid_dir_ == AvoidDirection::RIGHT) ? 1.0 : -1.0;
        publishCmd(0.0, sign * avoidance_turn_speed_);
        if (hasRotated(TARGET_TURN_RAD)) {
          stopRobot();
          saveCurrentPose();
          state_ = State::MOVE_RETURN;
        }
      }
      break;

    case State::MOVE_RETURN:
      publishCmd(avoidance_speed_, 0.0);
      if (hasMoved(side_distance_m_)) {
        stopRobot();
        saveCurrentPose();
        state_ = State::TURN_RESUME;
      }
      break;

    case State::TURN_RESUME:
      {
        // 최종적으로 다시 원래 가던 앞을 바라봄
        double sign = (avoid_dir_ == AvoidDirection::RIGHT) ? -1.0 : 1.0;
        publishCmd(0.0, sign * avoidance_turn_speed_);
        if (hasRotated(TARGET_TURN_RAD)) {
          stopRobot();
          state_ = State::DONE;
        }
      }
      break;

    case State::DONE:
      RCLCPP_INFO(this->get_logger(), "Avoidance Complete. Handing control back to Nav2.");
      state_ = State::IDLE;
      break;
  }
}

double ObstacleAvoidance::getMinDistance(const sensor_msgs::msg::LaserScan::SharedPtr & scan,
                                         double angle_min_deg, double angle_max_deg)
{
  double min_dist = std::numeric_limits<double>::max();
  
  // LDS-02 특징: 0번 인덱스가 로봇의 정면
  int n_ranges = scan->ranges.size();
  if (n_ranges == 0) return min_dist;

  for (int i = 0; i < n_ranges; ++i) {
    double angle_rad = scan->angle_min + i * scan->angle_increment;
    double angle_deg = normalizeAngle(angle_rad) * 180.0 / M_PI;

    if (angle_deg >= angle_min_deg && angle_deg <= angle_max_deg) {
      double r = scan->ranges[i];
      if (std::isfinite(r) && r > scan->range_min && r < scan->range_max) {
        if (r < min_dist) {
          min_dist = r;
        }
      }
    }
  }
  return min_dist;
}

double ObstacleAvoidance::getLeftClearance(const sensor_msgs::msg::LaserScan::SharedPtr & scan)
{
  // 좌측 80도 ~ 100도 사이의 최소 거리 스캔
  return getMinDistance(scan, 80.0, 100.0);
}

double ObstacleAvoidance::getRightClearance(const sensor_msgs::msg::LaserScan::SharedPtr & scan)
{
  // 우측 -100도 ~ -80도 (또는 260~280도) 사이의 최소 거리 스캔
  return getMinDistance(scan, -100.0, -80.0);
}

void ObstacleAvoidance::publishCmd(double linear, double angular)
{
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = linear;
  cmd.angular.z = angular;
  cmd_vel_pub_->publish(cmd);
}

void ObstacleAvoidance::stopRobot()
{
  publishCmd(0.0, 0.0);
}

bool ObstacleAvoidance::hasRotated(double target_angle_rad)
{
  if (!latest_odom_) return false;
  const auto & q = latest_odom_->pose.pose.orientation;
  double current_yaw = quaternionToYaw(q.x, q.y, q.z, q.w);
  
  double diff = std::abs(normalizeAngle(current_yaw - saved_yaw_));
  return (diff >= target_angle_rad);
}

bool ObstacleAvoidance::hasMoved(double target_distance_m)
{
  if (!latest_odom_) return false;
  double curr_x = latest_odom_->pose.pose.position.x;
  double curr_y = latest_odom_->pose.pose.position.y;
  
  double dist = std::hypot(curr_x - saved_x_, curr_y - saved_y_);
  return (dist >= target_distance_m);
}

void ObstacleAvoidance::saveCurrentPose()
{
  if (!latest_odom_) return;
  saved_x_ = latest_odom_->pose.pose.position.x;
  saved_y_ = latest_odom_->pose.pose.position.y;
  const auto & q = latest_odom_->pose.pose.orientation;
  saved_yaw_ = quaternionToYaw(q.x, q.y, q.z, q.w);
}

double ObstacleAvoidance::quaternionToYaw(double qx, double qy, double qz, double qw)
{
  // tf2 의존성을 없애기 위한 수동 변환 (RPi4 경량화)
  double siny_cosp = 2.0 * (qw * qz + qx * qy);
  double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  return std::atan2(siny_cosp, cosy_cosp);
}

double ObstacleAvoidance::normalizeAngle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

} // namespace sc_cpp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sc_cpp::ObstacleAvoidance)
cmake_minimum_required(VERSION 3.8)
project(sc_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# ── 의존성 ──
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sc_interfaces REQUIRED)

set(DEPS
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  std_msgs
  sc_interfaces
)

# ══════════════════════════════════════════════════════
#  KalmanFilter (공유 정적 라이브러리)
# ══════════════════════════════════════════════════════
add_library(kalman_filter STATIC
  src/kalman_filter.cpp
)
target_include_directories(kalman_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ══════════════════════════════════════════════════════
#  FollowController — 사람 추종 PID 제어
#  subscribe: /person_bbox, /safety_stop
#  publish:   /cmd_vel
# ══════════════════════════════════════════════════════
add_library(follow_controller_component SHARED
  src/follow_controller.cpp
)
target_include_directories(follow_controller_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(follow_controller_component ${DEPS})
target_link_libraries(follow_controller_component kalman_filter)
rclcpp_components_register_node(follow_controller_component
  PLUGIN "sc_cpp::FollowController"
  EXECUTABLE follow_controller
)

# ══════════════════════════════════════════════════════
#  SafetyMonitor — LiDAR 장애물 감지 + 긴급정지
#  subscribe: /scan
#  publish:   /cmd_vel (zero), /safety_stop
# ══════════════════════════════════════════════════════
add_library(safety_monitor_component SHARED
  src/safety_monitor.cpp
)
target_include_directories(safety_monitor_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(safety_monitor_component ${DEPS})
rclcpp_components_register_node(safety_monitor_component
  PLUGIN "sc_cpp::SafetyMonitor"
  EXECUTABLE safety_monitor
)

# ══════════════════════════════════════════════════════
#  LidController — 바구니 뚜껑 서보 제어
#  subscribe: /item_detected
#  publish:   /servo_control, /item_confirm
#  ※ 물체 인식 → 연속 N프레임 확정 → 뚜껑 열기 → 타이머 후 닫기
# ══════════════════════════════════════════════════════
add_library(lid_controller_component SHARED
  src/lid_controller.cpp
)
target_include_directories(lid_controller_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(lid_controller_component ${DEPS})
rclcpp_components_register_node(lid_controller_component
  PLUGIN "sc_cpp::LidController"
  EXECUTABLE lid_controller
)

# ══════════════════════════════════════════════════════
#  PanTiltController — 웹캠 서보(Pan/Tilt) 제어
#  subscribe: /person_bbox
#  publish:   /servo_control
#  ※ 사람 bbox 중심 → Pan(좌우), Tilt(상하) 각도 계산
# ══════════════════════════════════════════════════════
add_library(pan_tilt_controller_component SHARED
  src/pan_tilt_controller.cpp
)
target_include_directories(pan_tilt_controller_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pan_tilt_controller_component ${DEPS})
rclcpp_components_register_node(pan_tilt_controller_component
  PLUGIN "sc_cpp::PanTiltController"
  EXECUTABLE pan_tilt_controller
)

# ══════════════════════════════════════════════════════
#  StatusPublisher — CartStatus 종합 상태 발행
#  subscribe: /person_bbox, /item_detected, /scan, /safety_stop
#  publish:   /cart_status
# ══════════════════════════════════════════════════════
add_library(status_publisher_component SHARED
  src/status_publisher.cpp
)
target_include_directories(status_publisher_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(status_publisher_component ${DEPS})
rclcpp_components_register_node(status_publisher_component
  PLUGIN "sc_cpp::StatusPublisher"
  EXECUTABLE status_publisher
)

# ── Install ──
install(TARGETS
  kalman_filter
  follow_controller_component
  safety_monitor_component
  lid_controller_component
  pan_tilt_controller_component
  status_publisher_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()

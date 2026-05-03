cmake_minimum_required(VERSION 3.8)
project(sc_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # ── 기존 (Python AI ↔ C++ 제어 통신) ──
  "msg/PersonBbox.msg"
  "msg/ItemDetected.msg"
  "msg/ServoControl.msg"
  "msg/CartStatus.msg"
  # ── 신규 (GUI ↔ C++ ↔ Python 통합 pub/sub) ──
  "msg/TrackerState.msg"
  "msg/Waypoint.msg"
  "msg/WaypointList.msg"
  "msg/PaymentEvent.msg"
  "msg/BasketEvent.msg"
  "srv/SaveWaypoints.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

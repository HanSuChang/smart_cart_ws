#!/bin/bash
# ================================================================
# build.sh — 전체 워크스페이스 빌드
# 사용법: chmod +x build.sh && ./build.sh
# ================================================================
set -e

cd "$(dirname "$0")"

source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle_pi

echo "============================================"
echo " [1/4] sc_interfaces 빌드 (반드시 먼저)"
echo "============================================"
colcon build --packages-select sc_interfaces
source install/setup.bash

echo "============================================"
echo " [2/4] sc_cpp 빌드"
echo "============================================"
colcon build --packages-select sc_cpp \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
               -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

echo "============================================"
echo " [3/4] sc_python + sc_gui 빌드"
echo "============================================"
colcon build --packages-select sc_python sc_gui

echo "============================================"
echo " [4/4] sc_bringup 빌드"
echo "============================================"
colcon build --packages-select sc_bringup

source install/setup.bash

echo ""
echo "============================================"
echo " 빌드 완료!"
echo "============================================"
echo ""
echo " 실제 로봇:  ros2 launch sc_bringup robot.launch.py"
echo " Gazebo:     ros2 launch sc_bringup sim.launch.py"
echo " 디버그:     ros2 launch sc_bringup robot.launch.py debug:=true"
echo ""

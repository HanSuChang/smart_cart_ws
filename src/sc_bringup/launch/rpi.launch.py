"""
rpi.launch.py
========================================================
[라즈베리파이4 (TurtleBot3 Waffle Pi 온보드) 전용]

라즈베리파이는 무거운 AI 추론을 안 돌립니다.
대신 실시간 제어 (저지연이 중요한 노드) 는 모두 여기서:

  ── 카메라/주행/센서 ──
    - usb_cam x 2 (사람 추종 + 바구니, mjpeg2rgb)
    - turtlebot3_bringup (LDS-02 LiDAR + 모터 + IMU)

  ── 안전 (네트워크 끊겨도 동작해야 함) ──
    - safety_monitor       LiDAR 기반 비상 정지

  ── 실시간 제어 (★ /cmd_vel, /servo_control 로컬 발행) ──
    - follow_controller    PID + Kalman → /cmd_vel
                           PC 의 person_tracker 가 발행한 /person_bbox 를 WiFi 로 받아 처리.
                           WiFi 끊기면 bbox_timeout 자동 정지 (안전)
    - pan_tilt_controller  /person_bbox → /servo_control (id=0,1)
    - lid_controller       /item_detected, /smart_cart/destination, /payment/event
                           → /servo_control (id=2)

  ── OpenCR 브릿지 (옵션) ──
    - micro_ros_agent      OpenCR 가 micro-ROS 펌웨어로 빌드되면 활성화

[실행]
  ros2 launch sc_bringup rpi.launch.py

[네트워크 설정]
  export ROS_DOMAIN_ID=27       # PC 와 동일
  export ROS_LOCALHOST_ONLY=0
========================================================
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('sc_bringup')
    config = PathJoinSubstitution([pkg, 'config'])

    run_cameras_arg  = DeclareLaunchArgument('run_cameras',  default_value='true')
    run_safety_arg   = DeclareLaunchArgument('run_safety',   default_value='true')
    run_tb3_arg      = DeclareLaunchArgument('run_tb3',      default_value='true')
    run_follow_arg   = DeclareLaunchArgument('run_follow',   default_value='true')
    run_pan_tilt_arg = DeclareLaunchArgument('run_pan_tilt', default_value='true')
    run_lid_arg      = DeclareLaunchArgument('run_lid',      default_value='true')
    run_microros_arg = DeclareLaunchArgument('run_microros', default_value='false')

    run_cameras  = LaunchConfiguration('run_cameras')
    run_safety   = LaunchConfiguration('run_safety')
    run_tb3      = LaunchConfiguration('run_tb3')
    run_follow   = LaunchConfiguration('run_follow')
    run_pan_tilt = LaunchConfiguration('run_pan_tilt')
    run_lid      = LaunchConfiguration('run_lid')
    run_microros = LaunchConfiguration('run_microros')

    # ── usb_cam x 2 (mjpeg2rgb) ──
    webcam1_node = Node(
        package='usb_cam', executable='usb_cam_node_exe', name='webcam',
        parameters=[PathJoinSubstitution([config, 'webcam_params.yaml'])],
        condition=IfCondition(run_cameras),
        output='screen')
    webcam2_node = Node(
        package='usb_cam', executable='usb_cam_node_exe', name='webcam2',
        parameters=[PathJoinSubstitution([config, 'webcam_params.yaml'])],
        condition=IfCondition(run_cameras),
        output='screen')

    # ── turtlebot3_bringup ──
    tb3_bringup = GroupAction(
        condition=IfCondition(run_tb3),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('turtlebot3_bringup'),
                    '/launch/robot.launch.py'
                ]),
            ),
        ],
    )

    # ── safety_monitor (LiDAR 안전정지) ──
    safety_node = Node(
        package='sc_cpp', executable='safety_monitor', name='safety_monitor',
        parameters=[PathJoinSubstitution([config, 'safety_params.yaml'])],
        condition=IfCondition(run_safety),
        output='screen')

    # ★★★ 실시간 제어 노드를 Pi 로 이동 ★★★
    # ── follow_controller (Pi 측) ──
    follow_node = Node(
        package='sc_cpp', executable='follow_controller', name='follow_controller',
        parameters=[PathJoinSubstitution([config, 'follow_params.yaml'])],
        condition=IfCondition(run_follow),
        output='screen')

    # ── pan_tilt_controller (Pi 측) ──
    pan_tilt_node = Node(
        package='sc_cpp', executable='pan_tilt_controller', name='pan_tilt_controller',
        parameters=[PathJoinSubstitution([config, 'servo_params.yaml'])],
        condition=IfCondition(run_pan_tilt),
        output='screen')

    # ── lid_controller (Pi 측) ──
    lid_node = Node(
        package='sc_cpp', executable='lid_controller', name='lid_controller',
        parameters=[PathJoinSubstitution([config, 'lid_params.yaml'])],
        condition=IfCondition(run_lid),
        output='screen')

    # ── micro-ROS agent (OpenCR 시리얼, 옵션) ──
    microros_node = Node(
        package='micro_ros_agent', executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyACM0', '-b', '115200'],
        condition=IfCondition(run_microros),
        output='screen')

    return LaunchDescription([
        run_cameras_arg, run_safety_arg, run_tb3_arg,
        run_follow_arg, run_pan_tilt_arg, run_lid_arg, run_microros_arg,
        # 카메라 / 주행
        webcam1_node, webcam2_node,
        tb3_bringup,
        # 안전
        safety_node,
        # 실시간 제어 (Pi 로 이동)
        follow_node,
        pan_tilt_node,
        lid_node,
        # OpenCR
        microros_node,
    ])

"""
pc.launch.py
========================================================
[우분투 PC (관제) 전용]

Pi 가 발행하는 영상/센서/주행 토픽을 구독해서 다음만 처리:
  ── 무거운 AI / 비전 ──
    - person_tracker     YOLOv8n + BoT-SORT + HSV → /person_bbox
                         (Pi 의 follow_controller 가 이 토픽을 WiFi 로 구독)
    - item_classifier    YOLOv8n → /item_detected
    - basket_vision      OpenCV (배경차분, 모션) → /basket/event

  ── 상태/통신/UI ──
    - status_publisher   여러 토픽 통합 → /cart_status (GUI 용)
    - rosbridge_websocket (port 9090)  GUI 가 roslibpy 로 접속
    - Nav2               자동 주행 (/navigate_to_pose 액션)
    - GUI                PyQt5 + Flask 결제 + ROS 브릿지

[실행]
  ros2 launch sc_bringup pc.launch.py

[네트워크]
  export ROS_DOMAIN_ID=27
  export ROS_LOCALHOST_ONLY=0

[Pi 측에서 영상/센서가 오는지 확인]
  ros2 topic list | grep -E "webcam|scan|odom"
  ros2 topic hz /webcam/image_raw
  ros2 topic hz /scan
========================================================
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('sc_bringup')
    sc_bringup_dir = get_package_share_directory('sc_bringup')
    config = PathJoinSubstitution([pkg, 'config'])

    debug_arg = DeclareLaunchArgument('debug', default_value='false')
    run_item_classifier_arg = DeclareLaunchArgument(
        'run_item_classifier', default_value='true')
    run_basket_vision_arg = DeclareLaunchArgument(
        'run_basket_vision', default_value='true')
    run_nav2_arg = DeclareLaunchArgument('run_nav2', default_value='true')
    run_gui_arg  = DeclareLaunchArgument('run_gui',  default_value='true')
    run_status_arg = DeclareLaunchArgument('run_status', default_value='true')
    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(sc_bringup_dir, 'maps', 'map_cleaned.yaml'))

    debug = LaunchConfiguration('debug')
    run_item_classifier = LaunchConfiguration('run_item_classifier')
    run_basket_vision = LaunchConfiguration('run_basket_vision')
    run_nav2 = LaunchConfiguration('run_nav2')
    run_gui = LaunchConfiguration('run_gui')
    run_status = LaunchConfiguration('run_status')
    map_yaml_file = LaunchConfiguration('map')

    # ── status_publisher (PC 측 — GUI 로 통합 상태 발행) ──
    status_node = Node(
        package='sc_cpp', executable='status_publisher', name='status_publisher',
        condition=IfCondition(run_status),
        output='screen')

    # ── Python AI (PC = YOLO/OpenCV 무거운 작업) ──
    tracker_node = Node(
        package='sc_python', executable='person_tracker', name='person_tracker',
        parameters=[PathJoinSubstitution([config, 'tracker_params.yaml'])],
        output='screen')
    classifier_node = Node(
        package='sc_python', executable='item_classifier', name='item_classifier',
        parameters=[
            PathJoinSubstitution([config, 'classifier_params.yaml']),
            {'show_debug': debug}],
        condition=IfCondition(run_item_classifier),
        output='screen')
    basket_node = Node(
        package='sc_python', executable='basket_vision', name='basket_vision',
        parameters=[PathJoinSubstitution([config, 'basket_vision_params.yaml'])],
        condition=IfCondition(run_basket_vision),
        output='screen')

    # ── rosbridge ──
    rosbridge_node = Node(
        package='rosbridge_server', executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{'port': 9090}],
        output='screen')

    # ── Nav2 ──
    nav2_launch = GroupAction(
        condition=IfCondition(run_nav2),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    FindPackageShare('nav2_bringup'), '/launch/bringup_launch.py'
                ]),
                launch_arguments={
                    'map': map_yaml_file,
                    'use_sim_time': 'false',
                    'params_file': PathJoinSubstitution([config, 'nav2_params.yaml']),
                    'autostart': 'true',
                }.items(),
            ),
        ],
    )

    # ── GUI ──
    gui_node = Node(
        package='sc_gui', executable='frictionless_gui', name='frictionless_gui',
        condition=IfCondition(run_gui),
        output='screen')

    return LaunchDescription([
        debug_arg, run_item_classifier_arg, run_basket_vision_arg,
        run_nav2_arg, run_gui_arg, run_status_arg, map_yaml_arg,
        # 상태
        status_node,
        # AI
        tracker_node, classifier_node, basket_node,
        # 통신
        rosbridge_node,
        # 자율주행
        nav2_launch,
        # GUI
        gui_node,
    ])

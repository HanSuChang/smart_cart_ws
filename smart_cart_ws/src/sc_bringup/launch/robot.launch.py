"""
robot.launch.py
========================================================
Smart Cart 통합 실행 launch (PC에서 실행)

[실행 흐름]
  1. (터틀봇 라즈베리파이) ssh 접속 후:
     ros2 launch turtlebot3_bringup robot.launch.py
     → /scan, /odom, /tf 등 발행

  2. (PC) 이 launch 실행:
     ros2 launch sc_bringup robot.launch.py
     → 카메라 + AI + 제어 + Nav2 + GUI 모두 자동 실행

[포함된 기능]
  - USB 웹캠 2대 (/webcam, /webcam2)
  - person_tracker (YOLOv8n + DeepSORT)
  - item_classifier (옵션, run_item_classifier:=true)
  - follow_controller (PID + Kalman + GUI mode)
  - safety_monitor / status_publisher / pan_tilt / lid
  - Nav2 (AMCL + Planner + Controller)
  - rosbridge_websocket (GUI 통신)
  - GUI (PyQt6 + Flask)

[GUI 토픽 인터페이스]
  - 사람 추종:  GUI → /smart_cart/mode "follow"
  - 자동 주행:  GUI → /smart_cart/mode "navigate" + /navigate_to_pose
  - 비상 정지:  GUI → /smart_cart/mode "idle" + /safety_stop
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

    # ── Launch Arguments ──
    debug_arg = DeclareLaunchArgument(
        'debug', default_value='false',
        description='디버그 창 표시 여부')

    run_item_classifier_arg = DeclareLaunchArgument(
        'run_item_classifier', default_value='false',
        description='물체 인식 노드 실행 여부 (Roboflow 모델 필요)')

    run_nav2_arg = DeclareLaunchArgument(
        'run_nav2', default_value='true',
        description='Nav2 자동 주행 노드 실행 여부')

    run_gui_arg = DeclareLaunchArgument(
        'run_gui', default_value='true',
        description='GUI 자동 실행 여부')

    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(sc_bringup_dir, 'maps', 'map_cleaned.yaml'),
        description='맵 yaml 파일 절대 경로')

    debug = LaunchConfiguration('debug')
    run_item_classifier = LaunchConfiguration('run_item_classifier')
    run_nav2 = LaunchConfiguration('run_nav2')
    run_gui = LaunchConfiguration('run_gui')
    map_yaml_file = LaunchConfiguration('map')

    # ════════════════════════════════════════════════════════
    # 1. USB 웹캠 2대 (사람 추종 + 물체 인식)
    # ════════════════════════════════════════════════════════
    webcam1_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='webcam',
        parameters=[PathJoinSubstitution([config, 'webcam_params.yaml'])],
        remappings=[('/image_raw', '/webcam/image_raw'),
                    ('/image_raw/compressed', '/webcam/image_raw/compressed')],
        output='screen',
    )

    webcam2_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='webcam2',
        parameters=[PathJoinSubstitution([config, 'webcam_params.yaml'])],
        remappings=[('/image_raw', '/webcam2/image_raw'),
                    ('/image_raw/compressed', '/webcam2/image_raw/compressed')],
        output='screen',
    )

    # ════════════════════════════════════════════════════════
    # 2. C++ 노드 (제어 + 안전)
    # ════════════════════════════════════════════════════════
    follow_node = Node(
        package='sc_cpp', executable='follow_controller', name='follow_controller',
        parameters=[PathJoinSubstitution([config, 'follow_params.yaml'])],
        output='screen',
    )

    safety_node = Node(
        package='sc_cpp', executable='safety_monitor', name='safety_monitor',
        parameters=[PathJoinSubstitution([config, 'safety_params.yaml'])],
        output='screen',
    )

    pan_tilt_node = Node(
        package='sc_cpp', executable='pan_tilt_controller', name='pan_tilt_controller',
        parameters=[PathJoinSubstitution([config, 'servo_params.yaml'])],
        output='screen',
    )

    lid_node = Node(
        package='sc_cpp', executable='lid_controller', name='lid_controller',
        parameters=[PathJoinSubstitution([config, 'lid_params.yaml'])],
        output='screen',
    )

    status_node = Node(
        package='sc_cpp', executable='status_publisher', name='status_publisher',
        output='screen',
    )

    # ════════════════════════════════════════════════════════
    # 3. Python AI 노드
    # ════════════════════════════════════════════════════════
    tracker_node = Node(
        package='sc_python', executable='person_tracker', name='person_tracker',
        parameters=[PathJoinSubstitution([config, 'tracker_params.yaml'])],
        output='screen',
    )

    classifier_node = Node(
        package='sc_python', executable='item_classifier', name='item_classifier',
        parameters=[
            PathJoinSubstitution([config, 'classifier_params.yaml']),
            {'show_debug': debug}],
        condition=IfCondition(run_item_classifier),
        output='screen',
    )

    # ════════════════════════════════════════════════════════
    # 4. rosbridge_websocket (GUI 통신용)
    #    GUI(roslibpy)가 9090 포트로 접속해서 토픽 주고받음
    # ════════════════════════════════════════════════════════
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{'port': 9090}],
        output='screen',
    )

    # ════════════════════════════════════════════════════════
    # 5. Nav2 (자동 주행 — AMCL + Planner + Controller)
    # ════════════════════════════════════════════════════════
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

    # ════════════════════════════════════════════════════════
    # 6. GUI (PyQt6 + Flask)
    # ════════════════════════════════════════════════════════
    gui_node = Node(
        package='sc_gui',
        executable='frictionless_gui',
        name='frictionless_gui',
        condition=IfCondition(run_gui),
        output='screen',
    )

    return LaunchDescription([
        # 인자
        debug_arg,
        run_item_classifier_arg,
        run_nav2_arg,
        run_gui_arg,
        map_yaml_arg,
        # 카메라
        webcam1_node,
        webcam2_node,
        # 제어
        safety_node,
        follow_node,
        pan_tilt_node,
        lid_node,
        status_node,
        # AI
        tracker_node,
        classifier_node,
        # 통신
        rosbridge_node,
        # 자율주행
        nav2_launch,
        # GUI
        gui_node,
    ])
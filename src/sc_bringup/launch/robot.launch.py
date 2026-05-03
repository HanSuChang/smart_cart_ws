"""
robot.launch.py
========================================================
Smart Cart 통합 실행 launch

[카메라 패키지: usb_cam (mjpeg2rgb)]
  - 패키지: usb_cam
  - 실행파일: usb_cam_node_exe
  - 토픽: /webcam/image_raw, /webcam/image_raw/compressed
         /webcam2/image_raw, /webcam2/image_raw/compressed

[Pub/Sub 흐름]
  Python(person_tracker)   → /person_bbox, /yolo/follow_image/compressed
  Python(item_classifier)  → /item_detected, /yolo/item_image/compressed
  Python(basket_vision)    → /basket/event, /basket/annotated/compressed
  C++(follow_controller)   → /cmd_vel, /tracker/state, /follow_status
  C++(safety_monitor)      → /safety_stop
  C++(pan_tilt_controller) → /servo_control (id=0,1)
  C++(lid_controller)      → /servo_control (id=2), /item_confirm, /lid_state
                            ※ 화장실 이동 시 /smart_cart/destination "toilet" 수신 → 뚜껑 CLOSE
  C++(status_publisher)    → /cart_status
  GUI(sc_gui)              → 위 모든 토픽 구독
                            → /smart_cart/mode, /smart_cart/learn,
                              /smart_cart/destination, /safety_stop,
                              /payment/event, /waypoint_list, /navigate_to_pose
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
    run_gui_arg = DeclareLaunchArgument('run_gui', default_value='true')
    run_cameras_arg = DeclareLaunchArgument('run_cameras', default_value='true')
    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(sc_bringup_dir, 'maps', 'map_cleaned.yaml'))

    debug = LaunchConfiguration('debug')
    run_item_classifier = LaunchConfiguration('run_item_classifier')
    run_basket_vision = LaunchConfiguration('run_basket_vision')
    run_nav2 = LaunchConfiguration('run_nav2')
    run_gui = LaunchConfiguration('run_gui')
    run_cameras = LaunchConfiguration('run_cameras')
    map_yaml_file = LaunchConfiguration('map')

    # ════════════════════════════════════════════════════════
    # 1. USB 웹캠 2대 — usb_cam (mjpeg2rgb)
    # ════════════════════════════════════════════════════════
    #  설치:  sudo apt install ros-humble-usb-cam
    #  토픽 자동 발행:
    #    /webcam/image_raw, /webcam/image_raw/compressed
    #    /webcam2/image_raw, /webcam2/image_raw/compressed
    webcam1_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='webcam',
        parameters=[PathJoinSubstitution([config, 'webcam_params.yaml'])],
        condition=IfCondition(run_cameras),
        output='screen')

    webcam2_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='webcam2',
        parameters=[PathJoinSubstitution([config, 'webcam_params.yaml'])],
        condition=IfCondition(run_cameras),
        output='screen')

    # ════════════════════════════════════════════════════════
    # 2. C++ 노드
    # ════════════════════════════════════════════════════════
    follow_node = Node(
        package='sc_cpp', executable='follow_controller', name='follow_controller',
        parameters=[PathJoinSubstitution([config, 'follow_params.yaml'])],
        output='screen')
    safety_node = Node(
        package='sc_cpp', executable='safety_monitor', name='safety_monitor',
        parameters=[PathJoinSubstitution([config, 'safety_params.yaml'])],
        output='screen')
    pan_tilt_node = Node(
        package='sc_cpp', executable='pan_tilt_controller', name='pan_tilt_controller',
        parameters=[PathJoinSubstitution([config, 'servo_params.yaml'])],
        output='screen')
    lid_node = Node(
        package='sc_cpp', executable='lid_controller', name='lid_controller',
        parameters=[PathJoinSubstitution([config, 'lid_params.yaml'])],
        output='screen')
    status_node = Node(
        package='sc_cpp', executable='status_publisher', name='status_publisher',
        output='screen')

    # ════════════════════════════════════════════════════════
    # 3. Python 노드
    # ════════════════════════════════════════════════════════
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

    # ════════════════════════════════════════════════════════
    # 4. rosbridge
    # ════════════════════════════════════════════════════════
    rosbridge_node = Node(
        package='rosbridge_server', executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{'port': 9090}],
        output='screen')

    # ════════════════════════════════════════════════════════
    # 5. Nav2
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
    # 6. GUI (PyQt5 + Flask + ROS 브릿지)
    # ════════════════════════════════════════════════════════
    gui_node = Node(
        package='sc_gui', executable='frictionless_gui', name='frictionless_gui',
        condition=IfCondition(run_gui),
        output='screen')

    return LaunchDescription([
        debug_arg, run_item_classifier_arg, run_basket_vision_arg,
        run_nav2_arg, run_gui_arg, run_cameras_arg, map_yaml_arg,
        webcam1_node, webcam2_node,
        safety_node, follow_node, pan_tilt_node, lid_node, status_node,
        tracker_node, classifier_node, basket_node,
        rosbridge_node,
        nav2_launch,
        gui_node,
    ])

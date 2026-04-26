"""
robot.launch.py
실제 TurtleBot3 Waffle Pi + USB 웹캠 + RPi 카메라 환경
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('sc_bringup')
    config = PathJoinSubstitution([pkg, 'config'])

    debug_arg = DeclareLaunchArgument('debug', default_value='false')
    debug = LaunchConfiguration('debug')

    # ── USB 웹캠 (사람 추종용) ──
    webcam_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='webcam',
        parameters=[PathJoinSubstitution([config, 'webcam_params.yaml'])],
        remappings=[('/image_raw', '/webcam/image_raw')],
    )

    # TODO: RPi 카메라 노드 (물체 인식용)
    # 라즈베리파이에서 실행. camera_ros 또는 v4l2_camera로 /rpi_camera/image_raw

    # ── C++ 노드들 ──
    safety_node = Node(
        package='sc_cpp', executable='safety_monitor', name='safety_monitor',
        parameters=[PathJoinSubstitution([config, 'safety_params.yaml'])],
        output='screen',
    )
    follow_node = Node(
        package='sc_cpp', executable='follow_controller', name='follow_controller',
        parameters=[PathJoinSubstitution([config, 'follow_params.yaml'])],
        output='screen',
    )
    lid_node = Node(
        package='sc_cpp', executable='lid_controller', name='lid_controller',
        parameters=[PathJoinSubstitution([config, 'lid_params.yaml'])],
        output='screen',
    )
    pan_tilt_node = Node(
        package='sc_cpp', executable='pan_tilt_controller', name='pan_tilt_controller',
        parameters=[PathJoinSubstitution([config, 'servo_params.yaml'])],
        output='screen',
    )
    status_node = Node(
        package='sc_cpp', executable='status_publisher', name='status_publisher',
        output='screen',
    )

    # ── Python 노드들 ──
    tracker_node = Node(
        package='sc_python', executable='person_tracker', name='person_tracker',
        parameters=[
            PathJoinSubstitution([config, 'tracker_params.yaml']),
            {'show_debug': debug},
        ],
        output='screen',
    )
    classifier_node = Node(
        package='sc_python', executable='item_classifier', name='item_classifier',
        parameters=[
            PathJoinSubstitution([config, 'classifier_params.yaml']),
            {'show_debug': debug},
        ],
        output='screen',
    )

    return LaunchDescription([
        debug_arg,
        webcam_node,
        safety_node,
        follow_node,
        lid_node,
        pan_tilt_node,
        status_node,
        tracker_node,
        classifier_node,
    ])

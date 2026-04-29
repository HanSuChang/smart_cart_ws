"""
sim.launch.py
Gazebo 시뮬레이션 환경 (WSL2 / Ubuntu PC)
TurtleBot3 Waffle Pi + 가상 카메라
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('sc_bringup')
    config = PathJoinSubstitution([pkg, 'config'])

    world_arg = DeclareLaunchArgument(
        'world', default_value='empty_world.world',
        description='Gazebo world 파일명 (worlds/ 폴더 안)')

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
    status_node = Node(
        package='sc_cpp', executable='status_publisher', name='status_publisher',
        output='screen',
    )

    tracker_node = Node(
        package='sc_python', executable='person_tracker', name='person_tracker',
        parameters=[PathJoinSubstitution([config, 'tracker_params.yaml'])],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        safety_node,
        follow_node,
        status_node,
        tracker_node,
    ])

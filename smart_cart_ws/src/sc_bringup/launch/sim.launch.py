"""
sim.launch.py
Gazebo 시뮬레이션 환경 (WSL2 / Ubuntu PC)
TurtleBot3 Waffle Pi + 가상 카메라
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('sc_bringup')
    config = PathJoinSubstitution([pkg, 'config'])

    # ── Gazebo world 선택 ──
    world_arg = DeclareLaunchArgument(
        'world', default_value='empty_world.world',
        description='Gazebo world 파일명 (worlds/ 폴더 안)')
    world = LaunchConfiguration('world')

    # ── TurtleBot3 Gazebo launch (turtlebot3_gazebo 패키지) ──
    # TODO: turtlebot3_gazebo의 turtlebot3_world.launch.py를 include하거나
    #       커스텀 world 파일 사용
    # gazebo_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('turtlebot3_gazebo'),
    #             'launch', 'turtlebot3_world.launch.py'
    #         ])
    #     ])
    # )

    # ── C++ 노드들 (Gazebo에서도 동일하게 실행) ──
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

    # ── Python 노드 (시뮬레이션용) ──
    tracker_node = Node(
        package='sc_python', executable='person_tracker', name='person_tracker',
        parameters=[
            PathJoinSubstitution([config, 'tracker_params.yaml']),
            {'show_debug': True},
        ],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        # gazebo_launch,  # TODO: 주석 해제하여 Gazebo 포함
        safety_node,
        follow_node,
        status_node,
        tracker_node,
    ])

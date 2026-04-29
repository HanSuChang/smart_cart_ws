"""
bringup.launch.py
라즈베리파이에서 실행 — turtlebot3_bringup + USB 웹캠 2대
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('sc_bringup'), 'config'])

    # ── turtlebot3_bringup ──
    turtlebot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_bringup'), 'launch', 'robot.launch.py'
            ])
        ])
    )

    # ── USB 웹캠 1번 (사람 추종용) ──
    webcam1_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='webcam',
        parameters=[PathJoinSubstitution([config, 'webcam_params.yaml'])],
        remappings=[
            ('/image_raw', '/webcam/image_raw'),
            ('/image_raw/compressed', '/webcam/image_raw/compressed'),
        ],
        output='screen',
    )

    # ── USB 웹캠 2번 (물체 인식용) ──
    webcam2_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='webcam2',
        parameters=[PathJoinSubstitution([config, 'webcam_params.yaml'])],
        remappings=[
            ('/image_raw', '/webcam2/image_raw'),
            ('/image_raw/compressed', '/webcam2/image_raw/compressed'),
        ],
        output='screen',
    )

    return LaunchDescription([
        turtlebot_bringup,
        webcam1_node,
        webcam2_node,
    ])
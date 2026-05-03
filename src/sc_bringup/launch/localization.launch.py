import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    sc_bringup_dir = get_package_share_directory('sc_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(sc_bringup_dir, 'maps', 'map_cleaned.yaml'),
            description='Full path to map yaml file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(sc_bringup_dir, 'config', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use'),

        # 1. Localization 실행 (Map Server + AMCL)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
            ),
            launch_arguments={
                'map': map_yaml_file,
                'params_file': params_file,
                'use_sim_time': 'false',
                'autostart': 'true',
            }.items(),
        ),

        # 2. Rviz2 실행
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(sc_bringup_dir, 'rviz', 'smart_cart.rviz')],
            output='screen'),
    ])
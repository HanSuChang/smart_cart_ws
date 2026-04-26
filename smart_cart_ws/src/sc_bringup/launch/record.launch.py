from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2bag',
            executable='record',
            arguments=[
                '-o', '~/smart_cart_ws/src/sc_bringup/bags/run',
                '/person_bbox',
                '/item_detected',
                '/scan',
                '/cmd_vel',
                '/servo_control',
                '/webcam/image_raw',
                '/rpi_camera/image_raw',
            ]
        )
    ])
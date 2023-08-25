import os

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            output='screen',
            name='rplidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/ttyUSB0',
                'frame_id': 'laser_frame',
                'inverted': False,
                'serial_baudrate': 115200,
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }],
        )
    ])

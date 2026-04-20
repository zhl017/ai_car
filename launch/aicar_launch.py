#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ai_car',
            executable='aicar_base',
            name='aicar_base',
            output='screen'
        ),
        Node(
            package='ai_car',
            executable='aicar_imu',
            name='aicar_imu',
            output='screen'
        ),
        Node(
            package='ai_car',
            executable='aicar_odom',
            name='aicar_odom',
            output='screen'
        ),
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            output='screen',
            parameters=[{
                'width': 320,
                'height': 240,
            }]
        ),
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            output='screen',
            parameters=[{
                'serial_port': '/dev/RPLIDAR',
                'serial_baudrate': 115200,
                'frame_id': 'base_scan',
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }]
        ),
    ])
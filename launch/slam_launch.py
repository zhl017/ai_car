#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('ai_car')
    urdf_file = os.path.join(pkg, 'urdf', 'ai_car.urdf')
    slam_rviz_file = os.path.join(pkg, 'rviz', 'slam.rviz')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # lifecycle configure
    slam_configure = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'configure'],
        output='screen'
    )

    # lifecycle activate
    slam_activate = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/slam_toolbox', 'activate'],
        output='screen'
    )

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # Static TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_scan',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_scan']
        ),

        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[os.path.join(pkg, 'config', 'mapper_params.yaml')],
        ),

        # wait 3s configure
        TimerAction(
            period=3.0,
            actions=[slam_configure]
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=slam_configure,
                on_exit=[slam_activate]
            )
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', slam_rviz_file]
        ),
    ])
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='canera_server',
            executable='vamp_server_node',
            name='vamp_server_node',
            output='screen',
            parameters=[
                {'socket_path': '/tmp/koch_vamp_server.sock'},
                {'auto_start_server': True},
                {'status_check_interval_ms': 5000}
            ]
        )
    ])

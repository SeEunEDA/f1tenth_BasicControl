#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node as LaunchNode
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    pkg_share = Path(get_package_share_directory('twist_to_ackermann_pid'))
    params = pkg_share / 'config' / 'twist_to_ackermann_pid.yaml'

    node = LaunchNode(
        package='twist_to_ackermann_pid',
        executable='twist_to_ackermann_pid',  # setup.py entry_points와 일치해야 함
        name='twist_to_ackermann_pid',
        output='screen',
        parameters=[str(params)],
        # 필요하면 오돔 리맵 사용:
        # remappings=[('/odom', '/ego_racecar/odom')],
    )

    return LaunchDescription([node])

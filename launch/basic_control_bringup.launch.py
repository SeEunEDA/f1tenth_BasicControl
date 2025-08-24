#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    # 1) F1TENTH Gym bridge 포함 (sim_ws에 f1tenth_gym_ros가 빌드되어 있어야 함)
    gym_share = Path(get_package_share_directory('f1tenth_gym_ros'))
    gym_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gym_share / 'launch' / 'gym_bridge_launch.py'))
    )

    # 2) Twist -> Ackermann + PID
    my_share = Path(get_package_share_directory('twist_to_ackermann_pid'))
    pid_params = str(my_share / 'config' / 'twist_to_ackermann_pid.yaml')
    pid_node = Node(
        package='twist_to_ackermann_pid',
        executable='twist_to_ackermann_pid',
        name='twist_to_ackermann_pid',
        output='screen',
        parameters=[pid_params],
        # 필요 시 오돔 리맵:
        # remappings=[('/odom', '/ego_racecar/odom')],
    )

    return LaunchDescription([gym_launch, pid_node])

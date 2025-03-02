#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    leader_arg = DeclareLaunchArgument(
        'leader',
        default_value='waffle1',
        description='Name of the leader robot'
    )
    
    follower_arg = DeclareLaunchArgument(
        'follower',
        default_value='waffle2',
        description='Name of the follower robot'
    )
    
    return LaunchDescription([
        leader_arg,
        follower_arg,
        Node(
            package='platoon_simulations',  # Replace with your package name
            executable='formation_control_node.py',  # Replace with your executable name
            name='platoon_control',
            output='screen',
            parameters=[
                {'leader': LaunchConfiguration('leader')},
                {'follower': LaunchConfiguration('follower')}
            ]
        )
    ])
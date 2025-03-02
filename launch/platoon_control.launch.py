#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    leader_arg1 = DeclareLaunchArgument(
        'leader1',
        default_value='waffle1',
        description='Name of the leader robot'
    )
    
    follower_arg1 = DeclareLaunchArgument(
        'follower1',
        default_value='waffle2',
        description='Name of the follower robot'
    )

    leader_arg2 = DeclareLaunchArgument(
        'leader2',
        default_value='waffle2',
        description='Name of the leader robot'
    )
    
    follower_arg2 = DeclareLaunchArgument(
        'follower2',
        default_value='waffle3',
        description='Name of the follower robot'
    )
    
    return LaunchDescription([
        leader_arg1,
        follower_arg1,
        leader_arg2,
        follower_arg2,
        Node(
            package='platoon_simulations',  # Replace with your package name
            executable='formation_control_node.py',  # Replace with your executable name
            name='platoon_control_1',
            output='screen',
            parameters=[
                {'leader': LaunchConfiguration('leader1')},
                {'follower': LaunchConfiguration('follower1')}
            ]
        ),

        Node(
            package='platoon_simulations',  # Replace with your package name
            executable='formation_control_node.py',  # Replace with your executable name
            name='platoon_control_2',
            output='screen',
            parameters=[
                {'leader': LaunchConfiguration('leader2')},
                {'follower': LaunchConfiguration('follower2')}
            ]
        )
    ])
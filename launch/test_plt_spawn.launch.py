#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_platoon_simulations = FindPackageShare('platoon_simulations')
    spawn_robot_launch = PathJoinSubstitution([pkg_platoon_simulations, 'launch', 'spawn_robot.launch.py'])

    # Define the robots to spawn
    robots = [
        {'prefix': 'waffle1', 'x': '2.0', 'y': '0.0'},
        {'prefix': 'waffle2', 'x': '1.0', 'y': '0.0'},
        # {'prefix': 'burger3', 'x': '0.0', 'y': '1.0'},
        # Add more robots as needed
    ]

    # Create a list to hold all the include launch descriptions
    spawn_launches = []

    for robot in robots:
        spawn_launches.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(spawn_robot_launch),
                launch_arguments={
                    'robot_prefix': robot['prefix'],
                    'x_pose': robot['x'],
                    'y_pose': robot['y']
                }.items()
            )
        )

    return LaunchDescription(spawn_launches)

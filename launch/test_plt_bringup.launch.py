#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Package directories
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_platoon_sim = get_package_share_directory('platoon_simulations')

    # Launch arguments
    verbose_arg = DeclareLaunchArgument('verbose', default_value='false',
                          description='Open Gazebo in verbose mode.')
    verbose = LaunchConfiguration('verbose')

    world_name = LaunchConfiguration('world_name')
    world_name_arg = DeclareLaunchArgument(
          'world_name',
          default_value='empty.world',
          description='SDF world file name')

    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
        launch_arguments = {
            'world': PathJoinSubstitution([pkg_platoon_sim,'worlds', world_name]),
            'verbose': verbose,
            'gui': 'true',
        }.items()
    )

    # Robot spawning
    spawn_robot_launch = PathJoinSubstitution([pkg_platoon_sim, 'launch', 'spawn_robot.launch.py'])

    # Define the robots to spawn
    robots = [
        {'prefix': 'waffle1', 'x': '2.0', 'y': '0.0'},
        {'prefix': 'waffle2', 'x': '1.0', 'y': '0.0'},
        # Add more robots as needed
    ]

    # Create a list to hold all the include launch descriptions for robot spawning and trajectory nodes
    spawn_and_trajectory_launches = []

    for robot in robots:
        # Spawn robot
        spawn_and_trajectory_launches.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(spawn_robot_launch),
                launch_arguments={
                    'robot_prefix': robot['prefix'],
                    'x_pose': robot['x'],
                    'y_pose': robot['y']
                }.items()
            )
        )

        # Trajectory node
        spawn_and_trajectory_launches.append(
            Node(
                package='platoon_simulations',
                executable='generate_trajectory_node.py',
                name=f'{robot["prefix"]}_trajectory_node',
                namespace=robot['prefix'],
                remappings=[
                    ('robot_trajectory', f'/{robot["prefix"]}/robot_trajectory')
                ]
            )
        )

    # Group action for spawning robots and launching trajectory nodes
    spawn_and_trajectory = GroupAction(spawn_and_trajectory_launches)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(verbose_arg)
    ld.add_action(world_name_arg)
    ld.add_action(gazebo_launch)
    ld.add_action(spawn_and_trajectory)

    return ld

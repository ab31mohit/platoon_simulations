#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import yaml

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

    param_file_arg = DeclareLaunchArgument(
        'param_file',
        default_value=os.path.join(pkg_platoon_sim, 'params', 'platoon_config.yaml'),
        description='Path to the robot configuration parameter file'
    )
    param_file = LaunchConfiguration('param_file')

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

    def launch_setup(context, *args, **kwargs):
        # Load robot configurations
        param_file_path = param_file.perform(context)
        with open(param_file_path, 'r') as f:
            robot_params = yaml.safe_load(f)

        # Create a list to hold all the include launch descriptions for robot spawning and trajectory nodes
        spawn_and_trajectory_launches = []

        for robot_name, robot in robot_params.items():
            # Spawn robot
            spawn_and_trajectory_launches.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(spawn_robot_launch),
                    launch_arguments={
                        'robot_prefix': robot['robot_prefix'],
                        'x_pose': str(robot['x_pose']),
                        'y_pose': str(robot['y_pose']),
                        # 'robot_model': robot['robot_model']
                    }.items()
                )
            )

            # Trajectory node
            spawn_and_trajectory_launches.append(
                Node(
                    package='platoon_simulations',
                    executable='generate_trajectory_node.py',
                    name=f'{robot["robot_prefix"]}_trajectory_node',
                    namespace=robot['robot_prefix'],
                    remappings=[
                        ('robot_trajectory', f'/{robot["robot_prefix"]}/robot_trajectory'),
                    ]
                )
            )

        return spawn_and_trajectory_launches

    # OpaqueFunction to load parameters and create nodes
    spawn_and_trajectory = OpaqueFunction(function=launch_setup)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(verbose_arg)
    ld.add_action(world_name_arg)
    ld.add_action(param_file_arg)
    ld.add_action(gazebo_launch)
    ld.add_action(spawn_and_trajectory)

    return ld

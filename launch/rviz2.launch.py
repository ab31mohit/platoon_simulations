#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package name (replace 'your_package_name' with your actual package name)
    package_name = 'platoon_simulations'
    
    # Get the path to the RViz configuration file
    rviz_config_path = os.path.join(
        get_package_share_directory(package_name),
        'rviz',
        'plt_2robots.rviz'
    )
    
    # Launch RViz node with the specified configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )
    
    # Create and return the launch description
    return LaunchDescription([rviz_node])

#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('glim_ros')

    # Declare launch arguments
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=os.path.join(pkg_dir, 'config'),
        description='Path to configuration directory'
    )

    global_map_path_arg = DeclareLaunchArgument(
        'global_map_path',
        default_value='',
        description='Path to global map file (.pcd or directory containing submaps)'
    )

    auto_initial_pose_arg = DeclareLaunchArgument(
        'auto_initial_pose',
        default_value='false',
        description='Use identity as initial pose (true/false)'
    )

    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug mode (true/false)'
    )

    # Localization node
    localization_node = Node(
        package='glim_ros',
        executable='glim_localization_rosnode',
        name='glim_localization',
        output='screen',
        parameters=[{
            'config_path': LaunchConfiguration('config_path'),
            'global_map_path': LaunchConfiguration('global_map_path'),
            'auto_initial_pose': LaunchConfiguration('auto_initial_pose'),
            'debug': LaunchConfiguration('debug'),
        }]
    )

    return LaunchDescription([
        config_path_arg,
        global_map_path_arg,
        auto_initial_pose_arg,
        debug_arg,
        localization_node,
    ])

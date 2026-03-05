#!/usr/bin/env python3

"""
GLIM ROS2 Launch File - Localization Mode
This launch file starts glim_ros in localization mode against a pre-built map.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    # Get package directories
    glim_ros_dir = get_package_share_directory('glim_ros')
    glim_dir = get_package_share_directory('glim')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    params_file = LaunchConfiguration('params_file')
    map_path = LaunchConfiguration('map_path')
    debug = LaunchConfiguration('debug')

    # Get actual values
    config_path_value = config_path.perform(context)
    params_file_value = params_file.perform(context)
    map_path_value = map_path.perform(context)

    # Validate map_path
    if not map_path_value:
        raise ValueError('map_path is required for localization mode!')

    # Determine config path
    if not config_path_value:
        # Default to velodyne config in glim package
        config_path_value = os.path.join(glim_dir, 'config', 'livox')
    elif not os.path.isabs(config_path_value):
        # Relative path, resolve from glim package
        config_path_value = os.path.join(glim_dir, config_path_value)

    # Determine params file
    if not params_file_value:
        # Default to localization params file
        params_file_value = os.path.join(glim_ros_dir, 'config', 'glim_localization_params.yaml')
    elif not os.path.isabs(params_file_value):
        # Relative path, resolve from glim_ros package
        params_file_value = os.path.join(glim_ros_dir, params_file_value)

    # Resolve map_path
    if not os.path.isabs(map_path_value):
        # Try relative to glim package first
        map_path_value = os.path.join(glim_dir, map_path_value)

    # Check if map exists
    if not os.path.exists(map_path_value):
        raise FileNotFoundError(f'Map path does not exist: {map_path_value}')

    # Node parameters - LOCALIZATION MODE ENABLED
    node_params = {
        'use_sim_time': use_sim_time,
        'config_path': config_path_value,
        'debug': debug,
        'localization_mode': True,  # Enable localization mode
        'map_path': map_path_value,  # Pre-built map path
        'dump_on_unload': False,  # Don't dump in localization mode
    }

    # Load additional parameters from YAML if exists
    if os.path.exists(params_file_value):
        import yaml
        with open(params_file_value, 'r') as f:
            yaml_params = yaml.safe_load(f)
            if yaml_params and 'glim_ros' in yaml_params:
                if 'ros__parameters' in yaml_params['glim_ros']:
                    node_params.update(yaml_params['glim_ros']['ros__parameters'])

    # GLIM ROS Node in Localization Mode
    glim_ros_node = Node(
        package='glim_ros',
        executable='glim_rosnode',
        name='glim_ros',
        output='screen',
        parameters=[node_params],
        remappings=[
            # Add remappings here if needed
            # ('/imu', '/your_imu_topic'),
            # ('/velodyne_points', '/your_points_topic'),
        ],
    )

    return [glim_ros_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'config_path',
            default_value='',
            description='Path to glim config directory (default: glim/config/velodyne)'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value='',
            description='Path to ROS parameters YAML file (default: glim_ros/config/glim_localization_params.yaml)'
        ),
        DeclareLaunchArgument(
            'map_path',
            default_value='',
            description='Path to pre-built map directory (REQUIRED)'
        ),
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Enable debug logging'
        ),
        OpaqueFunction(function=launch_setup)
    ])

#!/usr/bin/env python3

"""
GLIM ROS2 Launch File - SLAM Mode
This launch file starts glim_ros in standard SLAM mode for mapping.
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
    debug = LaunchConfiguration('debug')
    dump_on_unload = LaunchConfiguration('dump_on_unload')

    # Get actual values
    config_path_value = config_path.perform(context)
    params_file_value = params_file.perform(context)

    # Determine config path
    if not config_path_value:
        # Default to velodyne config in glim package
        config_path_value = os.path.join(glim_dir, 'config', 'velodyne')
    elif not os.path.isabs(config_path_value):
        # Relative path, resolve from glim package
        config_path_value = os.path.join(glim_dir, config_path_value)

    # Determine params file
    if not params_file_value:
        # Default params file in glim_ros
        params_file_value = os.path.join(glim_ros_dir, 'config', 'glim_ros_params.yaml')
    elif not os.path.isabs(params_file_value):
        # Relative path, resolve from glim_ros package
        params_file_value = os.path.join(glim_ros_dir, params_file_value)

    # Node parameters
    node_params = {
        'use_sim_time': use_sim_time,
        'config_path': config_path_value,
        'debug': debug,
        'dump_on_unload': dump_on_unload,
    }

    # Load additional parameters from YAML if exists
    if os.path.exists(params_file_value):
        import yaml
        with open(params_file_value, 'r') as f:
            yaml_params = yaml.safe_load(f)
            if yaml_params and 'glim_ros' in yaml_params:
                if 'ros__parameters' in yaml_params['glim_ros']:
                    node_params.update(yaml_params['glim_ros']['ros__parameters'])

    # GLIM ROS Node
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
            description='Path to ROS parameters YAML file (default: glim_ros/config/glim_ros_params.yaml)'
        ),
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Enable debug logging'
        ),
        DeclareLaunchArgument(
            'dump_on_unload',
            default_value='false',
            description='Dump map on node shutdown'
        ),
        OpaqueFunction(function=launch_setup)
    ])

#!/usr/bin/env python3
# Copyright 2024 RealSense, Inc. All Rights Reserved.
#
# RealSense D455 + OctoMap + Foxglove Studio Web Visualization
# This launch file provides web-based visualization through Foxglove Studio

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directories
    realsense_pkg_dir = get_package_share_directory('realsense2_camera')
    
    # Declare launch arguments
    declare_camera_name = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='Camera unique name'
    )
    
    declare_camera_namespace = DeclareLaunchArgument(
        'camera_namespace',
        default_value='camera',
        description='Namespace for camera'
    )
    
    declare_octomap_resolution = DeclareLaunchArgument(
        'octomap_resolution',
        default_value='0.05',
        description='OctoMap resolution in meters'
    )
    
    declare_camera_height = DeclareLaunchArgument(
        'camera_height',
        default_value='0.5',
        description='Camera height above base_link in meters'
    )
    
    declare_decimation_filter = DeclareLaunchArgument(
        'decimation_filter',
        default_value='2',
        description='Decimation filter magnitude (1-8, higher = faster but lower quality)'
    )
    
    declare_foxglove_port = DeclareLaunchArgument(
        'foxglove_port',
        default_value='8765',
        description='Foxglove Bridge WebSocket port'
    )
    
    declare_foxglove_address = DeclareLaunchArgument(
        'foxglove_address',
        default_value='0.0.0.0',
        description='Foxglove Bridge listen address (0.0.0.0 for external access)'
    )
    
    # RealSense camera node with point cloud enabled
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(realsense_pkg_dir, 'launch', 'rs_launch.py')
        ]),
        launch_arguments={
            'camera_name': LaunchConfiguration('camera_name'),
            'camera_namespace': LaunchConfiguration('camera_namespace'),
            'enable_color': 'true',
            'enable_depth': 'true',
            'pointcloud.enable': 'true',
            'pointcloud.stream_filter': '2',  # Use color texture for point cloud
            'pointcloud.ordered_pc': 'false',
            'pointcloud.allow_no_texture_points': 'false',
            'align_depth.enable': 'true',  # Align depth to color
            'decimation_filter.enable': 'true',
            'decimation_filter.filter_magnitude': LaunchConfiguration('decimation_filter'),
            'spatial_filter.enable': 'true',  # Reduce noise
            'temporal_filter.enable': 'true',  # Reduce temporal noise
            'publish_tf': 'true',
            'tf_publish_rate': '10.0',
        }.items()
    )
    
    # Static transform: map -> base_link
    static_tf_map_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        output='log'
    )
    
    # Static transform: base_link -> camera_link
    static_tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_camera',
        arguments=[
            '0', '0', LaunchConfiguration('camera_height'),  # x, y, z
            '0', '0', '0',  # roll, pitch, yaw
            'base_link', 'camera_link'
        ],
        output='log'
    )
    
    # OctoMap Server node (with 4m range limit)
    octomap_node = Node(
        package='octomap_server2',
        executable='octomap_server',
        name='octomap_server',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('octomap_server2'),
                'config',
                'realsense_params.yaml'
            ]),
            {
                'resolution': LaunchConfiguration('octomap_resolution'),
                'frame_id': 'map',
                'base_frame_id': 'base_link',
            }
        ],
        remappings=[
            ('cloud_in', '/camera/camera/depth/color/points')
        ]
    )
    
    # Foxglove Bridge for web visualization
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'port': LaunchConfiguration('foxglove_port'),
            'address': LaunchConfiguration('foxglove_address'),
            'tls': False,
            'certfile': '',
            'keyfile': '',
            'topic_whitelist': ['.*'],  # Allow all topics
            'service_whitelist': ['.*'],
            'param_whitelist': ['.*'],
            'send_buffer_limit': 10000000,
            'use_compression': True,
            'capabilities': [
                'clientPublish',
                'parameters',
                'parametersSubscribe',
                'services',
                'connectionGraph',
                'assets'
            ],
            'asset_uri_allowlist': ['.*'],
        }],
        output='screen'
    )
    
    return LaunchDescription([
        # Declare arguments
        declare_camera_name,
        declare_camera_namespace,
        declare_octomap_resolution,
        declare_camera_height,
        declare_decimation_filter,
        declare_foxglove_port,
        declare_foxglove_address,
        
        # Launch nodes
        realsense_node,
        static_tf_map_to_base,
        static_tf_base_to_camera,
        octomap_node,
        foxglove_bridge,
    ])

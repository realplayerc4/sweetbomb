#!/usr/bin/env python3
# Copyright 2024 RealSense, Inc. All Rights Reserved.
#
# Integration launch file for RealSense D455 with OctoMap Server
# This launch file combines:
# - RealSense camera with point cloud generation
# - OctoMap Server for 3D occupancy mapping
# - Static TF transforms for coordinate frames
# - RViz2 visualization

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
    octomap_pkg_dir = get_package_share_directory('octomap_server2')
    
    # Declare launch arguments
    declare_camera_name = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='Camera unique name'
    )
    
    declare_camera_namespace = DeclareLaunchArgument(
        'camera_namespace',
        default_value='',
        description='Namespace for camera (leave empty if using camera_name only)'
    )
    
    declare_enable_rviz = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )
    
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('realsense2_camera'),
            'launch',
            'octomap_visualization.rviz'
        ]),
        description='Path to RViz configuration file'
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
        default_value='6',
        description='Decimation filter magnitude (1-8, higher = faster but lower quality)'
    )
    
    declare_pointcloud_topic = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/camera/depth/color/points',
        description='Point cloud topic name from camera'
    )
    
    declare_clip_distance = DeclareLaunchArgument(
        'clip_distance',
        default_value='3.0',
        description='Maximum distance for point cloud filtering (meters, set to -1 to disable)'
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
            'rgb_camera.color_profile': '1280,720,30',  # ⚡ 1280x720 @ 30fps
            'depth_module.depth_profile': '1280,720,30',  # ⚡ 1280x720 @ 30fps
            'pointcloud.enable': 'true',
            'pointcloud.stream_filter': '2',  # Use color texture for point cloud
            'pointcloud.ordered_pc': 'false',
            'pointcloud.allow_no_texture_points': 'false',
            'align_depth.enable': 'true',  # Align depth to color
            'decimation_filter.enable': 'true',
            'decimation_filter.filter_magnitude': LaunchConfiguration('decimation_filter'),  # Default: 4
            'spatial_filter.enable': 'true',  # Reduce noise
            'temporal_filter.enable': 'true',  # Reduce temporal noise
            'clip_distance': LaunchConfiguration('clip_distance'),  # ⚡ Clip point cloud at max distance
            'publish_tf': 'true',
            'tf_publish_rate': '10.0',
        }.items()
    )
    
    # Static transform: map -> base_link
    # This assumes the camera is mounted on a mobile robot
    # For a fixed camera setup, you might want map at the camera location
    static_tf_map_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
        output='log'
    )
    
    # Static transform: base_link -> camera_link
    # Adjust these values based on your actual camera mounting position
    # Format: x y z yaw pitch roll parent_frame child_frame
    # Default: camera is 0.5m above base_link, facing forward
    static_tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_camera',
        arguments=[
            '0', '0', LaunchConfiguration('camera_height'),  # x, y, z
            '0', '0', '0',  # roll, pitch, yaw (quaternion identity)
            'base_link', 'camera_link'
        ],
        output='log'
    )
    
    # OctoMap Server node
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
                'sensor_model/hit': 0.97,
                'sensor_model/miss': 0.01,
                'sensor_model/min': 0.49,
                'sensor_model/max': 0.97,
                'compress_map': True,
            }
        ],
        remappings=[
            ('cloud_in', LaunchConfiguration('pointcloud_topic'))
        ]
    )
    
    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_rviz'))
    )
    
    return LaunchDescription([
        # Declare arguments
        declare_camera_name,
        declare_camera_namespace,
        declare_enable_rviz,
        declare_rviz_config,
        declare_octomap_resolution,
        declare_camera_height,
        declare_decimation_filter,
        declare_pointcloud_topic,
        declare_clip_distance,
        
        # Launch nodes
        realsense_node,
        static_tf_map_to_base,
        static_tf_base_to_camera,
        octomap_node,
        rviz_node,
    ])

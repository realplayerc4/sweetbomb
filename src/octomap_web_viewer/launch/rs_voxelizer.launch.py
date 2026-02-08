#!/usr/bin/env python3
"""
RealSense + 轻量级体素化 Launch 文件
替代 rs_octomap.launch.py，使用 pointcloud_voxelizer 代替 octomap_server2
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package directories
    realsense_pkg_dir = get_package_share_directory('realsense2_camera')
    web_viewer_pkg_dir = get_package_share_directory('octomap_web_viewer')
    
    # ========== Launch Arguments ==========
    declare_camera_name = DeclareLaunchArgument(
        'camera_name', default_value='camera',
        description='Camera unique name'
    )
    
    declare_enable_web_viewer = DeclareLaunchArgument(
        'enable_web_viewer', default_value='true',
        description='Launch web visualization'
    )
    
    declare_voxel_size = DeclareLaunchArgument(
        'voxel_size', default_value='0.05',
        description='Voxel size in meters'
    )
    
    declare_max_range = DeclareLaunchArgument(
        'max_range', default_value='3.0',
        description='Maximum range for point cloud (meters)'
    )
    
    declare_decimation_filter = DeclareLaunchArgument(
        'decimation_filter', default_value='4',
        description='Decimation filter magnitude (1-8)'
    )
    
    declare_clip_distance = DeclareLaunchArgument(
        'clip_distance', default_value='3.0',
        description='Maximum distance for RealSense clipping (meters)'
    )
    
    # ========== RealSense Camera ==========
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(realsense_pkg_dir, 'launch', 'rs_launch.py')
        ]),
        launch_arguments={
            'camera_name': LaunchConfiguration('camera_name'),
            'enable_color': 'true',
            'enable_depth': 'true',
            'rgb_camera.color_profile': '640,480,30',
            'depth_module.depth_profile': '640,480,30',
            'pointcloud.enable': 'true',
            'pointcloud.stream_filter': '2',
            'pointcloud.ordered_pc': 'false',
            'align_depth.enable': 'true',
            'decimation_filter.enable': 'true',
            'decimation_filter.filter_magnitude': LaunchConfiguration('decimation_filter'),
            'spatial_filter.enable': 'true',
            'temporal_filter.enable': 'true',
            'clip_distance': LaunchConfiguration('clip_distance'),
            'publish_tf': 'true',
            'tf_publish_rate': '10.0',
        }.items()
    )
    
    # ========== 轻量级体素化节点 ==========
    voxelizer_node = Node(
        package='octomap_web_viewer',
        executable='pointcloud_voxelizer',
        name='pointcloud_voxelizer',
        output='screen',
        parameters=[{
            'input_cloud_topic': '/camera/camera/depth/color/points',
            'output_marker_topic': '/occupied_cells_vis_array',
            'voxel_size': LaunchConfiguration('voxel_size'),
            'frame_id': 'camera_depth_optical_frame',
            'max_range': LaunchConfiguration('max_range'),
            'min_z': -3.0,
            'max_z': 3.0,
            'publish_rate': 10.0,  # 最大 10Hz
            'point_skip': 4,  # 下采样 1/4
        }]
    )
    
    # ========== Rosbridge WebSocket ==========
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{'port': 9090, 'address': ''}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_web_viewer'))
    )
    
    # ========== Web Server ==========
    web_server = Node(
        package='octomap_web_viewer',
        executable='start_web_server',
        name='web_server',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_web_viewer'))
    )
    
    return LaunchDescription([
        # Arguments
        declare_camera_name,
        declare_enable_web_viewer,
        declare_voxel_size,
        declare_max_range,
        declare_decimation_filter,
        declare_clip_distance,
        
        # Nodes
        realsense_node,
        voxelizer_node,
        rosbridge_node,
        web_server,
    ])

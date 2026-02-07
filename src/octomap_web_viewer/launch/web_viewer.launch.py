"""
OctoMap Web Viewer Launch File
Launches rosbridge_server and HTTP server for web visualization.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('octomap_web_viewer')
    
    # Rosbridge WebSocket server
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'port': 9090,
            'address': '',
            'retry_startup_delay': 5.0,
        }],
        output='screen',
    )
    
    # HTTP server for web files
    # pkg_dir is .../share/octomap_web_viewer
    # script is in .../lib/octomap_web_viewer/start_web_server.py
    web_server = ExecuteProcess(
        cmd=['python3', os.path.join(pkg_dir, '..', '..', 'lib', 'octomap_web_viewer', 'start_web_server.py')],
        output='screen',
    )
    
    return LaunchDescription([
        rosbridge_node,
        web_server,
    ])

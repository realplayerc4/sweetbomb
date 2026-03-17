from typing import Optional

from app.services.rs_manager import RealSenseManager
from app.services.webrtc_manager import WebRTCManager
from app.services.task_manager import TaskManager
from app.services.robot_tcp_server import RobotTCPServer
from app.services.socketio import sio

# Singleton instances
_realsense_manager = None
_webrtc_manager = None
_task_manager = None
_robot_tcp_server: Optional[RobotTCPServer] = None

def get_realsense_manager() -> RealSenseManager:
    global _realsense_manager
    if _realsense_manager is None:
        _realsense_manager = RealSenseManager(sio)
    return _realsense_manager

def get_webrtc_manager() -> WebRTCManager:
    global _webrtc_manager
    if _webrtc_manager is None:
        _webrtc_manager = WebRTCManager(get_realsense_manager())
    return _webrtc_manager

def get_task_manager() -> TaskManager:
    """Get the singleton TaskManager instance."""
    global _task_manager
    if _task_manager is None:
        _task_manager = TaskManager.get_instance(sio, get_realsense_manager())
    return _task_manager


def get_robot_tcp_server() -> Optional[RobotTCPServer]:
    """获取机器人 TCP 服务器实例。"""
    global _robot_tcp_server
    return _robot_tcp_server


def set_robot_tcp_server(server: Optional[RobotTCPServer]) -> None:
    """设置机器人 TCP 服务器实例（在 main.py 启动时调用）。"""
    global _robot_tcp_server
    _robot_tcp_server = server

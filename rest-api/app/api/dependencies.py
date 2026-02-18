from fastapi.security import OAuth2PasswordBearer


from app.services.rs_manager import RealSenseManager
from app.services.webrtc_manager import WebRTCManager
from app.services.task_manager import TaskManager
from app.services.socketio import sio

# OAuth2 setup
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")

# Singleton instances
_realsense_manager = None
_webrtc_manager = None
_task_manager = None

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
        _task_manager = TaskManager.get_instance(sio)
    return _task_manager
from app.services.rs_manager import RealSenseManager
from app.services.webrtc_manager import WebRTCManager
from app.services.task_manager import TaskManager
from app.services.socketio import sio

# Try to import GPU-accelerated WebRTC manager
try:
    from app.services.webrtc_gpu_manager import (
        NvidiaWebRTCManager,
        create_webrtc_manager,
        CUDA_AVAILABLE
    )
    GPU_WEBRTC_AVAILABLE = True
except ImportError:
    GPU_WEBRTC_AVAILABLE = False
    CUDA_AVAILABLE = False

# Singleton instances
_realsense_manager = None
_webrtc_manager = None
_task_manager = None

def get_realsense_manager() -> RealSenseManager:
    global _realsense_manager
    if _realsense_manager is None:
        _realsense_manager = RealSenseManager(sio)
    return _realsense_manager

def get_webrtc_manager():
    """Get WebRTC manager with GPU acceleration if available.

    Returns:
        NvidiaWebRTCManager if GPU is available, otherwise WebRTCManager
    """
    global _webrtc_manager
    if _webrtc_manager is None:
        realsense_manager = get_realsense_manager()

        if GPU_WEBRTC_AVAILABLE:
            try:
                _webrtc_manager = create_webrtc_manager(
                    realsense_manager,
                    use_gpu=True,
                    target_resolution=(1280, 720)
                )
                print(f"[WebRTC] GPU-accelerated manager initialized (CUDA available: {CUDA_AVAILABLE})")
            except Exception as e:
                print(f"[WebRTC] Failed to initialize GPU manager: {e}, falling back to CPU")
                _webrtc_manager = WebRTCManager(realsense_manager)
        else:
            _webrtc_manager = WebRTCManager(realsense_manager)
            print("[WebRTC] CPU-based manager initialized (GPU acceleration not available)")

    return _webrtc_manager

def get_task_manager() -> TaskManager:
    """Get the singleton TaskManager instance."""
    global _task_manager
    if _task_manager is None:
        _task_manager = TaskManager.get_instance(sio, get_realsense_manager())
    return _task_manager

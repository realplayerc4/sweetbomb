"""OAK 设备数据模型。"""

from typing import Optional, Dict, Any
from dataclasses import dataclass, field
from enum import Enum

import depthai as dai


class OAKDeviceState(Enum):
    """OAK 设备状态。"""
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    STREAMING = "streaming"
    ERROR = "error"


@dataclass
class OAKDeviceInfo:
    """OAK 设备信息。"""
    device_id: str  # depthai 3.x 使用 deviceId
    name: str  # 设备名称（如 OAK-D, OAK-D-Lite）
    state: OAKDeviceState = OAKDeviceState.DISCONNECTED
    protocol: Optional[dai.XLinkProtocol] = None
    platform: Optional[str] = None
    # 相机信息
    color_camera: bool = True
    left_camera: bool = True
    right_camera: bool = True
    # 连接信息
    last_connected: Optional[float] = None
    connection_error: Optional[str] = None
    # 元数据
    custom_data: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict:
        """转换为字典。"""
        return {
            "device_id": self.device_id,
            "name": self.name,
            "state": self.state.value,
            "protocol": self.protocol.name if self.protocol else None,
            "platform": self.platform,
            "color_camera": self.color_camera,
            "left_camera": self.left_camera,
            "right_camera": self.right_camera,
            "last_connected": self.last_connected,
            "connection_error": self.connection_error,
        }

"""点云处理模块。

从 rs_manager 拆分而来，负责点云功能的激活/停用与状态查询。
"""

from typing import Dict

import pyrealsense2 as rs

from app.core.errors import RealSenseError
from app.models.stream import PointCloudStatus


class PointCloudProcessor:
    """管理 RealSense 点云处理的启停与状态。

    点云的实际计算在 StreamController._extract_raw_frames 中完成，
    本模块仅控制开关和查询状态。
    """

    def __init__(
        self,
        devices: Dict[str, rs.device],
        is_pointcloud_enabled: Dict[str, bool],
        refresh_fn,
    ):
        self.devices = devices
        self.is_pointcloud_enabled = is_pointcloud_enabled
        self._refresh_devices = refresh_fn

    def _ensure_device(self, device_id: str) -> None:
        """确保设备存在，若不在则尝试刷新。"""
        if device_id not in self.devices:
            self._refresh_devices()
            if device_id not in self.devices:
                raise RealSenseError(
                    status_code=404, detail=f"Device {device_id} not found"
                )

    def activate(self, device_id: str, enable: bool) -> PointCloudStatus:
        """激活或停用设备的点云处理。"""
        self._ensure_device(device_id)
        self.is_pointcloud_enabled[device_id] = enable
        return PointCloudStatus(device_id=device_id, is_active=enable)

    def get_status(self, device_id: str) -> PointCloudStatus:
        """获取设备的点云处理状态。"""
        self._ensure_device(device_id)
        return PointCloudStatus(
            device_id=device_id,
            is_active=self.is_pointcloud_enabled.get(device_id, False),
        )

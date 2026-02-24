"""设备发现与枚举模块。

从 rs_manager 拆分而来，负责 RealSense 设备的发现、枚举与基本信息管理。
"""

import threading
from typing import Dict, List

import pyrealsense2 as rs

from app.core.errors import RealSenseError
from app.models.device import DeviceInfo


class DeviceDiscovery:
    """管理 RealSense 设备的发现与枚举。

    通过 pyrealsense2 context 监控设备连接/断开事件，
    维护设备运行状态列表。
    """

    def __init__(self, ctx: rs.context, lock: threading.Lock):
        # 共享 context 和锁，因为其他模块也需要访问设备列表
        self.ctx = ctx
        self.lock = lock
        self.devices: Dict[str, rs.device] = {}
        self.device_infos: Dict[str, DeviceInfo] = {}
        # 由外部（Facade）注入的流水线引用，用于判断设备是否正在流式传输
        self.pipelines: Dict[str, rs.pipeline] = {}

    def refresh_devices(self) -> List[DeviceInfo]:
        """刷新并返回已连接的设备列表。"""
        with self.lock:
            # 清除已断开且未在流式传输的设备
            for device_id in list(self.devices.keys()):
                if device_id not in self.pipelines:
                    del self.devices[device_id]
                    if device_id in self.device_infos:
                        del self.device_infos[device_id]

            # 发现新连接的设备
            for dev in self.ctx.devices:
                device_id = dev.get_info(rs.camera_info.serial_number)

                if device_id in self.devices:
                    continue

                self.devices[device_id] = dev
                device_info = self._extract_device_info(dev, device_id)
                self.device_infos[device_id] = device_info

            return list(self.device_infos.values())

    def get_devices(self) -> List[DeviceInfo]:
        """获取所有已连接设备（内部调用 refresh）。"""
        return self.refresh_devices()

    def get_device(self, device_id: str) -> DeviceInfo:
        """根据 ID 获取特定设备信息。"""
        devices = self.get_devices()
        for device in devices:
            if device.device_id == device_id:
                return device
        raise RealSenseError(status_code=404, detail=f"Device {device_id} not found")

    def reset_device(self, device_id: str) -> bool:
        """硬件重置指定设备。"""
        self._ensure_device_exists(device_id)

        dev = self.devices[device_id]
        try:
            dev.hardware_reset()
            return True
        except RuntimeError as e:
            raise RealSenseError(
                status_code=500, detail=f"Failed to reset device: {str(e)}"
            )

    def _ensure_device_exists(self, device_id: str) -> None:
        """确保设备存在，不存在则尝试刷新后再检查。"""
        if device_id not in self.devices:
            self.refresh_devices()
            if device_id not in self.devices:
                raise RealSenseError(
                    status_code=404, detail=f"Device {device_id} not found"
                )

    def _extract_device_info(self, dev: rs.device, device_id: str) -> DeviceInfo:
        """从 pyrealsense2 设备对象中提取设备信息。"""
        def _safe_get(info_type, default=None):
            try:
                return dev.get_info(info_type)
            except RuntimeError:
                return default

        name = _safe_get(rs.camera_info.name, "Unknown Device")
        firmware_version = _safe_get(rs.camera_info.firmware_version)
        physical_port = _safe_get(rs.camera_info.physical_port)
        usb_type = _safe_get(rs.camera_info.usb_type_descriptor)
        product_id = _safe_get(rs.camera_info.product_id)

        sensors = []
        for sensor in dev.sensors:
            try:
                sensors.append(sensor.get_info(rs.camera_info.name))
            except RuntimeError:
                pass

        return DeviceInfo(
            device_id=device_id,
            name=name,
            serial_number=device_id,
            firmware_version=firmware_version,
            physical_port=physical_port,
            usb_type=usb_type,
            product_id=product_id,
            sensors=sensors,
            is_streaming=device_id in self.pipelines,
        )

"""OAK 设备发现与枚举模块。

负责 OAK 设备的发现、枚举与基本信息管理。
适配 depthai 3.x API。
"""

import threading
import time
from typing import Dict, List, Optional, Callable
from dataclasses import dataclass, field
from enum import Enum

import depthai as dai

from app.core.oak_errors import OAKDeviceError, OAKConnectionError


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
    custom_data: Dict[str, any] = field(default_factory=dict)

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


class OAKDeviceDiscovery:
    """OAK 设备发现管理器。

    负责：
    - 扫描和发现 OAK 设备
    - 维护设备连接状态
    - 监控设备连接/断开事件
    - 提供设备信息查询
    """

    def __init__(self):
        self._devices: Dict[str, OAKDeviceInfo] = {}
        self._connected_devices: Dict[str, dai.Device] = {}
        self._lock = threading.RLock()
        self._callbacks: List[Callable[[str, OAKDeviceState, OAKDeviceState], None]] = []
        self._monitor_thread: Optional[threading.Thread] = None
        self._stop_monitoring = threading.Event()

    # ==================== 公共 API ====================

    def start(self) -> None:
        """启动设备发现服务。"""
        with self._lock:
            if self._monitor_thread is not None and self._monitor_thread.is_alive():
                return

            self._stop_monitoring.clear()
            self._monitor_thread = threading.Thread(
                target=self._monitor_loop,
                name="OAKDeviceMonitor",
                daemon=True
            )
            self._monitor_thread.start()
            print(f"[OAKDeviceDiscovery] 设备监控线程已启动")

    def stop(self) -> None:
        """停止设备发现服务。"""
        with self._lock:
            self._stop_monitoring.set()

            # 断开所有连接的设备
            for device_id, device in list(self._connected_devices.items()):
                try:
                    device.close()
                    print(f"[OAKDeviceDiscovery] 已断开设备: {device_id}")
                except Exception as e:
                    print(f"[OAKDeviceDiscovery] 断开设备 {device_id} 时出错: {e}")

            self._connected_devices.clear()

            # 等待监控线程结束
            if self._monitor_thread is not None and self._monitor_thread.is_alive():
                self._monitor_thread.join(timeout=2.0)
                if self._monitor_thread.is_alive():
                    print("[OAKDeviceDiscovery] 警告: 监控线程未能及时停止")

            print("[OAKDeviceDiscovery] 设备发现服务已停止")

    def scan_devices(self) -> List[OAKDeviceInfo]:
        """扫描并返回所有可用的 OAK 设备。

        Returns:
            设备信息列表
        """
        device_infos = []

        try:
            # depthai 3.x: 使用 getAllAvailableDevices()
            available_devices = dai.Device.getAllAvailableDevices()

            for device_desc in available_devices:
                # depthai 3.x: 从 XLinkDeviceDesc 获取信息
                device_id = device_desc.getXLinkDeviceDesc()
                device_name = device_desc.name or "OAK Device"

                oak_info = OAKDeviceInfo(
                    device_id=device_id,
                    name=device_name,
                    state=OAKDeviceState.CONNECTED if self._is_device_connected(device_id) else OAKDeviceState.DISCONNECTED,
                    protocol=device_desc.protocol,
                    platform=device_desc.platform.tag if device_desc.platform else None
                )

                device_infos.append(oak_info)

                # 更新内部设备列表
                with self._lock:
                    self._devices[device_id] = oak_info

        except Exception as e:
            print(f"[OAKDeviceDiscovery] 扫描设备时出错: {e}")

        return device_infos

    def get_device(self, device_id: str) -> Optional[OAKDeviceInfo]:
        """获取指定设备的详细信息。

        Args:
            device_id: 设备 ID

        Returns:
            设备信息，不存在则返回 None
        """
        with self._lock:
            if device_id in self._devices:
                return self._devices[device_id]

        # 如果没有缓存，尝试扫描
        devices = self.scan_devices()
        for device in devices:
            if device.device_id == device_id:
                return device

        return None

    def connect_device(self, device_id: str) -> dai.Device:
        """连接到指定设备。

        Args:
            device_id: 设备 ID

        Returns:
            depthai Device 对象

        Raises:
            OAKDeviceError: 设备未找到
            OAKConnectionError: 连接失败
        """
        with self._lock:
            # 检查是否已连接
            if device_id in self._connected_devices:
                return self._connected_devices[device_id]

            # 更新设备状态
            if device_id in self._devices:
                self._update_device_state(device_id, OAKDeviceState.CONNECTING)

        try:
            # depthai 3.x: 查找并连接设备
            available_devices = dai.Device.getAllAvailableDevices()
            target_device = None

            for device_desc in available_devices:
                if device_desc.getXLinkDeviceDesc() == device_id:
                    target_device = device_desc
                    break

            if target_device is None:
                raise OAKDeviceError(f"设备未找到: {device_id}")

            # 连接设备 - depthai 3.x 直接使用构造函数
            print(f"[OAKDeviceDiscovery] 正在连接设备: {device_id}")
            device = dai.Device(target_device)

            # 保存连接
            with self._lock:
                self._connected_devices[device_id] = device
                self._update_device_state(device_id, OAKDeviceState.CONNECTED)

            print(f"[OAKDeviceDiscovery] 设备连接成功: {device_id}")
            return device

        except OAKDeviceError:
            with self._lock:
                self._update_device_state(device_id, OAKDeviceState.ERROR)
            raise
        except Exception as e:
            with self._lock:
                self._update_device_state(device_id, OAKDeviceState.ERROR)
            raise OAKConnectionError(f"连接设备失败 {device_id}: {e}")

    def disconnect_device(self, device_id: str) -> None:
        """断开指定设备。

        Args:
            device_id: 设备 ID
        """
        with self._lock:
            if device_id in self._connected_devices:
                try:
                    device = self._connected_devices[device_id]
                    device.close()
                    print(f"[OAKDeviceDiscovery] 设备已断开: {device_id}")
                except Exception as e:
                    print(f"[OAKDeviceDiscovery] 断开设备时出错 {device_id}: {e}")
                finally:
                    del self._connected_devices[device_id]
                    self._update_device_state(device_id, OAKDeviceState.DISCONNECTED)

    def register_state_callback(
        self,
        callback: Callable[[str, OAKDeviceState, OAKDeviceState], None]
    ) -> None:
        """注册设备状态变化回调。

        Args:
            callback: 回调函数，参数为 (device_id, old_state, new_state)
        """
        with self._lock:
            self._callbacks.append(callback)

    def unregister_state_callback(
        self,
        callback: Callable[[str, OAKDeviceState, OAKDeviceState], None]
    ) -> None:
        """注销设备状态变化回调。

        Args:
            callback: 要注销的回调函数
        """
        with self._lock:
            if callback in self._callbacks:
                self._callbacks.remove(callback)

    # ==================== 内部方法 ====================

    def _monitor_loop(self) -> None:
        """设备监控循环。"""
        print("[OAKDeviceDiscovery] 监控循环已启动")

        while not self._stop_monitoring.is_set():
            try:
                # 扫描设备
                self._check_device_changes()

                # 等待下一次扫描
                self._stop_monitoring.wait(timeout=2.0)

            except Exception as e:
                print(f"[OAKDeviceDiscovery] 监控循环出错: {e}")
                self._stop_monitoring.wait(timeout=2.0)

        print("[OAKDeviceDiscovery] 监控循环已停止")

    def _check_device_changes(self) -> None:
        """检查设备变化。"""
        try:
            # 获取当前可用设备
            available_devices = dai.Device.getAllAvailableDevices()
            current_ids = {d.getXLinkDeviceDesc() for d in available_devices}

            with self._lock:
                previous_ids = set(self._devices.keys())

                # 检测新连接的设备
                for device_id in current_ids - previous_ids:
                    device_desc = next((d for d in available_devices if d.getXLinkDeviceDesc() == device_id), None)
                    if device_desc:
                        oak_info = OAKDeviceInfo(
                            device_id=device_id,
                            name=device_desc.name or "OAK Device",
                            state=OAKDeviceState.DISCONNECTED
                        )
                        self._devices[device_id] = oak_info
                        self._update_device_state(device_id, OAKDeviceState.CONNECTED)
                        print(f"[OAKDeviceDiscovery] 检测到新设备: {device_id}")

                # 检测断开的设备
                for device_id in previous_ids - current_ids:
                    if device_id in self._connected_devices:
                        self.disconnect_device(device_id)
                    else:
                        self._update_device_state(device_id, OAKDeviceState.DISCONNECTED)
                        if device_id in self._devices:
                            del self._devices[device_id]
                    print(f"[OAKDeviceDiscovery] 设备断开: {device_id}")

        except Exception as e:
            print(f"[OAKDeviceDiscovery] 检查设备变化时出错: {e}")

    def _update_device_state(self, device_id: str, new_state: OAKDeviceState) -> None:
        """更新设备状态并触发回调。"""
        if device_id not in self._devices:
            return

        old_state = self._devices[device_id].state
        if old_state == new_state:
            return

        self._devices[device_id].state = new_state
        self._devices[device_id].last_connected = time.time() if new_state == OAKDeviceState.CONNECTED else None

        # 触发回调
        for callback in self._callbacks:
            try:
                callback(device_id, old_state, new_state)
            except Exception as e:
                print(f"[OAKDeviceDiscovery] 状态回调出错: {e}")

    def _is_device_connected(self, device_id: str) -> bool:
        """检查设备是否已连接。"""
        return device_id in self._connected_devices

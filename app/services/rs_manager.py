"""RealSense 管理器门面模块。

作为原 rs_manager 的 Facade，将请求委托给四个职责单一的子模块：
- DeviceDiscovery: 设备发现与枚举
- SensorControl: 传感器参数读写
- StreamController: 流启停与帧采集
- PointCloudProcessor: 点云处理
"""

import threading
from typing import Any, Dict, List, Optional
from datetime import datetime

import numpy as np
import pyrealsense2 as rs
import socketio

from app.core.errors import RealSenseError
from app.models.device import DeviceInfo
from app.models.option import OptionInfo
from app.models.sensor import SensorInfo
from app.models.stream import PointCloudStatus, StreamConfig, StreamStatus

from app.services.device_discovery import DeviceDiscovery
from app.services.sensor_control import SensorControl
from app.services.stream_controller import StreamController
from app.services.point_cloud_processor import PointCloudProcessor
from app.services.metadata_socket_server import MetadataSocketServer
from app.services.point_cloud_analyzer import PointCloudAnalysisResult


class RealSenseManager:
    """RealSense 管理器门面。

    对外保持与旧版完全一致的公开 API，内部将职责委托给
    各个子模块。所有调用方（路由、依赖注入等）无需任何改动。
    """

    def __init__(self, sio: socketio.AsyncServer):
        self.ctx = rs.context()
        self.lock = threading.Lock()

        # 初始化滤波器（由 StreamController 共享）
        decimation_filter = rs.decimation_filter()
        decimation_filter.set_option(rs.option.filter_magnitude, 3)
        spatial_filter = rs.spatial_filter()
        temporal_filter = rs.temporal_filter()
        colorizer = rs.colorizer()
        colorizer.set_option(rs.option.color_scheme, 0)
        colorizer.set_option(rs.option.histogram_equalization_enabled, 1)
        colorizer.set_option(rs.option.min_distance, 1.0)
        colorizer.set_option(rs.option.max_distance, 6.0)
        threshold_filter = rs.threshold_filter()
        threshold_filter.set_option(rs.option.min_distance, 1.0)
        threshold_filter.set_option(rs.option.max_distance, 6.0)

        self.filters = {
            "decimation": decimation_filter,
            "spatial": spatial_filter,
            "temporal": temporal_filter,
            "colorizer": colorizer,
            "threshold": threshold_filter,
        }
        self.pc = rs.pointcloud()

        # 子模块：设备发现
        self._discovery = DeviceDiscovery(self.ctx, self.lock)

        # 子模块：元数据服务
        self.metadata_socket_server = MetadataSocketServer(sio, self)

        # 子模块：流控制（需要共享设备列表、设备信息字典、滤波器）
        self._stream = StreamController(
            ctx=self.ctx,
            lock=self.lock,
            devices=self._discovery.devices,
            device_infos=self._discovery.device_infos,
            filters=self.filters,
            point_cloud_ref=self.pc,
            metadata_socket_server=self.metadata_socket_server,
            analysis_result_callback=self._on_analysis_result,
        )

        # 让 DeviceDiscovery 能感知到 pipelines 的存在
        # 这样在刷新设备时不会误删正在流式传输的设备
        self._discovery.pipelines = self._stream.pipelines

        # 子模块：传感器控制
        self._sensor = SensorControl(
            devices=self._discovery.devices,
            lock=self.lock,
            refresh_fn=self._discovery.refresh_devices,
        )

        # 子模块：点云处理
        self._pointcloud = PointCloudProcessor(
            devices=self._discovery.devices,
            is_pointcloud_enabled=self._stream.is_pointcloud_enabled,
            refresh_fn=self._discovery.refresh_devices,
        )

        # 初始化点云分析结果存储
        self._latest_analysis_results: Dict[str, PointCloudAnalysisResult] = {}
        self._latest_move_distances: Dict[str, float] = {}
        self._analysis_timestamps: Dict[str, datetime] = {}

        # 启动自动流管理后台线程
        self._auto_stream_enabled = True
        self._auto_stream_thread = threading.Thread(target=self._auto_stream_manager, daemon=True)
        self._auto_stream_thread.start()

        # 初始化设备列表（启动流）
        self.refresh_devices()

    def _auto_stream_manager(self) -> None:
        """后台线程：自动管理设备流的启动和停止。"""
        import time
        while self._auto_stream_enabled:
            try:
                for device_id in list(self._discovery.devices.keys()):
                    # 如果设备没有在流传输，自动启动
                    if device_id not in self._stream.pipelines:
                        try:
                            print(f"[AutoStream] Auto-starting stream for {device_id}")
                            self._auto_start_device_stream(device_id)
                        except Exception as e:
                            print(f"[AutoStream] Failed to start stream for {device_id}: {e}")
            except Exception as e:
                print(f"[AutoStream] Error in auto stream manager: {e}")
            time.sleep(5)  # 每5秒检查一次

    def _auto_start_device_stream(self, device_id: str) -> None:
        """自动启动设备的 depth 流并激活点云处理。"""
        from app.models.stream import StreamConfig

        # 创建 depth 流配置
        depth_config = StreamConfig(
            sensor_id=f"{device_id}-sensor-0",
            stream_type="depth",
            format="z16",
            resolution={"width": 1280, "height": 720},
            framerate=15,
            enable=True
        )

        # 启动流
        self._stream.start_stream(device_id, [depth_config], align_to="color")

        # 激活点云处理
        self._pointcloud.activate(device_id, True)

        print(f"[AutoStart] Stream and point cloud activated for {device_id}")

    def _on_analysis_result(self, device_id: str, result: PointCloudAnalysisResult) -> None:
        """回调函数：保存点云分析结果。"""
        approach_offset = 0.05
        self._latest_analysis_results[device_id] = result
        self._analysis_timestamps[device_id] = datetime.now()

        if result.material_distance is not None:
            move_distance = result.material_distance - approach_offset
            if move_distance <= 0:
                move_distance = 0.1
            self._latest_move_distances[device_id] = move_distance
        else:
            self._latest_move_distances[device_id] = 0.0

    def get_analysis_result(self, device_id: str) -> Optional[PointCloudAnalysisResult]:
        return self._latest_analysis_results.get(device_id)

    def get_move_distance(self, device_id: str) -> Optional[float]:
        return self._latest_move_distances.get(device_id)

    def get_analysis_timestamp(self, device_id: str) -> Optional[datetime]:
        return self._analysis_timestamps.get(device_id)

    # ========== 设备管理 API ==========

    def refresh_devices(self) -> List[DeviceInfo]:
        return self._discovery.refresh_devices()

    def get_devices(self) -> List[DeviceInfo]:
        return self._discovery.get_devices()

    def get_device(self, device_id: str) -> DeviceInfo:
        return self._discovery.get_device(device_id)

    def reset_device(self, device_id: str) -> bool:
        return self._discovery.reset_device(device_id)

    # ========== 传感器管理 API ==========

    def get_sensors(self, device_id: str) -> List[SensorInfo]:
        return self._sensor.get_sensors(device_id)

    def get_sensor(self, device_id: str, sensor_id: str) -> SensorInfo:
        return self._sensor.get_sensor(device_id, sensor_id)

    def get_sensor_options(self, device_id: str, sensor_id: str) -> List[OptionInfo]:
        return self._sensor.get_sensor_options(device_id, sensor_id)

    def get_sensor_option(
        self, device_id: str, sensor_id: str, option_id: str
    ) -> OptionInfo:
        return self._sensor.get_sensor_option(device_id, sensor_id, option_id)

    def set_sensor_option(
        self, device_id: str, sensor_id: str, option_id: str, value: Any
    ) -> bool:
        return self._sensor.set_sensor_option(device_id, sensor_id, option_id, value)

    # ========== 流管理 API ==========

    def start_stream(
        self,
        device_id: str,
        configs: List[StreamConfig],
        align_to: Optional[str] = None,
    ) -> StreamStatus:
        # 在启动流之前先刷新设备列表，确保设备存在
        self._discovery.refresh_devices()
        return self._stream.start_stream(device_id, configs, align_to)

    def stop_stream(self, device_id: str) -> StreamStatus:
        return self._stream.stop_stream(device_id)

    def get_stream_status(self, device_id: str) -> StreamStatus:
        self._discovery._ensure_device_exists(device_id)
        return self._stream.get_stream_status(device_id)

    def get_latest_frame(
        self, device_id: str, stream_type: str
    ) -> np.ndarray:
        return self._stream.get_latest_frame(device_id, stream_type)

    def get_latest_metadata(self, device_id: str, stream_type: str) -> Dict:
        return self._stream.get_latest_metadata(device_id, stream_type)

    def check_all_streams_health(self):
        """遍历所有正在进行的流并进行健康检查。"""
        active_device_ids = list(self.pipelines.keys())
        for device_id in active_device_ids:
            self._stream.check_stream_health(device_id)

    # ========== 点云 API ==========

    def activate_point_cloud(self, device_id: str, enable: bool) -> PointCloudStatus:
        print(f"[RealSenseManager] activate_point_cloud: device_id={device_id}, enable={enable}")
        return self._pointcloud.activate(device_id, enable)

    def get_point_cloud_status(self, device_id: str) -> PointCloudStatus:
        return self._pointcloud.get_status(device_id)

    # ========== 兼容性属性 ==========
    # 某些外部模块（如 endpoints、webrtc_manager）可能直接访问属性

    @property
    def devices(self) -> Dict[str, rs.device]:
        return self._discovery.devices

    @property
    def device_infos(self) -> Dict[str, DeviceInfo]:
        return self._discovery.device_infos

    @property
    def pipelines(self):
        return self._stream.pipelines

    @property
    def active_streams(self):
        return self._stream.active_streams

    @property
    def frame_queues(self):
        return self._stream.frame_queues

    @property
    def metadata_queues(self):
        return self._stream.metadata_queues

    @property
    def is_pointcloud_enabled(self):
        return self._stream.is_pointcloud_enabled

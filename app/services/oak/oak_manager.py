"""OAK 管理器门面类。

提供简化的接口来管理 OAK 设备、流和配置。
适配 depthai 3.x API。
"""

import logging
from typing import Dict, List, Optional, Callable, Any
import threading
import time

import depthai as dai
import numpy as np


logger = logging.getLogger(__name__)


class OAKManager:
    """OAK 管理器门面 - 简化接口。"""

    def __init__(self):
        self._device: Optional[dai.Device] = None
        self._pipeline: Optional[dai.Pipeline] = None
        self._device_info: Optional[dai.DeviceInfo] = None
        self._is_streaming = False
        self._lock = threading.RLock()
        self._frame_callbacks: List[Callable[[np.ndarray, str], None]] = []
        self._output_queues: Dict[str, dai.Queue] = {}
        self._capture_thread: Optional[threading.Thread] = None
        self._stop_capture = threading.Event()

    # ==================== 设备管理 ====================

    def initialize(self, device_id: Optional[str] = None) -> bool:
        """初始化 OAK 设备。

        Args:
            device_id: 可选的设备 ID，None 则使用第一个可用设备

        Returns:
            是否成功初始化
        """
        try:
            with self._lock:
                # 获取可用设备
                available = dai.Device.getAllAvailableDevices()
                if not available:
                    logger.error("[OAKManager] 未找到 OAK 设备")
                    return False

                # 选择设备
                if device_id:
                    target = next((d for d in available if d.getDeviceId() == device_id), None)
                    if not target:
                        logger.error(f"[OAKManager] 未找到指定设备: {device_id}")
                        return False
                else:
                    target = available[0]

                self._device_info = target

                # 创建设备
                logger.info(f"[OAKManager] 正在连接设备: {target.name} ({target.getDeviceId()})")
                self._device = dai.Device(target)

                logger.info("[OAKManager] 设备连接成功")
                return True

        except Exception as e:
            logger.error(f"[OAKManager] 初始化失败: {e}")
            return False

    def shutdown(self) -> None:
        """关闭 OAK 设备。"""
        with self._lock:
            self.stop_stream()

            if self._device:
                try:
                    self._device.close()
                    logger.info("[OAKManager] 设备已关闭")
                except Exception as e:
                    logger.error(f"[OAKManager] 关闭设备时出错: {e}")
                finally:
                    self._device = None
                    self._device_info = None

    def is_initialized(self) -> bool:
        """检查是否已初始化。"""
        with self._lock:
            return self._device is not None

    def get_device_info(self) -> Optional[Dict[str, Any]]:
        """获取设备信息。"""
        with self._lock:
            if not self._device_info:
                return None

            return {
                "name": self._device_info.name,
                "device_id": self._device_info.getDeviceId(),
                "protocol": str(self._device_info.protocol),
                "platform": str(self._device_info.platform),
                "state": str(self._device_info.state),
            }

    # ==================== 流管理 ====================

    def start_stream(self, enable_depth: bool = True, enable_color: bool = True) -> bool:
        """启动视频流。

        Args:
            enable_depth: 是否启用深度流
            enable_color: 是否启用彩色流

        Returns:
            是否成功启动
        """
        with self._lock:
            if not self._device:
                logger.error("[OAKManager] 设备未初始化")
                return False

            if self._is_streaming:
                logger.warning("[OAKManager] 流已在运行")
                return True

            try:
                # 创建管道
                self._pipeline = dai.Pipeline()

                # 创建相机节点
                if enable_color:
                    self._create_color_camera()

                if enable_depth:
                    self._create_depth_cameras()

                # 启动管道
                self._device.startPipeline(self._pipeline)

                # 设置输出队列
                self._setup_queues(enable_color, enable_depth)

                # 启动捕获线程
                self._stop_capture.clear()
                self._capture_thread = threading.Thread(
                    target=self._capture_loop,
                    daemon=True
                )
                self._capture_thread.start()

                self._is_streaming = True
                logger.info("[OAKManager] 流已启动")
                return True

            except Exception as e:
                logger.error(f"[OAKManager] 启动流失败: {e}")
                return False

    def stop_stream(self) -> None:
        """停止视频流。"""
        with self._lock:
            if not self._is_streaming:
                return

            logger.info("[OAKManager] 停止流...")

            # 停止捕获线程
            self._stop_capture.set()
            if self._capture_thread:
                self._capture_thread.join(timeout=2.0)

            # 清理队列
            self._output_queues.clear()

            self._is_streaming = False
            self._pipeline = None

            logger.info("[OAKManager] 流已停止")

    def is_streaming(self) -> bool:
        """检查是否正在流式传输。"""
        with self._lock:
            return self._is_streaming

    def register_frame_callback(self, callback: Callable[[np.ndarray, str], None]) -> None:
        """注册帧回调。

        Args:
            callback: 回调函数，接收 (frame, stream_name)
        """
        with self._lock:
            if callback not in self._frame_callbacks:
                self._frame_callbacks.append(callback)

    def unregister_frame_callback(self, callback: Callable[[np.ndarray, str], None]) -> None:
        """注销帧回调。"""
        with self._lock:
            if callback in self._frame_callbacks:
                self._frame_callbacks.remove(callback)

    # ==================== 内部方法 ====================

    def _create_color_camera(self) -> None:
        """创建彩色相机节点。"""
        cam_rgb = self._pipeline.createColorCamera()
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setFps(30)

        # 创建输出
        xout_rgb = self._pipeline.createXLinkOut()
        xout_rgb.setStreamName("color")
        cam_rgb.video.link(xout_rgb.input)

    def _create_depth_cameras(self) -> None:
        """创建深度相机节点。"""
        # 单目相机
        mono_left = self._pipeline.createMonoCamera()
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)

        mono_right = self._pipeline.createMonoCamera()
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)

        # 立体深度
        stereo = self._pipeline.createStereoDepth()
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)

        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        # 深度输出
        xout_depth = self._pipeline.createXLinkOut()
        xout_depth.setStreamName("depth")
        stereo.depth.link(xout_depth.input)

    def _setup_queues(self, enable_color: bool, enable_depth: bool) -> None:
        """设置输出队列。"""
        if enable_color:
            self._output_queues["color"] = self._device.getOutputQueue(
                name="color", maxSize=4, blocking=False
            )

        if enable_depth:
            self._output_queues["depth"] = self._device.getOutputQueue(
                name="depth", maxSize=4, blocking=False
            )

    def _capture_loop(self) -> None:
        """帧捕获循环。"""
        logger.info("[OAKManager] 捕获循环已启动")

        while not self._stop_capture.is_set():
            try:
                for name, queue in self._output_queues.items():
                    img_frame = queue.tryGet()
                    if img_frame is not None:
                        frame = img_frame.getCvFrame()

                        # 分发到回调
                        for callback in self._frame_callbacks:
                            try:
                                callback(frame, name)
                            except Exception as e:
                                logger.error(f"[OAKManager] 帧回调出错: {e}")

            except Exception as e:
                logger.error(f"[OAKManager] 捕获循环出错: {e}")

        logger.info("[OAKManager] 捕获循环已停止")


# 全局实例
oak_manager = OAKManager()

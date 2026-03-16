"""OAK 流控制模块。

负责视频流的启动、停止、帧捕获和队列管理。
"""

import threading
import queue
import time
from typing import Dict, Optional, Callable, List, Tuple
from dataclasses import dataclass
from enum import Enum
import logging

import depthai as dai
import numpy as np

from app.core.oak_errors import OAKStreamError, OAKDeviceError
from app.models.oak.config import (
    OAKPipelineConfig, OAKCameraConfig, OAKStreamConfig,
    OAKCameraType, OAKFrameType, OAKResolution
)
from app.models.oak.frame import OAKFrame


logger = logging.getLogger(__name__)


class StreamState(Enum):
    """流状态。"""
    STOPPED = "stopped"
    STARTING = "starting"
    RUNNING = "running"
    STOPPING = "stopping"
    ERROR = "error"


@dataclass
class StreamStats:
    """流统计信息。"""
    fps: float = 0.0
    frame_count: int = 0
    dropped_frames: int = 0
    latency_ms: float = 0.0
    start_time: Optional[float] = None
    last_frame_time: Optional[float] = None


class OAKStreamController:
    """OAK 流控制器。

    负责管理视频流的生命周期：
    - 构建和配置 depthai Pipeline
    - 启动/停止视频流
    - 管理帧队列和回调
    - 流统计和监控
    """

    def __init__(self):
        self._device: Optional[dai.Device] = None
        self._pipeline: Optional[dai.Pipeline] = None
        self._config: Optional[OAKPipelineConfig] = None
        self._state = StreamState.STOPPED
        self._lock = threading.RLock()
        self._frame_queues: Dict[str, queue.Queue] = {}
        self._output_queues: Dict[str, dai.Node.Output] = {}
        self._camera_nodes: Dict[str, dai.Node] = {}
        self._frame_callbacks: List[Callable[[OAKFrame], None]] = []
        self._error_callbacks: List[Callable[[Exception], None]] = []
        self._capture_thread: Optional[threading.Thread] = None
        self._stop_capture = threading.Event()
        self._stats = StreamStats()

    # ==================== 公共 API ====================

    def initialize(self, device: dai.Device) -> None:
        """初始化流控制器。

        Args:
            device: 已连接的 depthai Device
        """
        with self._lock:
            self._device = device

    def start_stream(self, config: OAKPipelineConfig) -> bool:
        """启动视频流。

        Args:
            config: 管道配置

        Returns:
            是否成功启动

        Raises:
            OAKStreamError: 启动失败
        """
        with self._lock:
            if self._state not in [StreamState.STOPPED, StreamState.ERROR]:
                logger.warning(f"[OAKStreamController] 无法启动，当前状态: {self._state}")
                return False

            self._state = StreamState.STARTING
            self._config = config

        try:
            # 验证配置
            errors = config.validate()
            if errors:
                raise OAKStreamError(f"配置验证失败: {', '.join(errors)}")

            # 构建立管道
            self._build_pipeline()

            # 启动管道
            with self._lock:
                if self._device is None:
                    raise OAKStreamError("设备未初始化")

                self._device.startPipeline(self._pipeline)

                # 获取输出队列
                self._setup_output_queues()

                # 启动捕获线程
                self._stop_capture.clear()
                self._capture_thread = threading.Thread(
                    target=self._capture_loop,
                    name="OAKFrameCapture",
                    daemon=True
                )
                self._capture_thread.start()

                # 更新状态
                self._state = StreamState.RUNNING
                self._stats.start_time = time.time()

                logger.info(f"[OAKStreamController] 流已启动，配置: {config.to_dict()}")

            return True

        except Exception as e:
            with self._lock:
                self._state = StreamState.ERROR
            self._notify_error(e)
            raise OAKStreamError(f"启动流失败: {e}")

    def stop_stream(self) -> None:
        """停止视频流。"""
        with self._lock:
            if self._state not in [StreamState.RUNNING, StreamState.ERROR]:
                return

            self._state = StreamState.STOPPING
            logger.info("[OAKStreamController] 正在停止流...")

        # 停止捕获线程
        self._stop_capture.set()

        if self._capture_thread and self._capture_thread.is_alive():
            self._capture_thread.join(timeout=2.0)

        # 清理队列
        with self._lock:
            self._output_queues.clear()
            self._frame_queues.clear()
            self._camera_nodes.clear()

            # 停止管道（通过关闭设备上的管道）
            if self._device:
                try:
                    # depthai 没有显式的 stopPipeline，通过重新创建设备来停止
                    pass
                except Exception as e:
                    logger.warning(f"[OAKStreamController] 停止管道时出错: {e}")

            self._pipeline = None
            self._state = StreamState.STOPPED
            self._stats = StreamStats()  # 重置统计

            logger.info("[OAKStreamController] 流已停止")

    def get_state(self) -> StreamState:
        """获取当前流状态。"""
        with self._lock:
            return self._state

    def get_stats(self) -> StreamStats:
        """获取流统计信息。"""
        with self._lock:
            return StreamStats(
                fps=self._stats.fps,
                frame_count=self._stats.frame_count,
                dropped_frames=self._stats.dropped_frames,
                latency_ms=self._stats.latency_ms,
                start_time=self._stats.start_time,
                last_frame_time=self._stats.last_frame_time
            )

    def register_frame_callback(self, callback: Callable[[OAKFrame], None]) -> None:
        """注册帧回调函数。"""
        with self._lock:
            if callback not in self._frame_callbacks:
                self._frame_callbacks.append(callback)

    def unregister_frame_callback(self, callback: Callable[[OAKFrame], None]) -> None:
        """注销帧回调函数。"""
        with self._lock:
            if callback in self._frame_callbacks:
                self._frame_callbacks.remove(callback)

    def get_frame_queue(self, stream_name: str, maxsize: int = 4) -> queue.Queue:
        """获取指定流的帧队列。

        Args:
            stream_name: 流名称
            maxsize: 队列最大大小

        Returns:
            帧队列
        """
        with self._lock:
            if stream_name not in self._frame_queues:
                self._frame_queues[stream_name] = queue.Queue(maxsize=maxsize)
            return self._frame_queues[stream_name]

    # ==================== 内部方法 ====================

    def _build_pipeline(self) -> None:
        """构建 depthai Pipeline。"""
        if self._config is None:
            raise OAKStreamError("配置未设置")

        self._pipeline = dai.Pipeline()
        self._camera_nodes = {}

        # 创建相机节点
        for cam_config in self._config.cameras:
            self._create_camera_node(cam_config)

        # 创建立体深度节点（如果需要）
        if self._config.depth_enabled:
            self._create_stereo_depth_node()

        # 创建输出流
        for stream_config in self._config.streams:
            self._create_output_stream(stream_config)

        logger.info(f"[OAKStreamController] Pipeline 构建完成，包含 {len(self._camera_nodes)} 个相机节点")

    def _create_camera_node(self, config: OAKCameraConfig) -> None:
        """创建相机节点。"""
        if config.camera_type == OAKCameraType.COLOR:
            # 彩色相机
            cam_rgb = self._pipeline.createColorCamera()
            cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
            cam_rgb.setResolution(self._map_resolution(config.resolution))
            cam_rgb.setFps(config.fps)

            # 曝光控制
            if config.manual_exposure and config.exposure_us:
                cam_rgb.initialControl.setManualExposure(config.exposure_us, config.iso or 800)

            # 对焦控制
            if config.manual_focus and config.lens_position is not None:
                cam_rgb.initialControl.setManualFocus(config.lens_position)

            self._camera_nodes["color"] = cam_rgb

        elif config.camera_type == OAKCameraType.MONO:
            # 单目相机（左或右）
            cam_mono = self._pipeline.createMonoCamera()

            # 根据已有节点判断是左还是右
            if "left" not in self._camera_nodes:
                cam_mono.setBoardSocket(dai.CameraBoardSocket.CAM_B)
                cam_mono.setResolution(self._map_resolution(config.resolution))
                self._camera_nodes["left"] = cam_mono
            else:
                cam_mono.setBoardSocket(dai.CameraBoardSocket.CAM_C)
                cam_mono.setResolution(self._map_resolution(config.resolution))
                self._camera_nodes["right"] = cam_mono

            cam_mono.setFps(config.fps)

    def _create_stereo_depth_node(self) -> None:
        """创建立体深度节点。"""
        if "left" not in self._camera_nodes or "right" not in self._camera_nodes:
            logger.warning("[OAKStreamController] 缺少左/右相机，无法创建立体深度")
            return

        stereo = self._pipeline.createStereoDepth()

        # 配置立体深度
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)

        # 设置深度模式
        if self._config.depth_align_to_color and "color" in self._camera_nodes:
            stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)

        # 设置深度计算选项
        stereo.setSubpixel(self._config.depth_subpixel)
        stereo.setExtendedDisparity(self._config.depth_extended_disparity)
        stereo.setLeftRightCheck(self._config.depth_left_right_check)

        # 设置中值滤波
        median_map = {
            "KERNEL_3x3": dai.StereoDepthProperties.MedianFilter.KERNEL_3x3,
            "KERNEL_5x5": dai.StereoDepthProperties.MedianFilter.KERNEL_5x5,
            "KERNEL_7x7": dai.StereoDepthProperties.MedianFilter.KERNEL_7x7,
        }
        if self._config.median_filter in median_map:
            stereo.setMedianFilter(median_map[self._config.median_filter])

        # 连接输入
        left_cam = self._camera_nodes["left"]
        right_cam = self._camera_nodes["right"]

        left_cam.out.link(stereo.left)
        right_cam.out.link(stereo.right)

        self._camera_nodes["stereo"] = stereo

        logger.info("[OAKStreamController] 立体深度节点创建完成")

    def _create_output_stream(self, config: OAKStreamConfig) -> None:
        """创建输出流。"""
        stream_name = config.stream_name
        frame_type = config.frame_type

        # 创建 XLink 输出
        xout = self._pipeline.createXLinkOut()
        xout.setStreamName(stream_name)

        # 根据帧类型连接对应的节点输出
        node = self._get_node_for_frame_type(frame_type)
        if node:
            if hasattr(node, 'out'):
                node.out.link(xout.input)
            elif hasattr(node, 'video'):
                node.video.link(xout.input)
            elif hasattr(node, 'depth'):
                node.depth.link(xout.input)
            elif hasattr(node, 'disparity'):
                node.disparity.link(xout.input)

        # 保存输出队列配置
        self._output_queues[stream_name] = xout

    def _get_node_for_frame_type(self, frame_type: OAKFrameType):
        """根据帧类型获取对应的节点。"""
        type_map = {
            OAKFrameType.COLOR: "color",
            OAKFrameType.LEFT: "left",
            OAKFrameType.RIGHT: "right",
            OAKFrameType.DEPTH: "stereo",
            OAKFrameType.DISPARITY: "stereo",
            OAKFrameType.RECTIFIED_LEFT: "stereo",
            OAKFrameType.RECTIFIED_RIGHT: "stereo",
        }

        node_name = type_map.get(frame_type)
        return self._camera_nodes.get(node_name) if node_name else None

    def _map_resolution(self, resolution: OAKResolution) -> dai.ColorCameraProperties.SensorResolution:
        """映射分辨率到 depthai 枚举。"""
        resolution_map = {
            OAKResolution.THE_1080_P: dai.ColorCameraProperties.SensorResolution.THE_1080_P,
            OAKResolution.THE_720_P: dai.ColorCameraProperties.SensorResolution.THE_720_P,
            OAKResolution.THE_480_P: dai.ColorCameraProperties.SensorResolution.THE_480_P,
            OAKResolution.THE_400_P: dai.ColorCameraProperties.SensorResolution.THE_400_P,
        }
        return resolution_map.get(resolution, dai.ColorCameraProperties.SensorResolution.THE_1080_P)

    def _setup_output_queues(self) -> None:
        """设置输出队列。"""
        if self._device is None:
            return

        for stream_name in self._output_queues.keys():
            # 从设备获取输出队列
            output_queue = self._device.getOutputQueue(
                name=stream_name,
                maxSize=4,
                blocking=False
            )
            # 保存队列引用
            self._frame_queues[stream_name] = output_queue

    def _capture_loop(self) -> None:
        """帧捕获循环。"""
        logger.info("[OAKStreamController] 帧捕获循环已启动")

        while not self._stop_capture.is_set():
            try:
                # 从每个队列获取帧
                for stream_name, q in list(self._frame_queues.items()):
                    try:
                        # 非阻塞获取
                        img_frame = q.tryGet()
                        if img_frame is not None:
                            # 转换为 OAKFrame
                            frame = self._convert_to_oak_frame(img_frame, stream_name)

                            # 更新统计
                            self._update_stats(frame)

                            # 分发帧
                            self._dispatch_frame(frame)

                    except Exception as e:
                        logger.error(f"[OAKStreamController] 处理帧时出错 {stream_name}: {e}")

                # 小延迟避免 CPU 占用过高
                time.sleep(0.001)

            except Exception as e:
                logger.error(f"[OAKStreamController] 捕获循环出错: {e}")
                time.sleep(0.1)

        logger.info("[OAKStreamController] 帧捕获循环已停止")

    def _convert_to_oak_frame(self, img_frame, stream_name: str) -> OAKFrame:
        """将 depthai ImgFrame 转换为 OAKFrame。"""
        # 获取帧数据
        cv_frame = img_frame.getCvFrame()

        # 获取时间戳
        timestamp = img_frame.getTimestamp().total_seconds()
        sequence_num = img_frame.getSequenceNum()

        # 确定帧类型
        frame_type = self._infer_frame_type(stream_name, cv_frame)

        return OAKFrame(
            data=cv_frame,
            frame_type=frame_type,
            timestamp=timestamp,
            sequence_num=sequence_num,
            stream_name=stream_name,
            width=cv_frame.shape[1],
            height=cv_frame.shape[0]
        )

    def _infer_frame_type(self, stream_name: str, frame: np.ndarray) -> OAKFrameType:
        """根据流名称和帧内容推断帧类型。"""
        stream_type_map = {
            "color": OAKFrameType.COLOR,
            "rgb": OAKFrameType.COLOR,
            "left": OAKFrameType.LEFT,
            "right": OAKFrameType.RIGHT,
            "depth": OAKFrameType.DEPTH,
            "disparity": OAKFrameType.DISPARITY,
        }

        # 尝试从流名称映射
        for key, frame_type in stream_type_map.items():
            if key in stream_name.lower():
                return frame_type

        # 根据通道数推断
        if len(frame.shape) == 3 and frame.shape[2] == 3:
            return OAKFrameType.COLOR
        else:
            return OAKFrameType.DEPTH

    def _update_stats(self, frame: OAKFrame) -> None:
        """更新流统计信息。"""
        with self._lock:
            self._stats.frame_count += 1
            self._stats.last_frame_time = time.time()

            if self._stats.start_time:
                elapsed = time.time() - self._stats.start_time
                if elapsed > 0:
                    self._stats.fps = self._stats.frame_count / elapsed

            # 计算延迟（假设帧时间戳是设备时间）
            if frame.timestamp > 0:
                self._stats.latency_ms = (time.time() - frame.timestamp) * 1000

    def _dispatch_frame(self, frame: OAKFrame) -> None:
        """分发帧到所有回调。"""
        # 放入队列
        if frame.stream_name in self._frame_queues:
            try:
                q = self._frame_queues[frame.stream_name]
                # 非阻塞放入，满了则丢弃最旧的
                if q.full():
                    try:
                        q.get_nowait()
                        self._stats.dropped_frames += 1
                    except queue.Empty:
                        pass
                q.put_nowait(frame)
            except queue.Full:
                pass

        # 调用回调
        for callback in self._frame_callbacks:
            try:
                callback(frame)
            except Exception as e:
                logger.error(f"[OAKStreamController] 帧回调出错: {e}")

    def _notify_error(self, error: Exception) -> None:
        """通知错误回调。"""
        for callback in self._error_callbacks:
            try:
                callback(error)
            except Exception as e:
                logger.error(f"[OAKStreamController] 错误回调出错: {e}")

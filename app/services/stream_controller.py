"""流启停与帧采集模块。

从 rs_manager 拆分而来，负责 pipeline 的启停、帧采集线程与后处理。
"""

import copy
import threading
import time
import traceback
from typing import Dict, List, Optional, Set

import cv2
import numpy as np
import pyrealsense2 as rs

from app.core.errors import RealSenseError
from app.models.stream import StreamConfig, StreamStatus
from app.services.coordinate_transform import transform_realsense_to_robot, CAMERA_HEIGHT_DEFAULT


class StreamController:
    """管理 RealSense 数据流的启停、帧采集与后处理。"""

    def __init__(
        self,
        ctx: rs.context,
        lock: threading.Lock,
        devices: Dict[str, rs.device],
        device_infos: dict,
        filters: dict,
        point_cloud_ref,
        metadata_socket_server,
    ):
        self.ctx = ctx
        self.lock = lock
        self.devices = devices
        self.device_infos = device_infos
        # 滤波器集合，由 Facade 注入
        self.filters = filters
        self.pc = point_cloud_ref

        self.pipelines: Dict[str, rs.pipeline] = {}
        self.configs: Dict[str, rs.config] = {}
        self.active_streams: Dict[str, Set[str]] = {}
        self.frame_queues: Dict[str, Dict[str, List]] = {}
        self.metadata_queues: Dict[str, Dict[str, List[Dict]]] = {}
        self.max_queue_size = 5
        self.is_pointcloud_enabled: Dict[str, bool] = {}
        self._last_pc_calc_time: Dict[str, float] = {}

        self.metadata_socket_server = metadata_socket_server

    def start_stream(
        self,
        device_id: str,
        configs: List[StreamConfig],
        align_to: Optional[str] = None,
    ) -> StreamStatus:
        """启动设备的数据流。"""
        if device_id not in self.devices:
            raise RealSenseError(
                status_code=404, detail=f"Device {device_id} not found"
            )

        # 若已在流式传输，直接返回状态
        if device_id in self.pipelines:
            return StreamStatus(
                device_id=device_id,
                is_streaming=True,
                active_streams=list(self.active_streams[device_id]),
            )

        pipeline = rs.pipeline(self.ctx)
        config = rs.config()
        config.enable_device(device_id)

        active_streams: set[str] = set()

        for stream_config in configs:
            self._enable_single_stream(device_id, config, stream_config, active_streams)

        # 尝试自动开启 IMU
        try:
            config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
            config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
            active_streams.add("accel")
            active_streams.add("gyro")
        except RuntimeError:
            print("[Warning] Could not enable IMU streams. Device might not support it.")

        try:
            pipeline.start(config)

            align_processor = self._create_align_processor(align_to)

            with self.lock:
                self.pipelines[device_id] = pipeline
                self.configs[device_id] = config
                self.active_streams[device_id] = active_streams
                self.frame_queues[device_id] = {
                    stream_type: [] for stream_type in active_streams
                }
                self.metadata_queues[device_id] = {
                    stream_key: [] for stream_key in active_streams
                }

            # 启动帧采集守护线程
            threading.Thread(
                target=self._collect_frames,
                args=(device_id, align_processor),
                daemon=True,
            ).start()

            # 更新设备状态
            if device_id in self.device_infos:
                self.device_infos[device_id].is_streaming = True

            # 启动元数据广播
            threading.Thread(
                target=self.metadata_socket_server.start_broadcast,
                args=(device_id,),
                daemon=True,
            ).start()

            return StreamStatus(
                device_id=device_id,
                is_streaming=True,
                active_streams=list(active_streams),
            )

        except RuntimeError as e:
            raise RealSenseError(
                status_code=500, detail=f"Failed to start streaming: {str(e)}"
            )

    def stop_stream(self, device_id: str) -> StreamStatus:
        """停止设备的数据流。"""
        with self.lock:
            if device_id not in self.pipelines:
                return StreamStatus(
                    device_id=device_id, is_streaming=False, active_streams=[]
                )

            try:
                self.metadata_socket_server.stop_broadcast()
                self.pipelines[device_id].stop()

                del self.pipelines[device_id]
                if device_id in self.configs:
                    del self.configs[device_id]
                active_streams = list(self.active_streams.pop(device_id, set()))
                self.frame_queues.pop(device_id, None)
                self.metadata_queues.pop(device_id, None)

                if device_id in self.device_infos:
                    self.device_infos[device_id].is_streaming = False

                return StreamStatus(
                    device_id=device_id,
                    is_streaming=False,
                    active_streams=active_streams,
                )
            except RuntimeError as e:
                raise RealSenseError(
                    status_code=500, detail=f"Failed to stop streaming: {str(e)}"
                )

    def get_stream_status(self, device_id: str) -> StreamStatus:
        """获取设备的流状态。"""
        is_streaming = device_id in self.pipelines
        active_streams = list(self.active_streams.get(device_id, set()))
        return StreamStatus(
            device_id=device_id,
            is_streaming=is_streaming,
            active_streams=active_streams,
        )

    def get_latest_frame(self, device_id: str, stream_type: str) -> np.ndarray:
        """获取指定流的最新帧数据。"""
        with self.lock:
            if device_id not in self.frame_queues:
                raise RealSenseError(
                    status_code=400, detail=f"Device {device_id} is not streaming"
                )
            if stream_type not in self.frame_queues[device_id]:
                raise RealSenseError(
                    status_code=400, detail=f"Stream type {stream_type} is not active"
                )
            if not self.frame_queues[device_id][stream_type]:
                raise RealSenseError(
                    status_code=503,
                    detail=f"No frames available for stream {stream_type}",
                )
            return self.frame_queues[device_id][stream_type][-1]

    def get_latest_metadata(self, device_id: str, stream_type: str) -> Dict:
        """获取指定流的最新元数据。"""
        stream_key = stream_type.lower()
        with self.lock:
            if device_id not in self.pipelines or device_id not in self.metadata_queues:
                if (
                    device_id not in self.pipelines
                    and not self.get_stream_status(device_id).is_streaming
                ):
                    raise RealSenseError(
                        status_code=400, detail=f"Device {device_id} is not streaming."
                    )
                else:
                    raise RealSenseError(
                        status_code=500,
                        detail=f"Inconsistent state for device {device_id}.",
                    )

            if stream_key not in self.metadata_queues.get(device_id, {}):
                active_keys = list(self.active_streams.get(device_id, []))
                raise RealSenseError(
                    status_code=400,
                    detail=f"Stream type '{stream_key}' is not active. Active: {active_keys}",
                )

            queue = self.metadata_queues[device_id][stream_key]
            if len(queue) == 0:
                return {}
            if not queue:
                raise RealSenseError(
                    status_code=503,
                    detail=f"No metadata available yet for stream '{stream_key}'.",
                )

            return copy.deepcopy(queue[-1])

    # --------- 内部方法 ---------

    def _enable_single_stream(
        self,
        device_id: str,
        config: rs.config,
        stream_config: StreamConfig,
        active_streams: set,
    ) -> None:
        """启用单个流配置。"""
        # 解析传感器索引
        try:
            sensor_index = int(stream_config.sensor_id.split("-")[-1])
            if sensor_index < 0 or sensor_index >= len(self.devices[device_id].sensors):
                raise RealSenseError(
                    status_code=404,
                    detail=f"Sensor {stream_config.sensor_id} not found",
                )
        except (ValueError, IndexError):
            raise RealSenseError(
                status_code=404,
                detail=f"Invalid sensor ID format: {stream_config.sensor_id}",
            )

        # 解析流类型
        stream_name_list = stream_config.stream_type.split("-")
        stream_type = None
        for name, val in rs.stream.__members__.items():
            if name.lower() == stream_name_list[0].lower():
                stream_type = val
                break
        if stream_type is None:
            raise RealSenseError(
                status_code=400,
                detail=f"Invalid stream type: {stream_config.stream_type}",
            )

        # 解析格式
        format_type = None
        for name, val in rs.format.__members__.items():
            if name.lower() == stream_config.format.lower():
                format_type = val
                break
        if format_type is None:
            raise RealSenseError(
                status_code=400, detail=f"Invalid format: {stream_config.format}"
            )

        if stream_config.stream_type in active_streams:
            return

        try:
            if len(stream_name_list) > 1:
                stream_index = int(stream_name_list[1])
                config.enable_stream(
                    stream_type,
                    stream_index,
                    stream_config.resolution.width,
                    stream_config.resolution.height,
                    format_type,
                    stream_config.framerate,
                )
            elif format_type == rs.format.combined_motion:
                config.enable_stream(stream_type)
            else:
                config.enable_stream(
                    stream_type,
                    stream_config.resolution.width,
                    stream_config.resolution.height,
                    format_type,
                    stream_config.framerate,
                )
            active_streams.add(stream_config.stream_type)
        except RuntimeError as e:
            raise RealSenseError(
                status_code=400, detail=f"Failed to enable stream: {str(e)}"
            )

    @staticmethod
    def _create_align_processor(align_to: Optional[str]):
        """创建帧对齐处理器。"""
        if not align_to:
            return None
        for name, val in rs.stream.__members__.items():
            if name.lower() == align_to.lower():
                return rs.align(val)
        return None

    def _collect_frames(self, device_id: str, align_processor=None):
        """帧采集主循环（守护线程。"""
        try:
            while device_id in self.pipelines:
                try:
                    frames = self.pipelines[device_id].wait_for_frames()
                    if align_processor:
                        frames = align_processor.process(frames)

                    raw_frames = {}
                    active_streams_snapshot = []
                    should_break = False

                    with self.lock:
                        if device_id not in self.frame_queues:
                            should_break = True
                        else:
                            active_streams_snapshot = list(
                                self.active_streams[device_id]
                            )
                            raw_frames = self._extract_raw_frames(
                                device_id, frames, active_streams_snapshot
                            )

                    if should_break:
                        break

                    # 锁外执行 OpenCV 后处理
                    self._process_and_enqueue(device_id, raw_frames)

                except Exception as e:
                    print(f"Error collecting frames loop: {str(e)}")
                    traceback.print_exc()
                    time.sleep(0.1)

        except Exception as e:
            print(f"Frame collection thread exception: {str(e)}")
            self._emergency_cleanup(device_id)

    def _extract_raw_frames(
        self, device_id: str, frames, active_streams: list
    ) -> dict:
        """在锁内提取原始帧数据（RealSense 滤波须在帧有效期内完成）。"""
        raw_frames = {}

        depth_frame = frames.get_depth_frame()
        if depth_frame:
            depth_frame = self.filters["decimation"].process(depth_frame)
            depth_frame = self.filters["threshold"].process(depth_frame)
            depth_frame = self.filters["spatial"].process(depth_frame)
            depth_frame = self.filters["temporal"].process(depth_frame)

        for stream_type in active_streams:
            stream_name_list = stream_type.split("-")
            stype = stream_name_list[0]

            try:
                if stype.lower() == "depth":
                    frame_data = depth_frame if depth_frame else frames.get_depth_frame()
                    if frame_data is None:
                        continue
                    data_array = np.asanyarray(frame_data.get_data()).copy()
                    raw_frames[stream_type] = {
                        "type": "depth",
                        "data": data_array,
                        "timestamp": frame_data.get_timestamp(),
                        "frame_number": frame_data.get_frame_number(),
                        "width": data_array.shape[1],
                        "height": data_array.shape[0],
                        "frame_data": frame_data,
                    }
                elif stype.lower() == "color":
                    frame_data = frames.get_color_frame()
                    if frame_data is None:
                        continue
                    data_array = np.asanyarray(frame_data.get_data()).copy()
                    raw_frames[stream_type] = {
                        "type": "color",
                        "data": data_array,
                        "timestamp": frame_data.get_timestamp(),
                        "frame_number": frame_data.get_frame_number(),
                        "width": data_array.shape[1],
                        "height": data_array.shape[0],
                    }
                elif stype.lower() == "infrared":
                    frame_data = frames.get_infrared_frame(int(stream_name_list[1]))
                    if frame_data is None:
                        continue
                    data_array = np.asanyarray(frame_data.get_data()).copy()
                    raw_frames[stream_type] = {
                        "type": "infrared",
                        "data": data_array,
                        "timestamp": frame_data.get_timestamp(),
                        "frame_number": frame_data.get_frame_number(),
                        "width": data_array.shape[1],
                        "height": data_array.shape[0],
                    }
                elif stype == rs.stream.gyro.name or stype == rs.stream.accel.name:
                    motion_data = None
                    frame_data = None
                    for f in frames:
                        if f.get_profile().stream_type().name == stype:
                            frame_data = f.as_motion_frame()
                            motion_data = frame_data.get_motion_data()
                    if motion_data is None:
                        continue
                    raw_frames[stream_type] = {
                        "type": "motion",
                        "motion_data": motion_data,
                        "timestamp": frame_data.get_timestamp(),
                        "frame_number": frame_data.get_frame_number(),
                        "width": 640,
                        "height": 480,
                    }

                # 点云计算 - 已禁用，待重写
                # if stype.lower() == "depth" and self.is_pointcloud_enabled.get(
                #     device_id, False
                # ):
                #     import time
                #     now = time.time()
                #     if now - self._last_pc_calc_time.get(device_id, 0.0) >= 0.2:
                #         self._last_pc_calc_time[device_id] = now
                #         fd = raw_frames.get(stream_type, {}).get("frame_data")
                #         if fd:
                #             pts = self.pc.calculate(fd)
                #             v = pts.get_vertices()
                #             verts = (
                #                 np.asanyarray(v)
                #                 .view(np.float32)
                #                 .reshape(-1, 3)
                #                 .copy()
                #             )
                #
                #             # 使用统一的坐标转换模块
                #             verts = transform_realsense_to_robot(verts, CAMERA_HEIGHT_DEFAULT)
                #
                #             raw_frames[stream_type]["point_cloud"] = {
                #                 "vertices": verts,
                #                 "texture_coordinates": [],
                #             }
            except RuntimeError:
                pass

        return raw_frames

    def _process_and_enqueue(self, device_id: str, raw_frames: dict) -> None:
        """锁外完成 OpenCV 后处理并入队。"""
        for stream_type, raw in raw_frames.items():
            stream_name_list = stream_type.split("-")

            try:
                frame = None
                metadata = {
                    "timestamp": raw["timestamp"],
                    "frame_number": raw["frame_number"],
                    "width": raw["width"],
                    "height": raw["height"],
                }

                if raw["type"] == "depth":
                    depth_image = raw["data"]
                    depth_normalized = cv2.normalize(
                        depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U
                    )
                    depth_colormap = cv2.applyColorMap(
                        depth_normalized, cv2.COLORMAP_INFERNO
                    )
                    edges = cv2.Canny(depth_normalized, 25, 100)
                    edges = cv2.dilate(edges, None)
                    depth_colormap[edges > 0] = [0, 0, 0]
                    frame = cv2.cvtColor(depth_colormap, cv2.COLOR_BGR2RGB)
                    # 点云数据已禁用，不添加到元数据
                    # if "point_cloud" in raw:
                    #     metadata["point_cloud"] = raw["point_cloud"]

                elif raw["type"] == "color":
                    frame = raw["data"]

                elif raw["type"] == "infrared":
                    frame = raw["data"]

                elif raw["type"] == "motion":
                    motion_data = raw["motion_data"]
                    metadata["motion_data"] = {
                        "x": float(motion_data.x),
                        "y": float(motion_data.y),
                        "z": float(motion_data.z),
                    }
                    text = f"x: {motion_data.x:.6f}\ny: {motion_data.y:.6f}\nz: {motion_data.z:.6f}".split(
                        "\n"
                    )
                    frame = np.zeros((480, 640, 3), dtype=np.uint8)
                    y0, dy = 50, 40
                    for i, coord in enumerate(text):
                        y = y0 + dy * i
                        cv2.putText(
                            frame,
                            coord,
                            (10, y),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1,
                            (255, 255, 255),
                            2,
                            cv2.LINE_AA,
                        )

                if frame is None:
                    continue

                frame = np.asanyarray(frame)

                with self.lock:
                    if device_id not in self.frame_queues:
                        break
                    stream_key = "-".join(stream_name_list)
                    frame_queue = self.frame_queues[device_id][stream_key]
                    frame_queue.append(frame)
                    while len(frame_queue) > self.max_queue_size:
                        frame_queue.pop(0)

                    metadata_queue = self.metadata_queues[device_id][stream_key]
                    metadata_queue.append(metadata)
                    while len(metadata_queue) > self.max_queue_size:
                        metadata_queue.pop(0)

            except Exception as e:
                print(f"[ERROR] Processing frame for {stream_type}: {e}")
                traceback.print_exc()

    def _emergency_cleanup(self, device_id: str) -> None:
        """帧采集线程异常时的紧急清理。"""
        try:
            with self.lock:
                if device_id in self.pipelines:
                    self.pipelines[device_id].stop()
                    del self.pipelines[device_id]
                    self.configs.pop(device_id, None)
                    self.active_streams.pop(device_id, None)
                    self.frame_queues.pop(device_id, None)
                    self.metadata_queues.pop(device_id, None)
                    if device_id in self.device_infos:
                        self.device_infos[device_id].is_streaming = False
        except Exception:
            pass

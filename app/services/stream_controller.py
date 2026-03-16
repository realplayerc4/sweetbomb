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
from app.services.coordinate_transform import transform_realsense_to_robot
from app.services.point_cloud_analyzer import PointCloudAnalyzer


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
        self.max_queue_size = 1  # 缩小堆积队列，仅保留最新一帧以保证极高实时响应
        self.is_pointcloud_enabled: Dict[str, bool] = {}
        self._last_pc_calc_time: Dict[str, float] = {}

        self.metadata_socket_server = metadata_socket_server

        # 点云分析器（用于后端计算体积、距离等）
        self.point_cloud_analyzer = PointCloudAnalyzer()
        self.latest_analysis_result = None  # 存储最新分析结果

        # 健康监控相关
        self.last_frame_time: Dict[str, float] = {}  # device_id -> timestamp
        self._stream_configs_cache: Dict[str, dict] = {}  # device_id -> {configs, align_to}

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

        # 缓存启动配置以便后续健康检查时自动重启
        self._stream_configs_cache[device_id] = {
            "configs": configs,
            "align_to": align_to
        }
        self.last_frame_time[device_id] = time.time()

        pipeline = rs.pipeline(self.ctx)
        config = rs.config()
        config.enable_device(device_id)

        active_streams: set[str] = set()

        for stream_config in configs:
            self._enable_single_stream(device_id, config, stream_config, active_streams)


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
                # 初始化点云启用状态 - 必须在锁内完成
                self.is_pointcloud_enabled[device_id] = False

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

    def check_stream_health(self, device_id: str) -> bool:
        """检查流健康状况。如果超过 10 秒无帧且流应该在运行，则返回 False。"""
        if device_id not in self.pipelines:
            return True # 未启动流不视为不健康

        last_time = self.last_frame_time.get(device_id, 0)
        if time.time() - last_time > 10.0:
            print(f"[HealthCheck] Device {device_id} has no frames for 10s. Triggering restart...")
            try:
                # 尝试重启
                cache = self._stream_configs_cache.get(device_id)
                if cache:
                    self.stop_stream(device_id)
                    time.sleep(1) # 等待释放
                    self.start_stream(device_id, cache["configs"], cache["align_to"])
                    return True
            except Exception as e:
                print(f"[HealthCheck] Failed to auto-restart device {device_id}: {str(e)}")
            return False
        return True

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
                    pc_enabled = False
                    should_break = False

                    with self.lock:
                        if device_id not in self.frame_queues:
                            should_break = True
                        else:
                            active_streams_snapshot = list(
                                self.active_streams[device_id]
                            )
                            pc_enabled = self.is_pointcloud_enabled.get(device_id, False)

                    if should_break:
                        break

                    # 锁外执行 RealSense 滤波与帧提取
                    raw_frames = self._extract_raw_frames(
                        device_id, frames, active_streams_snapshot, pc_enabled
                    )

                    # 锁外执行 OpenCV 后处理 (局部入队时加锁)
                    self._process_and_enqueue(device_id, raw_frames)

                    # 更新最后一次成功采集到的帧的时间戳
                    if raw_frames:
                        self.last_frame_time[device_id] = time.time()

                except Exception as e:
                    print(f"Error collecting frames loop: {str(e)}")
                    traceback.print_exc()
                    time.sleep(0.1)

        except Exception as e:
            print(f"Frame collection thread exception: {str(e)}")
            self._emergency_cleanup(device_id)

    def _extract_raw_frames(
        self, device_id: str, frames, active_streams: list, pc_enabled: bool
    ) -> dict:
        """提取原始帧数据（RealSense 滤波、点云计算移出锁外执行）。"""
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

                if stream_type not in raw_frames:
                    print(f"[DEBUG_STREAM] Could not extract raw frame for {stream_type}. frames object contains: {[f.get_profile().stream_type().name for f in frames]}")

                # 点云计算
                if stype.lower() == "depth" and pc_enabled:
                    import time
                    now = time.time()
                    if now - self._last_pc_calc_time.get(device_id, 0.0) >= 0.2:
                        self._last_pc_calc_time[device_id] = now
                        fd = raw_frames.get(stream_type, {}).get("frame_data")
                        if fd:
                            print(f"[PointCloudCalc] Calculating point cloud for {device_id}...")
                            pts = self.pc.calculate(fd)
                            v = pts.get_vertices()
                            verts = (
                                np.asanyarray(v)
                                .view(np.float32)
                                .reshape(-1, 3)
                                .copy()
                            )
                            print(f"[PointCloudCalc] Raw vertices count: {len(verts)}")

                            # 使用统一的坐标转换模块（转换为 ROS 坐标系）
                            verts = transform_realsense_to_robot(verts)
                            
                            # 【精细化空间分区采样】
                            # 中心焦点区 [-0.3m, 0.3m] 且深度 <= 4m 用于体积和切面高度积分，不抽稀保准
                            # 边缘区或深度超过 4m 的区域由于仅作周围地形辅助呈现，予以 1/30 重度抽稀以防前端计算假死
                            x_coords = verts[:, 0]
                            y_coords = verts[:, 1]
                            in_roi = (y_coords >= -0.3) & (y_coords <= 0.3) & (x_coords <= 4.0)
                            
                            verts_roi = verts[in_roi]
                            verts_outside = verts[~in_roi][::30]  # 边缘及远端强力裁切
                            
                            if len(verts_roi) > 0 or len(verts_outside) > 0:
                                # 合并并保证内存连续性用于 Socket 转字节传输
                                verts = np.vstack((verts_roi, verts_outside)).copy()
                            else:
                                verts = np.empty((0, 3), dtype=np.float32)

                            print(f"[PointCloudCalc] ROI points (X<=4m, Y±0.3m): {len(verts_roi)}, Outside points: {len(verts_outside)}, Total Final: {len(verts)}")

                            raw_frames[stream_type]["point_cloud"] = {
                                "vertices": verts,
                                "texture_coordinates": [],
                            }

                            # 执行点云分析（体积、距离计算等）
                            try:
                                # 从设备信息或默认值获取参数
                                # 注意：这些参数应该从配置中读取，这里使用合理默认值
                                teeth_height = getattr(self, 'slice_z1', -0.1)
                                bucket_volume = getattr(self, 'target_volume', 30.0)  # 默认30L
                                bucket_depth = getattr(self, 'bucket_depth', 0.3)
                                camera_to_teeth = getattr(self, 'camera_to_teeth', 0.8)  # 默认0.8m

                                analysis_result = self.point_cloud_analyzer.analyze(
                                    point_cloud=verts,
                                    target_volume=bucket_volume,
                                    camera_to_teeth=camera_to_teeth,
                                    z1=teeth_height,
                                    z2=teeth_height + 0.3,  # 铲斗高度0.3m
                                )

                                # 将分析结果转换为可序列化的字典
                                self.latest_analysis_result = {
                                    "volume": {
                                        "current": round(analysis_result.actual_volume * 1000, 2),  # 转换为升
                                        "target": round(analysis_result.target_volume * 1000, 1),
                                        "reached": analysis_result.volume_reached,
                                    },
                                    "distances": {
                                        "nearest_material": round(analysis_result.material_distance, 3) if analysis_result.material_distance else None,
                                        "nearest_x": round(analysis_result.nearest_x, 3) if analysis_result.nearest_x else None,
                                        "nearest_y": round(analysis_result.nearest_y, 3) if analysis_result.nearest_y else None,
                                    },
                                    "pile_height": round(analysis_result.pile_height, 3) if hasattr(analysis_result, 'pile_height') else None,
                                    "pile_height": round(analysis_result.pile_height, 3) if hasattr(analysis_result, 'pile_height') else None,
                                    "target_depth": {
                                        "x": round(analysis_result.target_depth_x, 3) if analysis_result.target_depth_x else None,
                                    },
                                    "has_material": analysis_result.has_material,
                                    "timestamp": time.time(),
                                }

                                print(f"[PointCloudAnalysis] Volume: {self.latest_analysis_result['volume']['current']:.2f}L, "
                                      f"Nearest: {self.latest_analysis_result['distances']['nearest_material']:.3f}m")

                            except Exception as e:
                                print(f"[PointCloudAnalysis] Error during analysis: {e}")
                                import traceback
                                traceback.print_exc()

                        else:
                            print(f"[PointCloudCalc] No frame data available for {device_id}")
                else:
                    if stype.lower() == "depth" and False: # Debug only
                        print(f"[PointCloudCalc] Depth stream processing but point cloud DISABLED (pc_enabled={pc_enabled})")
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

                # 点云数据直接传递 - 不检查 type
                if "point_cloud" in raw:
                    metadata["point_cloud"] = raw["point_cloud"]

                if raw.get("type") == "depth":
                    depth_image = raw["data"]
                    depth_normalized = cv2.normalize(
                        depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U
                    )
                    depth_colormap = cv2.applyColorMap(
                        depth_normalized, cv2.COLORMAP_INFERNO
                    )
                    edges = cv2.Canny(depth_normalized, 25, 100)
                    # 取消膨胀操作或者设积极细小的核，这里选择使用极其细微的核以保持微弱连接（如果不要任何扩展可以直接注销）
                    # edges = cv2.dilate(edges, None)
                    depth_colormap[edges > 0] = [0, 0, 0]
                    frame = cv2.cvtColor(depth_colormap, cv2.COLOR_BGR2RGB)

                elif raw.get("type") == "color":
                    frame = raw["data"]

                elif raw["type"] == "infrared":
                    frame = raw["data"]


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

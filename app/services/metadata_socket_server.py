import base64
import time
import threading
from typing import Optional, Dict
import asyncio
import psutil
import socket
import random
import math


class MetadataSocketServer:
    """
    从 RealSenseManager 获取元数据，并通过 Socket.IO 服务实例广播。
    """

    def __init__(
        self,
        sio,  # 可以是 socketio.Server 或 socketio.AsyncServer
        rs_manager,
        update_interval: float = 1.0/30.0,  # 默认 30 FPS
    ):
        self._sio = sio
        self._rs_manager = rs_manager
        self._update_interval = update_interval
        self._broadcast_thread: Optional[threading.Thread] = None
        self._target_device_id: Optional[str] = None
        self._is_broadcasting = False
        self._thread_stop_event = threading.Event()
        self._async_loop = None

    def _emit_event(self, event_name, data):
        """处理同步/异步 Socket.IO 服务发射事件的辅助方法。"""
        if self._async_loop is None:
            self._async_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._async_loop)

        async def async_emit():
            await self._sio.emit(event_name, data)

        # 在事件循环中运行协程
        self._async_loop.run_until_complete(async_emit())

    def _broadcast_metadata_loop(self):
        """获取并广播元数据的核心循环。"""
        print("[MetadataBroadcaster] Starting broadcast loop...")

        if not self._async_loop:
            self._async_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._async_loop)

        last_cpu_load = 0.0
        last_cpu_time = 0.0
        last_pc_timestamp = 0.0

        while self._is_broadcasting and not self._thread_stop_event.is_set():
            start_time = time.monotonic()

            if not self._target_device_id:
                time.sleep(self._update_interval)
                continue

            # --- 获取状态与元数据 ---
            active_streams = []
            is_streaming = False
            try:
                status = self._rs_manager.get_stream_status(self._target_device_id)
                is_streaming = status.is_streaming
                if is_streaming:
                    active_streams = status.active_streams
            except Exception as e:
                print(
                    f"[MetadataBroadcaster] Error getting stream status for {self._target_device_id}: {e}"
                )
                active_streams = []
            except Exception as e:
                print(f"[MetadataBroadcaster] Unexpected error getting status: {e}")
                active_streams = []

            all_metadata: Dict[str, Optional[Dict]] = {}
            if is_streaming and active_streams:
                for stream_type in active_streams:
                    try:
                        metadata = self._rs_manager.get_latest_metadata(
                            self._target_device_id, stream_type
                        )
                        if (
                            stream_type == "depth"
                            and "point_cloud" in metadata
                        ):
                            current_ts = metadata.get("timestamp", 0)
                            if current_ts == last_pc_timestamp:
                                # 避免重复编码并发送相同的点云
                                del metadata["point_cloud"]
                            else:
                                last_pc_timestamp = current_ts
                                if "vertices" in metadata["point_cloud"]:
                                    metadata["point_cloud"]["vertices"] = base64.b64encode(
                                        metadata["point_cloud"]["vertices"].tobytes()
                                    ).decode("utf-8")
                        all_metadata[stream_type] = metadata
                    except Exception as e:
                        if hasattr(e, "status_code"):
                            if e.status_code == 503 or e.status_code == 400:
                                all_metadata[stream_type] = None
                            else:
                                all_metadata[stream_type] = {"error": str(e)}
                        else:
                            all_metadata[stream_type] = {
                                "error": f"Unexpected: {str(e)}"
                            }

            # --- 获取系统状态 ---
            current_time = time.time()
            if current_time - last_cpu_time >= 1.0:
                last_cpu_load = psutil.cpu_percent()
                last_cpu_time = current_time

            system_stats = {
                "cpu_load": last_cpu_load,
                "hostname": socket.gethostname(),
            }

            # --- 通过 sio 实例发射 ---
            payload = {
                "device_id": self._target_device_id,
                "is_streaming": is_streaming,
                "timestamp_server": time.time(),
                "metadata_streams": all_metadata,
                "system_stats": system_stats,
            }
            try:
                # 调试：检查 payload 中是否包含点云
                if "depth" in all_metadata and all_metadata["depth"]:
                    pass
                # 使用辅助方法进行适当的 emit
                self._emit_event("metadata_update", payload)
            except Exception as e:
                print(
                    f"[MetadataBroadcaster] Error emitting 'metadata_update' event: {e}"
                )

            # --- 休眠 ---
            elapsed_time = time.monotonic() - start_time
            sleep_duration = max(0, self._update_interval - elapsed_time)
            time.sleep(sleep_duration)

        print("[MetadataBroadcaster] Broadcast loop stopped.")

    def start_broadcast(self, device_id: str):
        """启动后台线程形式的元数据广播循环。"""
        if self._is_broadcasting:
            return

        if not device_id:
            raise ValueError("A target device_id must be provided.")

        self._target_device_id = device_id
        self._is_broadcasting = True
        self._thread_stop_event.clear()

        self._broadcast_thread = threading.Thread(
            target=self._broadcast_metadata_loop, daemon=True
        )
        self._broadcast_thread.start()

        print(
            f"[MetadataBroadcaster] Broadcast loop started for device: {self._target_device_id}"
        )

    def stop_broadcast(self):
        """优雅停止元数据广播循环。"""
        if not self._is_broadcasting or not self._broadcast_thread:
            return

        print("[MetadataBroadcaster] Stopping broadcast loop...")
        self._is_broadcasting = False
        self._thread_stop_event.set()

        # 清理线程资源
        if self._broadcast_thread and self._broadcast_thread.is_alive():
            self._broadcast_thread.join(
                timeout=2.0
            )  # Wait for thread to terminate with timeout

        # 清理异步事件循环资源
        if self._async_loop:
            self._async_loop.close()
            self._async_loop = None

        self._broadcast_thread = None
        self._target_device_id = None
        print("[MetadataBroadcaster] Broadcast loop stopped.")

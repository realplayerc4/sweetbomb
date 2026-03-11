"""WebRTC 视频流管理器。

负责创建和管理 WebRTC PeerConnection，将 RealSense 视频流传输给客户端。
支持多客户端连接，自动清理过期会话。
"""

import asyncio
import uuid
import time
from typing import Dict, List, Any, Tuple
import numpy as np
import cv2
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCIceCandidate, RTCConfiguration, RTCIceServer
from aiortc.sdp import candidate_from_sdp
from aiortc.mediastreams import VideoStreamTrack
from av import VideoFrame
from app.core.errors import RealSenseError
from app.core.config import get_settings
from app.models.webrtc import WebRTCStatus


class RealSenseVideoTrack(VideoStreamTrack):
    """RealSense 视频轨道。

    从 RealSense 摄像头捕获帧并转换为 WebRTC 视频轨道。
    """

    def __init__(self, realsense_manager, device_id, stream_type):
        """初始化视频轨道。

        Args:
            realsense_manager: RealSense 管理器实例
            device_id: 设备 ID
            stream_type: 流类型（color、depth 等）
        """
        super().__init__()
        self.realsense_manager = realsense_manager
        self.device_id = device_id
        self.stream_type = stream_type
        self._start = time.time()
        self._last_frame_id = None

    async def recv(self):
        """接收视频帧。

        Returns:
            VideoFrame: 视频帧对象
        """
        try:
            # 防空转重试，等待新的一帧出现
            for _ in range(50):  # 最多等待约 0.5 秒
                # 从 RealSense 获取帧数据
                frame_data = self.realsense_manager.get_latest_frame(self.device_id, self.stream_type)
                current_frame_id = id(frame_data)
                if current_frame_id != self._last_frame_id:
                    self._last_frame_id = current_frame_id
                    break
                await asyncio.sleep(0.01)

            # 即使没获取到新的（极端情况），也发送手头最后这一帧，防止编码器报错
            
            # 必要时转换为 RGB 格式
            if len(frame_data.shape) == 3 and frame_data.shape[2] == 3:
                # 已经是 RGB 格式，无需转换
                img = frame_data
            elif len(frame_data.shape) == 2 or (len(frame_data.shape) == 3 and frame_data.shape[2] == 1):
                # 灰度图转 RGB
                img = cv2.cvtColor(frame_data, cv2.COLOR_GRAY2RGB)
            else:
                # 转换为 RGB (宽泛兜底)
                img = cv2.cvtColor(frame_data, cv2.COLOR_BGR2RGB)
            # 创建 VideoFrame
            video_frame = VideoFrame.from_ndarray(img, format="rgb24")

            # 设置帧时间戳
            pts, time_base = await self.next_timestamp()
            video_frame.pts = pts
            video_frame.time_base = time_base

            return video_frame
        except Exception as e:
            # 出错时返回黑帧
            width, height = 640, 480  # 默认尺寸
            img = np.zeros((height, width, 3), dtype=np.uint8)
            video_frame = VideoFrame.from_ndarray(img, format="rgb24")
            pts, time_base = await self.next_timestamp()
            video_frame.pts = pts
            video_frame.time_base = time_base

            import traceback
            exc_info = traceback.format_exc()
            print(f"Error getting frame: type={type(e)} repr={repr(e)} exc={exc_info}")
            return video_frame


class WebRTCManager:
    """WebRTC 会话管理器。

    负责：
    - 创建和管理 WebRTC PeerConnection
    - 将 RealSense 视频流添加到连接
    - 处理 ICE 候选
    - 清理过期会话
    """

    def __init__(self, realsense_manager):
        """初始化 WebRTC 管理器。

        Args:
            realsense_manager: RealSense 管理器实例
        """
        self.realsense_manager = realsense_manager
        self.sessions: Dict[str, Dict[str, Any]] = {}  # session_id -> session_data
        self.lock = asyncio.Lock()
        self.settings = get_settings()

        # 设置 WebRTC 的 ICE 服务器
        self.ice_servers = []

        if self.settings.STUN_SERVER:
            self.ice_servers.append(RTCIceServer(urls=self.settings.STUN_SERVER))

        if self.settings.TURN_SERVER:
            self.ice_servers.append(
                RTCIceServer(
                    urls=self.settings.TURN_SERVER,
                    username=self.settings.TURN_USERNAME,
                    credential=self.settings.TURN_PASSWORD
                )
            )

        # 后台清理任务
        self.cleanup_task = None

    async def start_cleanup_loop(self):
        """启动后台清理循环。"""
        if self.cleanup_task is None or self.cleanup_task.done():
            self.cleanup_task = asyncio.create_task(self._cleanup_loop())

    async def _cleanup_loop(self):
        """后台循环，定期清理过期会话。"""
        while True:
            await asyncio.sleep(60)  # 每分钟检查一次
            await self._cleanup_sessions()

    async def create_offer(self, device_id: str, stream_types: List[str]) -> Tuple[str, dict]:
        """创建 WebRTC offer 用于设备视频流。

        Args:
            device_id: 设备 ID
            stream_types: 请求的流类型列表（color、depth 等）

        Returns:
            Tuple[session_id, offer_data]: 会话 ID 和 offer 数据

        Raises:
            RealSenseError: 设备未流式传输或流类型不可用
        """
        # 验证设备存在且正在流式传输
        stream_status = self.realsense_manager.get_stream_status(device_id)
        print(f"[DEBUG] create_offer: device={device_id}, is_streaming={stream_status.is_streaming}")
        print(f"[DEBUG] active_streams: {stream_status.active_streams}")
        print(f"[DEBUG] requested_streams: {stream_types}")

        if not stream_status.is_streaming:
            raise RealSenseError(status_code=400, detail=f"Device {device_id} is not streaming")

        # 验证请求的流类型可用
        for stream_type in stream_types:
            if stream_type not in stream_status.active_streams:
                raise RealSenseError(status_code=400, detail=f"Stream type {stream_type} is not active (Active: {stream_status.active_streams})")

        # 创建 PeerConnection
        pc = RTCPeerConnection(RTCConfiguration(iceServers=self.ice_servers))

        # 创建会话 ID
        session_id = str(uuid.uuid4())

        # 为每个流类型添加视频轨道
        for stream_type in stream_types:
            video_track = RealSenseVideoTrack(self.realsense_manager, device_id, stream_type)
            pc.addTrack(video_track)

        # 创建 offer
        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)

        # 创建流映射（mid -> stream_type）
        stream_map = {}
        for transceiver in pc.getTransceivers():
            if transceiver.sender and transceiver.sender.track:
                 # 找到该轨道属于哪个流类型
                 track = transceiver.sender.track
                 if isinstance(track, RealSenseVideoTrack):
                     stream_map[transceiver.mid] = track.stream_type

        # 存储会话
        async with self.lock:
            self.sessions[session_id] = {
                "device_id": device_id,
                "stream_types": stream_types,
                "pc": pc,
                "connected": False,
                "created_at": time.time(),
                "stream_map": stream_map
            }

        # 返回会话 ID 和 offer
        return session_id, {
            "sdp": pc.localDescription.sdp,
            "type": pc.localDescription.type,
            "stream_map": stream_map
        }

    async def process_answer(self, session_id: str, sdp: str, type_: str) -> bool:
        """处理 WebRTC answer。

        Args:
            session_id: 会话 ID
            sdp: SDP 字符串
            type_: SDP 类型（answer）

        Returns:
            bool: 是否成功处理

        Raises:
            RealSenseError: 会话不存在或处理失败
        """
        async with self.lock:
            if session_id not in self.sessions:
                raise RealSenseError(status_code=404, detail=f"Session {session_id} not found")

            pc = self.sessions[session_id]["pc"]

        # 设置远程描述
        try:
            print(f"[DEBUG] Processing WebRTC answer for session: {session_id}")
            print(f"[DEBUG] SDP type: {type_}")
            
            # 如果已经处于 stable 状态，说明已经处理过 answer 了
            if pc.signalingState == "stable":
                print(f"[DEBUG] Session {session_id} is already in stable state, ignoring redundant answer.")
                return True

            await pc.setRemoteDescription(RTCSessionDescription(sdp=sdp, type=type_))

            # 标记为已连接
            async with self.lock:
                self.sessions[session_id]["connected"] = True

            print(f"[DEBUG] Successfully set remote description for session: {session_id}")
            return True
        except Exception as e:
            import traceback
            error_trace = traceback.format_exc()
            print(f"[ERROR] Failed to set remote description for session {session_id}: {str(e)}")
            print(f"[ERROR] Traceback: {error_trace}")
            raise RealSenseError(status_code=400, detail=f"Error processing answer: {str(e)}")

    async def add_ice_candidate(self, session_id: str, candidate: str, sdp_mid: str, sdp_mline_index: int) -> bool:
        """向会话添加 ICE 候选。

        Args:
            session_id: 会话 ID
            candidate: ICE 候选字符串
            sdp_mid: SDP mid
            sdp_mline_index: SDP 行索引

        Returns:
            bool: 是否成功添加

        Raises:
            RealSenseError: 会话不存在或添加失败
        """
        async with self.lock:
            if session_id not in self.sessions:
                raise RealSenseError(status_code=404, detail=f"Session {session_id} not found")

            pc = self.sessions[session_id]["pc"]

        # 添加 ICE 候选
        try:
            # 使用 aiortc.sdp.candidate_from_sdp 解析候选字符串
            # 候选字符串通常包含 "candidate:" 前缀，需要确保格式正确
            candidate_line = candidate
            if not candidate_line.startswith("candidate:"):
                candidate_line = "candidate:" + candidate
            
            candidate_obj = candidate_from_sdp(candidate_line)
            candidate_obj.sdpMid = sdp_mid
            candidate_obj.sdpMLineIndex = sdp_mline_index
            
            await pc.addIceCandidate(candidate_obj)
            return True
        except Exception as e:
            raise RealSenseError(status_code=400, detail=f"Error adding ICE candidate: {str(e)}")

    async def get_ice_candidates(self, session_id: str) -> List[dict]:
        """获取会话的 ICE 候选。

        Args:
            session_id: 会话 ID

        Returns:
            List[dict]: ICE 候选列表

        Raises:
            RealSenseError: 会话不存在
        """
        async with self.lock:
            if session_id not in self.sessions:
                raise RealSenseError(status_code=404, detail=f"Session {session_id} not found")

        # ICE 候选在实际应用中会通过事件发送
        # 这是一个占位符，用于 API
        return []

    async def get_session(self, session_id: str) -> WebRTCStatus:
        """获取会话状态。

        Args:
            session_id: 会话 ID

        Returns:
            WebRTCStatus: 会话状态信息

        Raises:
            RealSenseError: 会话不存在
        """
        async with self.lock:
            if session_id not in self.sessions:
                raise RealSenseError(status_code=404, detail=f"Session {session_id} not found")

            session = self.sessions[session_id]
            pc = session["pc"]

        # 获取 WebRTC 统计信息（如果可用）
        stats = None
        try:
            stats_dict = await pc.getStats()
            stats = {k: v.__dict__ for k, v in stats_dict.items()}
        except Exception:
            stats = None

        # 返回会话状态
        return WebRTCStatus(
            session_id=session_id,
            device_id=session["device_id"],
            connected=session["connected"],
            streaming=session["connected"],
            stream_types=session["stream_types"],
            stats=stats
        )

    async def close_session(self, session_id: str) -> bool:
        """关闭 WebRTC 会话。

        Args:
            session_id: 会话 ID

        Returns:
            bool: 是否成功关闭
        """
        async with self.lock:
            if session_id not in self.sessions:
                return False

            # 关闭 PeerConnection
            try:
                await self.sessions[session_id]["pc"].close()
            except Exception:
                pass

            # 删除会话
            del self.sessions[session_id]
            return True

    async def _cleanup_sessions(self):
        """清理过期或断开的会话。"""
        async with self.lock:
            now = time.time()
            session_ids = list(self.sessions.keys())

            for session_id in session_ids:
                session = self.sessions[session_id]

                # 删除超过 1 小时的会话
                if now - session["created_at"] > 3600:
                    try:
                        await session["pc"].close()
                    except Exception:
                        pass

                    del self.sessions[session_id]

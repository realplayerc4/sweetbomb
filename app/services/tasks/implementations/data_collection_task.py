"""Data collection task with real hardware grounding."""

import asyncio
import json
import logging
import os
import time
from datetime import datetime
from typing import Any, Dict, Optional

import cv2
import numpy as np

from app.models.task import TaskResult
from app.services.tasks.base_task import BaseTask
from app.services.tasks.registry import register_task

# 假设全局能够获得门面单例对象，对于依赖注入或单例，这需要与 router 生命周期对接
# 根据现有架构，将在 app.main 中附加 app.state.rs_manager
from starlette.requests import Request


logger = logging.getLogger(__name__)


@register_task
class DataCollectionTask(BaseTask):
    """
    Task for collecting and saving real sensor data from RealSense.
    
    1. Grabs latest RGB and Depth frames from rs_manager memory.
    2. Saves them locally to data/collections/{task_id}/ using cv2.
    3. Writes a metadata.json track record.
    """

    task_type = "data_collection"
    name = "数据采集"
    description = "定时截取并保存真实摄像头传感器帧数据至本地磁盘"
    category = "data"
    requires_device = True

    params_schema = {
        "type": "object",
        "properties": {
            "sensors": {
                "type": "array",
                "items": {"type": "string", "enum": ["color", "depth"]},
                "default": ["color", "depth"],
                "description": "要采集的传感器通道列表 (color / depth)"
            },
            "duration_seconds": {
                "type": "integer",
                "minimum": 1,
                "maximum": 3600,
                "default": 10,
                "description": "采集总时长（秒）"
            },
            "fps": {
                "type": "integer",
                "minimum": 1,
                "maximum": 30,
                "default": 2,
                "description": "每秒采集帧率设定"
            },
            "format": {
                "type": "string",
                "enum": ["png", "jpg"],
                "default": "png",
                "description": "保存的图片格式"
            }
        },
        "additionalProperties": False
    }

    def __init__(
        self,
        task_id: str,
        config,
        params: Dict[str, Any],
        device_id: Optional[str] = None
    ):
        super().__init__(task_id, config, params, device_id)

        # 挂载真实的 rs_manager，此处我们将通过后续传入或从全局获取
        # 暂时将引用置为 None，实际运行期需要框架注入
        self.rs_manager = None
        
        # 参数
        self.duration = self.params.get("duration_seconds", 10)
        self.sensors = self.params.get("sensors", ["color", "depth"])
        self.fps = self.params.get("fps", 2)
        self.format_ext = self.params.get("format", "png")

        # 确定存盘目录
        base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../data/collections"))
        self.save_dir = os.path.join(base_dir, self.task_id)

    def validate(self) -> bool:
        if not self.device_id:
            raise ValueError("device_id must be provided for data collection")
        if self.duration <= 0:
            raise ValueError("duration_seconds must be > 0")
        if self.fps <= 0 or self.fps > 30:
            raise ValueError("fps must be between 1 and 30")
        return True

    def setup(self):
        logger.info(f"Setting up real data collection for task {self.task_id} in {self.save_dir}")
        os.makedirs(self.save_dir, exist_ok=True)
        # 初始化一份空的元数据索引文件
        self.metadata = {
            "task_id": self.task_id,
            "device_id": self.device_id,
            "start_time": datetime.now().isoformat(),
            "params": self.params,
            "frames": []
        }
        self._save_metadata()

    def _save_metadata(self):
        meta_path = os.path.join(self.save_dir, "metadata.json")
        try:
            with open(meta_path, "w", encoding="utf-8") as f:
                json.dump(self.metadata, f, ensure_ascii=False, indent=2)
        except Exception as e:
            logger.error(f"Failed to write metadata for {self.task_id}: {e}")

    async def run(self) -> TaskResult:
        if not self.rs_manager:
            return TaskResult(
                success=False,
                message="系统缺少感知层注入引用 (rs_manager)，无法读取真实视频缓冲帧。",
                error="Reference missing"
            )

        interval_sec = 1.0 / self.fps
        total_expected_frames = self.duration * self.fps
        frames_collected = 0

        self.update_progress(
            current_step=0,
            total_steps=total_expected_frames,
            message=f"开始保存实机采样数据并压制 ({self.duration}秒 @ {self.fps}FPS)..."
        )

        start_time = time.time()
        
        while frames_collected < total_expected_frames:
            await self.async_check_paused()
            
            loop_start = time.time()
            if loop_start - start_time > self.duration:
                # 达到超时设定退出
                break

            record_entry = {
                "frame_id": frames_collected,
                "timestamp": datetime.now().isoformat(),
                "files": {}
            }

            successful_save = False

            try:
                for sensor in self.sensors:
                    # 调用 rs_manager facade 从缓冲区抓取内存画面缓存
                    frame_array = self.rs_manager.get_latest_frame(self.device_id, sensor)
                    
                    if frame_array is not None and frame_array.size > 0:
                        save_img = frame_array
                        
                        # RGB -> BGR 为了 OpenCV 编码
                        if sensor == "color" and len(frame_array.shape) == 3:
                            save_img = cv2.cvtColor(frame_array, cv2.COLOR_RGB2BGR)

                        filename = f"{sensor}_{frames_collected:04d}.{self.format_ext}"
                        filepath = os.path.join(self.save_dir, filename)
                        
                        cv2.imwrite(filepath, save_img)
                        record_entry["files"][sensor] = filename
                        successful_save = True
                        
            except Exception as e:
                logger.error(f"Error copying frames to disk at step {frames_collected}: {e}")

            if successful_save:
                self.metadata["frames"].append(record_entry)
                frames_collected += 1

            self.update_progress(
                current_step=frames_collected,
                message=f"入库 {frames_collected} / {total_expected_frames} 物理帧..."
            )

            # 五帧刷一次索引缓存
            if frames_collected % 5 == 0:
                self._save_metadata()

            if self._stop_requested:
                break

            # 精准睡至下一拍，刨去 I/O 花费的时间
            elapsed = time.time() - loop_start
            sleep_time = max(0.01, interval_sec - elapsed)
            await asyncio.sleep(sleep_time)

        self.metadata["end_time"] = datetime.now().isoformat()
        self._save_metadata()

        status_msg = "已在进程中强行中止！" if self._stop_requested else "圆满达成"
        
        return TaskResult(
            success=not self._stop_requested,
            message=f"真实环境图像采集{status_msg}！共导出并压制了 {frames_collected} 组快照！",
            data={
                "duration_seconds": self.duration,
                "frames_collected": frames_collected,
                "save_path": os.path.abspath(self.save_dir)
            },
            metrics={
                "total_images": frames_collected * len(self.sensors)
            }
        )

    def teardown(self):
        logger.info(f"Data collection task {self.task_id} successfully spun down.")

"""目标检测任务实现。"""

import asyncio
from typing import Any, Dict, Optional
import logging

from app.models.task import TaskResult, TaskStatus
from app.services.tasks.base_task import BaseTask
from app.services.tasks.registry import TaskRegistry, register_task


logger = logging.getLogger(__name__)


@register_task
class ObjectDetectionTask(BaseTask):
    """
    对 RealSense 相机帧运行目标检测的任务。

    当前为模拟实现，实际部署时将：
    1. 从 RealSense 设备获取帧
    2. 运行目标检测模型（YOLO、SSD 等）
    3. 报告检测到的物体及置信度分数
    """

    task_type = "object_detection"
    name = "目标检测"
    description = "使用深度学习模型对视频帧进行目标检测，识别和定位物体"
    category = "vision"
    requires_device = True

    # 参数校验模式
    params_schema = {
        "type": "object",
        "properties": {
            "model": {
                "type": "string",
                "enum": ["yolov8n", "yolov8s", "yolov8m", "yolov8l"],
                "default": "yolov8n",
                "description": "检测模型类型"
            },
            "confidence_threshold": {
                "type": "number",
                "minimum": 0,
                "maximum": 1,
                "default": 0.5,
                "description": "置信度阈值"
            },
            "interval_ms": {
                "type": "integer",
                "minimum": 100,
                "maximum": 5000,
                "default": 500,
                "description": "检测间隔（毫秒）"
            },
            "max_detections": {
                "type": "integer",
                "minimum": 1,
                "maximum": 100,
                "default": 20,
                "description": "每帧最大检测数量"
            },
            "classes": {
                "type": "array",
                "items": {"type": "string"},
                "default": [],
                "description": "要检测的类别列表（空表示所有类别）"
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

        # 任务参数及默认值
        self.model = params.get("model", "yolov8n")
        self.confidence_threshold = params.get("confidence_threshold", 0.5)
        self.interval_ms = params.get("interval_ms", 500)
        self.max_detections = params.get("max_detections", 20)
        self.classes = params.get("classes", [])

        # 内部状态
        self._frame_count = 0
        self._total_detections = 0
        self._model_loaded = False

    def validate(self) -> bool:
        """校验任务参数。"""
        if not self.device_id:
            raise ValueError("device_id is required for object detection")

        if self.confidence_threshold < 0 or self.confidence_threshold > 1:
            raise ValueError("confidence_threshold must be between 0 and 1")

        if self.interval_ms < 100:
            raise ValueError("interval_ms must be at least 100ms")

        return True

    def setup(self):
        """加载检测模型并初始化资源。"""
        logger.info(f"Loading {self.model} model for task {self.task_id}")

        # 模拟模型加载延迟
        # 实际部署时将加载真实模型
        import time
        time.sleep(0.5)

        self._model_loaded = True
        logger.info(f"Model loaded for task {self.task_id}")

    async def run(self) -> TaskResult:
        """执行目标检测循环。"""
        if not self._model_loaded:
            return TaskResult(
                success=False,
                message="Model not loaded",
                error="Setup did not complete successfully"
            )

        # 模拟检测循环
        # 实际部署时将获取设备帧并执行推理
        detection_results = []
        frames_processed = 0
        max_frames = self.params.get("max_frames", 100)  # 演示用，处理 N 帧后停止

        self.update_progress(
            current_step=0,
            total_steps=max_frames,
            message="开始目标检测..."
        )

        while frames_processed < max_frames:
            # 检查暂停/停止
            await self.async_check_paused()

            # 模拟帧处理
            await asyncio.sleep(self.interval_ms / 1000)

            # 模拟检测（实际部署时将运行推理）
            detections = self._simulate_detection(frames_processed)

            if detections:
                detection_results.extend(detections)
                self._total_detections += len(detections)

            frames_processed += 1
            self._frame_count = frames_processed

            # 更新进度
            percentage = (frames_processed / max_frames) * 100
            self.update_progress(
                current_step=frames_processed,
                percentage=percentage,
                message=f"已处理 {frames_processed}/{max_frames} 帧，检测到 {self._total_detections} 个物体"
            )

            # 检查是否停止
            if self._stop_requested:
                break

        # 准备结果
        final_status = "stopped" if self._stop_requested else "completed"

        return TaskResult(
            success=True if not self._stop_requested else False,
            message=f"检测{final_status}：处理了 {frames_processed} 帧",
            data={
                "frames_processed": frames_processed,
                "total_detections": self._total_detections,
                "detections": detection_results[:100],  # 限制存储结果数量
                "model": self.model,
                "confidence_threshold": self.confidence_threshold,
            },
            metrics={
                "avg_detections_per_frame": self._total_detections / max(1, frames_processed),
                "frames_per_second": 1000 / self.interval_ms,
            }
        )

    def _simulate_detection(self, frame_num: int) -> list:
        """演示用：模拟目标检测结果。"""
        import random

        # 模拟随机检测
        num_detections = random.randint(0, 3)

        if num_detections == 0:
            return []

        object_classes = ["person", "car", "bicycle", "dog", "cat", "chair", "cup"]

        detections = []
        for _ in range(num_detections):
            obj_class = random.choice(object_classes)
            confidence = random.uniform(self.confidence_threshold, 1.0)

            # 类别过滤器已设置且当前类别不在列表中时跳过
            if self.classes and obj_class not in self.classes:
                continue

            detections.append({
                "class": obj_class,
                "confidence": round(confidence, 3),
                "bbox": {
                    "x": random.randint(0, 600),
                    "y": random.randint(0, 400),
                    "width": random.randint(50, 200),
                    "height": random.randint(50, 200)
                },
                "frame": frame_num
            })

        return detections[:self.max_detections]

    def teardown(self):
        """清理资源。"""
        logger.info(f"Cleaning up object detection task {self.task_id}")
        self._model_loaded = False



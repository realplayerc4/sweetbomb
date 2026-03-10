"""Object detection task implementation."""

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
    Task for running object detection on RealSense camera frames.

    This task simulates object detection processing. In a real implementation,
    it would:
    1. Get frames from the RealSense device
    2. Run object detection model (YOLO, SSD, etc.)
    3. Report detected objects and confidence scores
    """

    task_type = "object_detection"
    name = "目标检测"
    description = "使用深度学习模型对视频帧进行目标检测，识别和定位物体"
    category = "vision"
    requires_device = True

    # Parameter schema for validation
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

        # Task parameters with defaults
        self.model = params.get("model", "yolov8n")
        self.confidence_threshold = params.get("confidence_threshold", 0.5)
        self.interval_ms = params.get("interval_ms", 500)
        self.max_detections = params.get("max_detections", 20)
        self.classes = params.get("classes", [])

        # Internal state
        self._frame_count = 0
        self._total_detections = 0
        self._model_loaded = False

    def validate(self) -> bool:
        """Validate task parameters."""
        if not self.device_id:
            raise ValueError("device_id is required for object detection")

        if self.confidence_threshold < 0 or self.confidence_threshold > 1:
            raise ValueError("confidence_threshold must be between 0 and 1")

        if self.interval_ms < 100:
            raise ValueError("interval_ms must be at least 100ms")

        return True

    def setup(self):
        """Load detection model and initialize resources."""
        logger.info(f"Loading {self.model} model for task {self.task_id}")

        # Simulate model loading delay
        # In real implementation, this would load the actual model
        import time
        time.sleep(0.5)

        self._model_loaded = True
        logger.info(f"Model loaded for task {self.task_id}")

    async def run(self) -> TaskResult:
        """Execute object detection loop."""
        if not self._model_loaded:
            return TaskResult(
                success=False,
                message="Model not loaded",
                error="Setup did not complete successfully"
            )

        # Simulated detection loop
        # In real implementation, this would get frames from the device
        detection_results = []
        frames_processed = 0
        max_frames = self.params.get("max_frames", 100)  # For demo, stop after N frames

        self.update_progress(
            current_step=0,
            total_steps=max_frames,
            message="开始目标检测..."
        )

        while frames_processed < max_frames:
            # Check for pause/stop
            await self.async_check_paused()

            # Simulate frame processing
            await asyncio.sleep(self.interval_ms / 1000)

            # Simulate detection (in real implementation, run inference here)
            detections = self._simulate_detection(frames_processed)

            if detections:
                detection_results.extend(detections)
                self._total_detections += len(detections)

            frames_processed += 1
            self._frame_count = frames_processed

            # Update progress
            percentage = (frames_processed / max_frames) * 100
            self.update_progress(
                current_step=frames_processed,
                percentage=percentage,
                message=f"已处理 {frames_processed}/{max_frames} 帧，检测到 {self._total_detections} 个物体"
            )

            # Check if stopped
            if self._stop_requested:
                break

        # Prepare results
        final_status = "stopped" if self._stop_requested else "completed"

        return TaskResult(
            success=True if not self._stop_requested else False,
            message=f"检测{final_status}：处理了 {frames_processed} 帧",
            data={
                "frames_processed": frames_processed,
                "total_detections": self._total_detections,
                "detections": detection_results[:100],  # Limit stored results
                "model": self.model,
                "confidence_threshold": self.confidence_threshold,
            },
            metrics={
                "avg_detections_per_frame": self._total_detections / max(1, frames_processed),
                "frames_per_second": 1000 / self.interval_ms,
            }
        )

    def _simulate_detection(self, frame_num: int) -> list:
        """Simulate object detection results for demo purposes."""
        import random

        # Simulate random detections
        num_detections = random.randint(0, 3)

        if num_detections == 0:
            return []

        object_classes = ["person", "car", "bicycle", "dog", "cat", "chair", "cup"]

        detections = []
        for _ in range(num_detections):
            obj_class = random.choice(object_classes)
            confidence = random.uniform(self.confidence_threshold, 1.0)

            # Skip if class filter is set and class not in list
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
        """Cleanup resources."""
        logger.info(f"Cleaning up object detection task {self.task_id}")
        self._model_loaded = False



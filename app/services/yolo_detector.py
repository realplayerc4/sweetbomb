"""非阻塞 YOLOv8 ONNX 推理服务，用于实时 RGB 和深度流检测。"""

from __future__ import annotations

import copy
import logging
import os
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple

import cv2
import numpy as np


logger = logging.getLogger(__name__)


@dataclass
class _FrameJob:
    """待推理帧任务。"""
    device_id: str
    stream_type: str
    image: np.ndarray
    frame_id: int
    capture_timestamp_ms: float
    submitted_at: float


class YoloV8Detector:
    """在 daemon 线程中运行共享 YOLO 网络，仅处理最新帧。"""

    DEFAULT_MODEL_PATH = "/home/jetson/sweetbomb/models/yolov8n/best.onnx"
    DEFAULT_LABELS = ("Depth_SweetBomb", "RGB_SweetBomb")

    def __init__(self) -> None:
        """初始化检测器，读取环境变量并启动工作线程。"""
        self.enabled = os.getenv("YOLO_ENABLED", "1").lower() not in {
            "0",
            "false",
            "no",
        }
        self.model_path = Path(
            os.getenv("YOLO_MODEL_PATH", self.DEFAULT_MODEL_PATH)
        )
        self.input_size = int(os.getenv("YOLO_INPUT_SIZE", "640"))
        self.confidence_threshold = float(os.getenv("YOLO_CONFIDENCE", "0.35"))
        self.iou_threshold = float(os.getenv("YOLO_IOU_THRESHOLD", "0.45"))
        self.max_total_fps = max(float(os.getenv("YOLO_MAX_FPS", "5")), 0.1)

        self._net = None
        self._labels = list(self.DEFAULT_LABELS)
        self._ready = False
        self._error: Optional[str] = None
        self._backend = "opencv-cpu"
        self._revision = 1
        self._status_updated_at_ms = time.time() * 1000.0

        self._condition = threading.Condition()
        self._pending: Dict[Tuple[str, str], _FrameJob] = {}
        self._last_stream_type: Optional[str] = None
        self._result_lock = threading.Lock()
        self._results: Dict[Tuple[str, str], dict] = {}
        self._running = self.enabled
        self._thread: Optional[threading.Thread] = None

        if self.enabled:
            self._thread = threading.Thread(
                target=self._worker_loop,
                daemon=True,
                name="yolov8-detection-worker",
            )
            self._thread.start()
        else:
            self._error = "YOLO detection is disabled"

    def submit(
        self,
        device_id: str,
        stream_type: str,
        image: np.ndarray,
        frame_id: int,
        capture_timestamp_ms: float,
    ) -> None:
        """提交一帧用于推理，不阻塞采集线程。"""
        if not self.enabled or image is None or image.size == 0:
            return

        normalized_stream_type = stream_type.split("-", 1)[0].lower()
        if normalized_stream_type not in {"color", "depth"}:
            return

        job = _FrameJob(
            device_id=device_id,
            stream_type=normalized_stream_type,
            image=np.ascontiguousarray(image),
            frame_id=int(frame_id),
            capture_timestamp_ms=float(capture_timestamp_ms),
            submitted_at=time.monotonic(),
        )
        with self._condition:
            self._pending[(device_id, normalized_stream_type)] = job
            self._condition.notify()

    def get_snapshot(self, device_id: str) -> dict:
        """获取检测器状态及每个流的最新结果。"""
        with self._result_lock:
            streams = {}
            for stream_type in ("color", "depth"):
                result = self._results.get((device_id, stream_type))
                if result is not None:
                    streams[stream_type] = copy.deepcopy(result)

            return {
                "device_id": device_id,
                "revision": self._revision,
                "enabled": self.enabled,
                "ready": self._ready,
                "error": self._error,
                "backend": self._backend,
                "model_name": self.model_path.name,
                "model_path": str(self.model_path),
                "labels": list(self._labels),
                "status_updated_at_ms": self._status_updated_at_ms,
                "streams": streams,
            }

    def annotate(
        self,
        device_id: str,
        stream_type: str,
        image: np.ndarray,
    ) -> np.ndarray:
        """在 RGB 帧副本上绘制最新检测框。"""
        normalized_stream_type = stream_type.split("-", 1)[0].lower()
        with self._result_lock:
            result = self._results.get((device_id, normalized_stream_type))
            if result is None:
                return image
            result = copy.deepcopy(result)

        if time.time() * 1000.0 - result["processed_at_ms"] > 2000.0:
            return image

        detections = result.get("detections", [])
        if not detections:
            return image

        annotated = image.copy()
        height, width = annotated.shape[:2]
        accent = (
            (255, 128, 46)
            if normalized_stream_type == "color"
            else (34, 211, 238)
        )
        line_width = max(1, round(width / 320))
        font_scale = max(0.35, width / 1000.0)
        font_thickness = max(1, line_width)

        for detection in detections:
            x1 = int(np.clip(detection["x1"] * width, 0, width - 1))
            y1 = int(np.clip(detection["y1"] * height, 0, height - 1))
            x2 = int(np.clip(detection["x2"] * width, 0, width - 1))
            y2 = int(np.clip(detection["y2"] * height, 0, height - 1))
            if x2 <= x1 or y2 <= y1:
                continue

            cv2.rectangle(annotated, (x1, y1), (x2, y2), accent, line_width)
            label = "{} {:.0f}%".format(
                detection["label"].replace("_", " "),
                detection["confidence"] * 100.0,
            )
            (text_width, text_height), baseline = cv2.getTextSize(
                label,
                cv2.FONT_HERSHEY_SIMPLEX,
                font_scale,
                font_thickness,
            )
            label_top = max(0, y1 - text_height - baseline - 4)
            label_bottom = min(height - 1, label_top + text_height + baseline + 4)
        label_right = min(width - 1, x1 + text_width + 8)
        cv2.rectangle(
            annotated,
            (x1, label_top),
            (label_right, label_bottom),
            accent,
            cv2.FILLED,
        )
        cv2.putText(
            annotated,
            label,
            (x1 + 4, label_bottom - baseline - 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (8, 8, 8),
            font_thickness,
            cv2.LINE_AA,
        )

        return annotated

    def clear_device(self, device_id: str) -> None:
        """清除指定设备的待处理任务和检测结果。"""
        with self._condition:
            for key in [key for key in self._pending if key[0] == device_id]:
                self._pending.pop(key, None)

        with self._result_lock:
            removed = False
            for key in [key for key in self._results if key[0] == device_id]:
                self._results.pop(key, None)
                removed = True
            if removed:
                self._revision += 1

    def stop(self) -> None:
        """停止检测器工作线程。"""
        self._running = False
        with self._condition:
            self._condition.notify_all()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def _worker_loop(self) -> None:
        """工作线程主循环：加载模型后持续消费待处理帧。"""
        try:
            self._load_model()
        except Exception as exc:
            logger.exception("YOLO model initialization failed")
            self._set_status(ready=False, error=str(exc))
            return

        minimum_period = 1.0 / self.max_total_fps
        while self._running:
            job = self._take_oldest_pending_job()
            if job is None:
                continue

            started_at = time.perf_counter()
            try:
                detections = self._infer(job.image, job.stream_type)
                inference_ms = (time.perf_counter() - started_at) * 1000.0
                self._store_result(job, detections, inference_ms)
            except Exception as exc:
                logger.exception("YOLO inference failed")
                self._set_status(ready=False, error=str(exc))
                return

            elapsed = time.perf_counter() - started_at
            remaining = minimum_period - elapsed
            if remaining > 0:
                time.sleep(remaining)

    def _take_oldest_pending_job(self) -> Optional[_FrameJob]:
        """取出最旧的待处理帧，优先交替颜色/深度流。"""
        with self._condition:
            while self._running and not self._pending:
                self._condition.wait(timeout=1.0)
            if not self._running:
                return None

            desired_stream_type = None
            if self._last_stream_type == "color":
                desired_stream_type = "depth"
            elif self._last_stream_type == "depth":
                desired_stream_type = "color"

            matching_keys = [
                key
                for key, pending_job in self._pending.items()
                if pending_job.stream_type == desired_stream_type
            ]
            if desired_stream_type and not matching_keys:
                self._condition.wait(timeout=0.01)
                matching_keys = [
                    key
                    for key, pending_job in self._pending.items()
                    if pending_job.stream_type == desired_stream_type
                ]

            candidate_keys = matching_keys or list(self._pending)
            key = min(
                candidate_keys,
                key=lambda pending_key: self._pending[pending_key].submitted_at,
            )
            job = self._pending.pop(key)
            self._last_stream_type = job.stream_type
            return job

    def _load_model(self) -> None:
        """加载 ONNX 模型并初始化后端。"""
        if not self.model_path.is_file():
            raise FileNotFoundError(f"YOLO model not found: {self.model_path}")

        net = cv2.dnn.readNetFromONNX(str(self.model_path))
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        self._net = net
        self._set_status(ready=True, error=None)
        logger.info(
            "YOLO model loaded: %s (%s, max %.1f FPS total)",
            self.model_path,
            self._backend,
            self.max_total_fps,
        )

    def _set_status(self, ready: bool, error: Optional[str]) -> None:
        """更新检测器就绪状态和错误信息。"""
        with self._result_lock:
            self._ready = ready
            self._error = error
            self._revision += 1
            self._status_updated_at_ms = time.time() * 1000.0

    def _store_result(
        self,
        job: _FrameJob,
        detections: list[dict],
        inference_ms: float,
    ) -> None:
        """保存单帧推理结果，并更新流级状态。"""
        height, width = job.image.shape[:2]
        with self._result_lock:
            self._revision += 1
            stream_revision = self._revision
            self._ready = True
            self._error = None
            self._results[(job.device_id, job.stream_type)] = {
                "revision": stream_revision,
                "stream_type": job.stream_type,
                "frame_id": job.frame_id,
                "capture_timestamp_ms": job.capture_timestamp_ms,
                "processed_at_ms": time.time() * 1000.0,
                "source_width": int(width),
                "source_height": int(height),
                "inference_ms": round(inference_ms, 1),
                "detections": detections,
            }

    def _infer(self, image: np.ndarray, stream_type: str) -> list[dict]:
        """执行单次 YOLO 推理，返回归一化检测框列表。"""
        if self._net is None:
            return []

        letterboxed, scale, pad_x, pad_y = self._letterbox(image)
        blob = cv2.dnn.blobFromImage(
            letterboxed,
            scalefactor=1.0 / 255.0,
            size=(self.input_size, self.input_size),
            mean=(0, 0, 0),
            swapRB=False,
            crop=False,
        )
        self._net.setInput(blob)
        output = self._net.forward()
        predictions = np.squeeze(output, axis=0)
        if predictions.shape[0] < predictions.shape[1]:
            predictions = predictions.T

        class_scores = predictions[:, 4:]
        class_ids = np.argmax(class_scores, axis=1)
        confidences = class_scores[
            np.arange(class_scores.shape[0]), class_ids
        ]

        expected_class_ids = self._expected_class_ids(stream_type)
        keep_mask = confidences >= self.confidence_threshold
        if expected_class_ids:
            keep_mask &= np.isin(class_ids, tuple(expected_class_ids))

        predictions = predictions[keep_mask]
        class_ids = class_ids[keep_mask]
        confidences = confidences[keep_mask]
        if predictions.size == 0:
            return []

        image_height, image_width = image.shape[:2]
        boxes_xywh = []
        candidates = []
        for prediction, class_id, confidence in zip(
            predictions, class_ids, confidences
        ):
            center_x, center_y, box_width, box_height = prediction[:4]
            x1 = (float(center_x) - float(box_width) / 2.0 - pad_x) / scale
            y1 = (float(center_y) - float(box_height) / 2.0 - pad_y) / scale
            x2 = (float(center_x) + float(box_width) / 2.0 - pad_x) / scale
            y2 = (float(center_y) + float(box_height) / 2.0 - pad_y) / scale

            x1 = float(np.clip(x1, 0, image_width - 1))
            y1 = float(np.clip(y1, 0, image_height - 1))
            x2 = float(np.clip(x2, 0, image_width - 1))
            y2 = float(np.clip(y2, 0, image_height - 1))
            width = x2 - x1
            height = y2 - y1
            if width <= 1 or height <= 1:
                continue

            boxes_xywh.append([x1, y1, width, height])
            candidates.append(
                {
                    "class_id": int(class_id),
                    "label": self._label_for(int(class_id)),
                    "confidence": round(float(confidence), 4),
                    "x1": x1 / image_width,
                    "y1": y1 / image_height,
                    "x2": x2 / image_width,
                    "y2": y2 / image_height,
                }
            )

        if not boxes_xywh:
            return []

        selected_indices = self._class_aware_nms(
            boxes_xywh,
            candidates,
        )
        return [candidates[index] for index in selected_indices]

    def _class_aware_nms(self, boxes_xywh: list, candidates: list[dict]) -> list[int]:
        """按类别执行 NMS，避免不同类别相互抑制。"""
        selected = []
        class_ids = sorted({candidate["class_id"] for candidate in candidates})
        for class_id in class_ids:
            candidate_indices = [
                index
                for index, candidate in enumerate(candidates)
                if candidate["class_id"] == class_id
            ]
            class_boxes = [boxes_xywh[index] for index in candidate_indices]
            class_scores = [
                candidates[index]["confidence"] for index in candidate_indices
            ]
            kept = cv2.dnn.NMSBoxes(
                class_boxes,
                class_scores,
                self.confidence_threshold,
                self.iou_threshold,
            )
            if len(kept) == 0:
                continue
            selected.extend(candidate_indices[int(index)] for index in np.array(kept).flatten())

        selected.sort(key=lambda index: candidates[index]["confidence"], reverse=True)
        return selected

    def _expected_class_ids(self, stream_type: str) -> set[int]:
        """根据流类型返回期望的类别 ID 集合。"""
        token = "rgb" if stream_type == "color" else "depth"
        return {
            index
            for index, label in enumerate(self._labels)
            if token in label.lower()
        }

    def _label_for(self, class_id: int) -> str:
        """根据类别 ID 返回标签，未知 ID 返回默认名称。"""
        if 0 <= class_id < len(self._labels):
            return self._labels[class_id]
        return f"class_{class_id}"

    def _letterbox(self, image: np.ndarray) -> tuple[np.ndarray, float, float, float]:
        """将图像缩放并填充到模型输入尺寸，保持长宽比。"""
        image_height, image_width = image.shape[:2]
        scale = min(
            self.input_size / image_width,
            self.input_size / image_height,
        )
        resized_width = int(round(image_width * scale))
        resized_height = int(round(image_height * scale))
        resized = cv2.resize(
            image,
            (resized_width, resized_height),
            interpolation=cv2.INTER_LINEAR,
        )

        pad_width = self.input_size - resized_width
        pad_height = self.input_size - resized_height
        left = int(round(pad_width / 2.0 - 0.1))
        right = int(round(pad_width / 2.0 + 0.1))
        top = int(round(pad_height / 2.0 - 0.1))
        bottom = int(round(pad_height / 2.0 + 0.1))
        letterboxed = cv2.copyMakeBorder(
            resized,
            top,
            bottom,
            left,
            right,
            cv2.BORDER_CONSTANT,
            value=(114, 114, 114),
        )
        return letterboxed, scale, float(left), float(top)

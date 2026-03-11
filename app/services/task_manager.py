"""任务生命周期管理器。

负责任务的创建、启动、暂停、恢复、停止和查询。
支持最多 4 个并发任务，通过 Socket.IO 广播任务事件，追踪任务进度。
"""

import asyncio
import uuid
from datetime import datetime
from typing import Dict, List, Optional, Callable, Any
import logging
from concurrent.futures import ThreadPoolExecutor

import socketio

from app.models.task import (
    TaskConfig,
    TaskCreateRequest,
    TaskInfo,
    TaskProgress,
    TaskResult,
    TaskStatus,
    TaskEvent,
    TaskListResponse,
)
from app.services.tasks.base_task import BaseTask, TaskStopException
from app.services.tasks.registry import TaskRegistry


logger = logging.getLogger(__name__)


# 最大并发运行任务数
MAX_CONCURRENT_TASKS = 4


class TaskManager:
    """
    任务管理器（单例模式）。

    功能：
    - 创建、启动、暂停、恢复、停止任务
    - 并发任务执行（最多 4 个任务）
    - Socket.IO 事件广播
    - 进度追踪
    """

    _instance: Optional["TaskManager"] = None

    def __new__(cls, sio: Optional[socketio.AsyncServer] = None, rs_manager=None) -> "TaskManager":
        """单例模式。"""
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self, sio: Optional[socketio.AsyncServer] = None, rs_manager=None):
        """初始化任务管理器。

        Args:
            sio: Socket.IO 服务器实例
            rs_manager: RealSense 管理器实例
        """
        if self._initialized:
            if sio is not None:
                self._sio = sio
            if rs_manager is not None:
                self._rs_manager = rs_manager
            return

        self._sio = sio
        self._rs_manager = rs_manager
        self._tasks: Dict[str, TaskInfo] = {}  # task_id -> TaskInfo
        self._task_instances: Dict[str, BaseTask] = {}  # task_id -> BaseTask
        self._running_tasks: Dict[str, asyncio.Task] = {}  # task_id -> asyncio.Task
        self._semaphore = asyncio.Semaphore(MAX_CONCURRENT_TASKS)
        self._registry = TaskRegistry.get_instance()
        self._executor = ThreadPoolExecutor(max_workers=2)
        self._initialized = True

        logger.info("TaskManager initialized")

    @classmethod
    def get_instance(cls, sio: Optional[socketio.AsyncServer] = None, rs_manager=None) -> "TaskManager":
        """获取单例实例。"""
        return cls(sio, rs_manager)

    def set_socketio(self, sio: socketio.AsyncServer):
        """设置 Socket.IO 服务器实例。"""
        self._sio = sio

    # --- Socket.IO 事件广播 ---

    async def _emit_event(self, event_type: str, task_info: TaskInfo):
        """通过 Socket.IO 发送任务事件。

        Args:
            event_type: 事件类型（created、started、paused、resumed、stopped、completed、failed）
            task_info: 任务信息
        """
        if self._sio is None:
            return

        event = TaskEvent(
            event_type=event_type,
            task_id=task_info.task_id,
            task_type=task_info.task_type,
            status=task_info.status,
            progress=task_info.progress,
            result=task_info.result,
        )
        try:
            await self._sio.emit("task_event", event.model_dump(mode="json"))
            logger.debug(f"Emitted {event_type} event for task {task_info.task_id}")
        except Exception as e:
            logger.error(f"Failed to emit event: {e}")

    # --- 任务创建 ---

    def create_task(self, request: TaskCreateRequest) -> TaskInfo:
        """
        创建新任务。

        Args:
            request: 任务创建请求

        Returns:
            TaskInfo: 创建的任务信息

        Raises:
            ValueError: 任务类型未注册或验证失败
        """
        task_class = self._registry.get(request.task_type)
        if task_class is None:
            raise ValueError(f"Unknown task type: {request.task_type}")

        # 检查设备依赖
        if task_class.requires_device and not request.device_id:
            raise ValueError(f"Task type '{request.task_type}' requires a device_id")

        # 生成任务 ID
        task_id = str(uuid.uuid4())

        # 合并配置
        config = request.config or TaskConfig(
            device_id=request.device_id,
            params=request.params
        )
        if request.device_id:
            config.device_id = request.device_id

        # 创建任务信息
        task_info = TaskInfo(
            task_id=task_id,
            task_type=request.task_type,
            device_id=request.device_id,
            config=config,
            params=request.params,
            status=TaskStatus.PENDING,
            created_at=datetime.now(),
        )

        # 创建任务实例
        task_instance = task_class(
            task_id=task_id,
            config=config,
            params=request.params,
            device_id=request.device_id
        )

        # 注入硬件依赖（如果实例支持）
        if hasattr(task_instance, "rs_manager"):
            task_instance.rs_manager = self._rs_manager

        # 存储任务
        self._tasks[task_id] = task_info
        self._task_instances[task_id] = task_instance

        logger.info(f"Created task {task_id} of type {request.task_type}")
        return task_info

    # --- 任务控制 ---

    async def start_task(self, task_id: str) -> TaskInfo:
        """
        启动待命任务。

        Args:
            task_id: 要启动的任务 ID

        Returns:
            TaskInfo: 更新后的任务信息

        Raises:
            ValueError: 任务不存在或不在 PENDING 状态
        """
        if task_id not in self._tasks:
            raise ValueError(f"Task not found: {task_id}")

        task_info = self._tasks[task_id]
        if task_info.status != TaskStatus.PENDING:
            raise ValueError(f"Task must be in PENDING status to start. Current: {task_info.status}")

        task_instance = self._task_instances[task_id]

        # 设置回调函数
        def on_progress(progress: TaskProgress):
            task_info.progress = progress

        def on_status_change(status: TaskStatus):
            task_info.status = status

        task_instance.set_callbacks(on_progress, on_status_change)

        # 启动异步执行
        async def run_with_semaphore():
            async with self._semaphore:
                task_info.started_at = datetime.now()
                await self._emit_event("started", task_info)

                try:
                    result = await task_instance.execute()
                    task_info.result = result
                    task_info.completed_at = datetime.now()

                    event_type = "completed" if result.success else "failed"
                    await self._emit_event(event_type, task_info)

                except Exception as e:
                    logger.error(f"Task {task_id} failed: {e}")
                    task_info.status = TaskStatus.FAILED
                    task_info.result = TaskResult(
                        success=False,
                        error=str(e),
                        message="Task execution failed"
                    )
                    task_info.completed_at = datetime.now()
                    await self._emit_event("failed", task_info)

                finally:
                    self._running_tasks.pop(task_id, None)

        # 创建并存储异步任务
        async_task = asyncio.create_task(run_with_semaphore())
        self._running_tasks[task_id] = async_task

        return task_info

    async def pause_task(self, task_id: str) -> TaskInfo:
        """暂停运行中的任务。"""
        if task_id not in self._tasks:
            raise ValueError(f"Task not found: {task_id}")

        task_info = self._tasks[task_id]
        if task_info.status != TaskStatus.RUNNING:
            raise ValueError(f"TaskTask must be RUNNING to pause. Current: {task_info.status}")

        task_instance = self._task_instances[task_id]
        task_instance.pause()
        task_info.status = TaskStatus.PAUSED

        await self._emit_event("paused", task_info)
        return task_info

    async def resume_task(self, task_id: str) -> TaskInfo:
        """恢复暂停的任务。"""
        if task_id not in self._tasks:
            raise ValueError(f"Task not found: {task_id}")

        task_info = self._tasks[task_id]
        if task_info.status != TaskStatus.PAUSED:
            raise ValueError(f"Task must be PAUSED to resume. Current: {task_info.status}")

        task_instance = self._task_instances[task_id]
        task_instance.resume()
        task_info.status = TaskStatus.RUNNING

        await self._emit_event("resumed", task_info)
        return task_info

    async def stop_task(self, task_id: str) -> TaskInfo:
        """停止运行中或暂停的任务。"""
        if task_id not in self._tasks:
            raise ValueError(f"Task not found: {task_id}")

        task_info = self._tasks[task_id]
        if task_info.status not in [TaskStatus.RUNNING, TaskStatus.PAUSED]:
            raise ValueError(f"Task must be RUNNING or PAUSED to stop. Current: {task_info.status}")

        task_instance = self._task_instances[task_id]
        task_instance.stop()
        task_info.status = TaskStatus.STOPPED
        task_info.completed_at = datetime.now()

        # 取消异步任务（如果存在）
        if task_id in self._running_tasks:
            self._running_tasks[task_id].cancel()
            del self._running_tasks[task_id]

        await self._emit_event("stopped", task_info)
        return task_info

    def delete_task(self, task_id: str) -> bool:
        """删除任务（仅非运行中任务）。"""
        if task_id not in self._tasks:
            raise ValueError(f"Task not found: {task_id}")

        task_info = self._tasks[task_id]
        if task_info.status == TaskStatus.RUNNING:
            raise ValueError("Cannot delete a running task. Stop it first.")

        del self._tasks[task_id]
        self._task_instances.pop(task_id, None)
        self._running_tasks.pop(task_id, None)

        logger.info(f"Deleted task {task_id}")
        return True

    # --- 任务查询 ---

    def get_task(self, task_id: str) -> Optional[TaskInfo]:
        """根据 ID 获取任务信息。"""
        return self._tasks.get(task_id)

    def list_tasks(
        self,
        status: Optional[TaskStatus] = None,
        device_id: Optional[str] = None,
        task_type: Optional[str] = None
    ) -> TaskListResponse:
        """列查任务，支持可选过滤。"""
        tasks = list(self._tasks.values())

        # 应用过滤器
        if status:
            tasks = [t for t in tasks if t.status == status]
        if device_id:
            tasks = [t for t in tasks if t.device_id == device_id]
        if task_type:
            tasks = [t for t in tasks if t.task_type == task_type]

        running_count = sum(1 for t in self._tasks.values() if t.status == TaskStatus.RUNNING)
        pending_count = sum(1 for t in self._tasks.values() if t.status == TaskStatus.PENDING)

        return TaskListResponse(
            tasks=tasks,
            total=len(tasks),
            running_count=running_count,
            pending_count=pending_count
        )

    def get_running_count(self) -> int:
        """获取当前运行中的任务数量。"""
        return sum(1 for t in self._tasks.values() if t.status == TaskStatus.RUNNING)

    def can_start_more_tasks(self) -> bool:
        """检查是否可以启动更多任务。"""
        return self.get_running_count() < MAX_CONCURRENT_TASKS

    # --- 清理 ---

    async def shutdown(self):
        """停止所有运行中的任务并清理资源。"""
        logger.info("Shutting down TaskManager...")

        # 停止所有运行中的任务
        for task_id, task_info in list(self._tasks.items()):
            if task_info.status in [TaskStatus.RUNNING, TaskStatus.PAUSED]:
                try:
                    await self.stop_task(task_id)
                except Exception as e:
                    logger.error(f"Failed to stop task {task_id}: {e}")

        # 取消所有异步任务
        for async_task in self._running_tasks.values():
            async_task.cancel()

        self._executor.shutdown(wait=False)
        logger.info("TaskManager shutdown complete")

"""Task manager for lifecycle management of tasks."""

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


# Maximum concurrent running tasks
MAX_CONCURRENT_TASKS = 4


class TaskManager:
    """
    Singleton manager for task lifecycle.

    Features:
    - Create, start, pause, resume, stop tasks
    - Concurrent task execution (max 4 tasks)
    - Socket.IO event broadcasting
    - Progress tracking
    """

    _instance: Optional["TaskManager"] = None

    def __new__(cls, sio: Optional[socketio.AsyncServer] = None) -> "TaskManager":
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self, sio: Optional[socketio.AsyncServer] = None):
        if self._initialized:
            if sio is not None:
                self._sio = sio
            return

        self._sio = sio
        self._tasks: Dict[str, TaskInfo] = {}
        self._task_instances: Dict[str, BaseTask] = {}
        self._running_tasks: Dict[str, asyncio.Task] = {}
        self._semaphore = asyncio.Semaphore(MAX_CONCURRENT_TASKS)
        self._registry = TaskRegistry.get_instance()
        self._executor = ThreadPoolExecutor(max_workers=2)
        self._initialized = True

        logger.info("TaskManager initialized")

    @classmethod
    def get_instance(cls, sio: Optional[socketio.AsyncServer] = None) -> "TaskManager":
        """Get the singleton TaskManager instance."""
        return cls(sio)

    def set_socketio(self, sio: socketio.AsyncServer):
        """Set the Socket.IO server instance."""
        self._sio = sio

    # --- Socket.IO Event Broadcasting ---

    async def _emit_event(self, event_type: str, task_info: TaskInfo):
        """Emit a task event via Socket.IO."""
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

    # --- Task Creation ---

    def create_task(self, request: TaskCreateRequest) -> TaskInfo:
        """
        Create a new task.

        Args:
            request: Task creation request

        Returns:
            TaskInfo for the created task

        Raises:
            ValueError: If task type is not registered or validation fails
        """
        task_class = self._registry.get(request.task_type)
        if task_class is None:
            raise ValueError(f"Unknown task type: {request.task_type}")

        # Check device requirement
        if task_class.requires_device and not request.device_id:
            raise ValueError(f"Task type '{request.task_type}' requires a device_id")

        # Generate task ID
        task_id = str(uuid.uuid4())

        # Merge config
        config = request.config or TaskConfig(
            device_id=request.device_id,
            params=request.params
        )
        if request.device_id:
            config.device_id = request.device_id

        # Create task info
        task_info = TaskInfo(
            task_id=task_id,
            task_type=request.task_type,
            device_id=request.device_id,
            config=config,
            params=request.params,
            status=TaskStatus.PENDING,
            created_at=datetime.now(),
        )

        # Create task instance
        task_instance = task_class(
            task_id=task_id,
            config=config,
            params=request.params,
            device_id=request.device_id
        )

        # Store task
        self._tasks[task_id] = task_info
        self._task_instances[task_id] = task_instance

        logger.info(f"Created task {task_id} of type {request.task_type}")
        return task_info

    # --- Task Control ---

    async def start_task(self, task_id: str) -> TaskInfo:
        """
        Start a pending task.

        Args:
            task_id: The task ID to start

        Returns:
            Updated TaskInfo

        Raises:
            ValueError: If task not found or not in PENDING status
        """
        if task_id not in self._tasks:
            raise ValueError(f"Task not found: {task_id}")

        task_info = self._tasks[task_id]
        if task_info.status != TaskStatus.PENDING:
            raise ValueError(f"Task must be in PENDING status to start. Current: {task_info.status}")

        task_instance = self._task_instances[task_id]

        # Set up callbacks
        def on_progress(progress: TaskProgress):
            task_info.progress = progress

        def on_status_change(status: TaskStatus):
            task_info.status = status

        task_instance.set_callbacks(on_progress, on_status_change)

        # Start async execution
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

        # Create and store the task
        async_task = asyncio.create_task(run_with_semaphore())
        self._running_tasks[task_id] = async_task

        return task_info

    async def pause_task(self, task_id: str) -> TaskInfo:
        """Pause a running task."""
        if task_id not in self._tasks:
            raise ValueError(f"Task not found: {task_id}")

        task_info = self._tasks[task_id]
        if task_info.status != TaskStatus.RUNNING:
            raise ValueError(f"Task must be RUNNING to pause. Current: {task_info.status}")

        task_instance = self._task_instances[task_id]
        task_instance.pause()
        task_info.status = TaskStatus.PAUSED

        await self._emit_event("paused", task_info)
        return task_info

    async def resume_task(self, task_id: str) -> TaskInfo:
        """Resume a paused task."""
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
        """Stop a running or paused task."""
        if task_id not in self._tasks:
            raise ValueError(f"Task not found: {task_id}")

        task_info = self._tasks[task_id]
        if task_info.status not in [TaskStatus.RUNNING, TaskStatus.PAUSED]:
            raise ValueError(f"Task must be RUNNING or PAUSED to stop. Current: {task_info.status}")

        task_instance = self._task_instances[task_id]
        task_instance.stop()
        task_info.status = TaskStatus.STOPPED
        task_info.completed_at = datetime.now()

        # Cancel the async task if it exists
        if task_id in self._running_tasks:
            self._running_tasks[task_id].cancel()
            del self._running_tasks[task_id]

        await self._emit_event("stopped", task_info)
        return task_info

    def delete_task(self, task_id: str) -> bool:
        """Delete a task (only if not running)."""
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

    # --- Task Queries ---

    def get_task(self, task_id: str) -> Optional[TaskInfo]:
        """Get task info by ID."""
        return self._tasks.get(task_id)

    def list_tasks(
        self,
        status: Optional[TaskStatus] = None,
        device_id: Optional[str] = None,
        task_type: Optional[str] = None
    ) -> TaskListResponse:
        """List tasks with optional filtering."""
        tasks = list(self._tasks.values())

        # Apply filters
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
        """Get count of currently running tasks."""
        return sum(1 for t in self._tasks.values() if t.status == TaskStatus.RUNNING)

    def can_start_more_tasks(self) -> bool:
        """Check if more tasks can be started."""
        return self.get_running_count() < MAX_CONCURRENT_TASKS

    # --- Cleanup ---

    async def shutdown(self):
        """Stop all running tasks and cleanup."""
        logger.info("Shutting down TaskManager...")

        # Stop all running tasks
        for task_id, task_info in list(self._tasks.items()):
            if task_info.status in [TaskStatus.RUNNING, TaskStatus.PAUSED]:
                try:
                    await self.stop_task(task_id)
                except Exception as e:
                    logger.error(f"Failed to stop task {task_id}: {e}")

        # Cancel all async tasks
        for async_task in self._running_tasks.values():
            async_task.cancel()

        self._executor.shutdown(wait=False)
        logger.info("TaskManager shutdown complete")

"""Base class for all tasks in the system."""

from abc import ABC, abstractmethod
from typing import Any, Callable, Dict, Optional
import asyncio
import logging
from datetime import datetime

from app.models.task import TaskConfig, TaskProgress, TaskResult, TaskStatus


logger = logging.getLogger(__name__)


class BaseTask(ABC):
    """
    Abstract base class for all tasks.

    Lifecycle:
        validate() -> setup() -> run() -> teardown()

    State transitions:
        PENDING -> RUNNING -> COMPLETED
                 |    ↓
                 |  PAUSED -> RUNNING
                 |    ↓
                 └→ STOPPED/FAILED/CANCELLED
    """

    # Task type identifier (must be overridden in subclass)
    task_type: str = "base_task"

    # Human-readable name
    name: str = "Base Task"

    # Description of what this task does
    description: str = "Base task class"

    # Category for UI grouping
    category: str = "general"

    # Whether this task requires a device
    requires_device: bool = True

    # JSON schema for parameters validation
    params_schema: Dict[str, Any] = {}

    def __init__(
        self,
        task_id: str,
        config: TaskConfig,
        params: Dict[str, Any],
        device_id: Optional[str] = None
    ):
        self.task_id = task_id
        self.config = config
        self.params = params
        self.device_id = device_id

        self._status: TaskStatus = TaskStatus.PENDING
        self._progress: TaskProgress = TaskProgress()
        self._result: Optional[TaskResult] = None
        self._error: Optional[str] = None

        self._started_at: Optional[datetime] = None
        self._completed_at: Optional[datetime] = None

        self._pause_event = asyncio.Event()
        self._pause_event.set()  # Not paused by default
        self._stop_requested = False

        # Callbacks for progress updates
        self._on_progress: Optional[Callable[[TaskProgress], None]] = None
        self._on_status_change: Optional[Callable[[TaskStatus], None]] = None

    @property
    def status(self) -> TaskStatus:
        return self._status

    @property
    def progress(self) -> TaskProgress:
        return self._progress

    @property
    def result(self) -> Optional[TaskResult]:
        return self._result

    def set_callbacks(
        self,
        on_progress: Optional[Callable[[TaskProgress], None]] = None,
        on_status_change: Optional[Callable[[TaskStatus], None]] = None
    ):
        """Set callback functions for progress and status updates."""
        self._on_progress = on_progress
        self._on_status_change = on_status_change

    def _set_status(self, status: TaskStatus):
        """Update task status and notify callback."""
        self._status = status
        if self._on_status_change:
            self._on_status_change(status)

    def update_progress(
        self,
        current_step: Optional[int] = None,
        total_steps: Optional[int] = None,
        percentage: Optional[float] = None,
        message: str = "",
        estimated_remaining: Optional[float] = None
    ):
        """Update task progress and notify callback."""
        if current_step is not None:
            self._progress.current_step = current_step
        if total_steps is not None:
            self._progress.total_steps = total_steps
        if percentage is not None:
            self._progress.percentage = percentage
        elif self._progress.total_steps > 0:
            self._progress.percentage = (self._progress.current_step / self._progress.total_steps) * 100

        self._progress.message = message
        self._progress.estimated_remaining_seconds = estimated_remaining

        if self._started_at:
            elapsed = (datetime.now() - self._started_at).total_seconds()
            self._progress.elapsed_seconds = elapsed

        if self._on_progress:
            self._on_progress(self._progress)

    def check_paused(self):
        """Check if task is paused and wait if so. Also checks for stop request."""
        if self._stop_requested:
            raise TaskStopException()
        self._pause_event.wait()  # Will block if paused

    async def async_check_paused(self):
        """Async version of check_paused."""
        if self._stop_requested:
            raise TaskStopException()
        await self._pause_event.wait()

    def pause(self):
        """Pause the task."""
        if self._status == TaskStatus.RUNNING:
            self._pause_event.clear()
            self._set_status(TaskStatus.PAUSED)
            logger.info(f"Task {self.task_id} paused")

    def resume(self):
        """Resume a paused task."""
        if self._status == TaskStatus.PAUSED:
            self._pause_event.set()
            self._set_status(TaskStatus.RUNNING)
            logger.info(f"Task {self.task_id} resumed")

    def stop(self):
        """Request task to stop."""
        self._stop_requested = True
        self._pause_event.set()  # Unblock if paused
        logger.info(f"Task {self.task_id} stop requested")

    # --- Abstract methods that must be implemented by subclasses ---

    @abstractmethod
    def validate(self) -> bool:
        """
        Validate task parameters and configuration.
        Called before setup() to ensure task can be executed.

        Returns:
            True if validation passes, False otherwise

        Raises:
            ValueError: If validation fails with specific error message
        """
        pass

    def setup(self):
        """
        Setup task before running.
        Override this method to perform initialization like:
        - Loading models
        - Connecting to resources
        - Allocating memory

        This method runs synchronously before the async run() method.
        """
        pass

    @abstractmethod
    async def run(self) -> TaskResult:
        """
        Execute the main task logic.
        This is an async method that should:
        - Periodically call check_paused() or async_check_paused()
        - Update progress using update_progress()
        - Return a TaskResult when complete

        Returns:
            TaskResult containing the outcome of the task
        """
        pass

    def teardown(self):
        """
        Cleanup after task completion.
        Override this method to perform cleanup like:
        - Releasing resources
        - Saving results
        - Closing connections

        This method runs regardless of task success or failure.
        """
        pass

    # --- Main execution method ---

    async def execute(self) -> TaskResult:
        """
        Execute the full task lifecycle.
        This method should not be overridden.
        """
        try:
            # Validate
            if not self.validate():
                self._result = TaskResult(
                    success=False,
                    message="Validation failed",
                    error="Task validation returned False"
                )
                self._set_status(TaskStatus.FAILED)
                return self._result

            # Setup
            try:
                self.setup()
            except Exception as e:
                logger.error(f"Task {self.task_id} setup failed: {e}")
                self._result = TaskResult(
                    success=False,
                    message="Setup failed",
                    error=str(e)
                )
                self._set_status(TaskStatus.FAILED)
                return self._result

            # Run
            self._started_at = datetime.now()
            self._set_status(TaskStatus.RUNNING)

            try:
                self._result = await self.run()
                if self._stop_requested:
                    self._set_status(TaskStatus.STOPPED)
                else:
                    self._set_status(TaskStatus.COMPLETED)
            except TaskStopException:
                self._result = TaskResult(
                    success=False,
                    message="Task was stopped",
                    error="Task stopped by user"
                )
                self._set_status(TaskStatus.STOPPED)
            except Exception as e:
                logger.error(f"Task {self.task_id} run failed: {e}")
                self._result = TaskResult(
                    success=False,
                    message="Execution failed",
                    error=str(e)
                )
                self._set_status(TaskStatus.FAILED)

            return self._result

        finally:
            self._completed_at = datetime.now()
            try:
                self.teardown()
            except Exception as e:
                logger.error(f"Task {self.task_id} teardown failed: {e}")


class TaskStopException(Exception):
    """Raised when a task is requested to stop."""
    pass

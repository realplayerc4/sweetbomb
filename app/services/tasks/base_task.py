"""系统中所有任务的基类。"""

from abc import ABC, abstractmethod
from typing import Any, Callable, Dict, Optional
import asyncio
import logging
from datetime import datetime

from app.models.task import TaskConfig, TaskProgress, TaskResult, TaskStatus


logger = logging.getLogger(__name__)


class BaseTask(ABC):
    """
    所有任务的抽象基类。

    生命周期:
        validate() -> setup() -> run() -> teardown()

    状态转换:
        PENDING -> RUNNING -> COMPLETED
                 |    ↓
                 |  PAUSED -> RUNNING
                 |    ↓
                 └→ STOPPED/FAILED/CANCELLED
    """

    task_type: str = "base_task"
    name: str = "Base Task"
    description: str = "Base task class"
    category: str = "general"
    requires_device: bool = True
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
        self._pause_event.set()
        self._stop_requested = False

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
        """设置进度和状态更新回调函数。"""
        self._on_progress = on_progress
        self._on_status_change = on_status_change

    def _set_status(self, status: TaskStatus):
        """更新任务状态并通知回调。"""
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
        """更新任务进度并通知回调。"""
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
        """检查任务是否暂停并等待。同时检查停止请求。"""
        if self._stop_requested:
            raise TaskStopException()
        self._pause_event.wait()  # 暂停时会阻塞

    async def async_check_paused(self):
        """check_paused 的异步版本。"""
        if self._stop_requested:
            raise TaskStopException()
        await self._pause_event.wait()

    def pause(self):
        """暂停任务。"""
        if self._status == TaskStatus.RUNNING:
            self._pause_event.clear()
            self._set_status(TaskStatus.PAUSED)
            logger.info(f"Task {self.task_id} paused")

    def resume(self):
        """恢复暂停的任务。"""
        if self._status == TaskStatus.PAUSED:
            self._pause_event.set()
            self._set_status(TaskStatus.RUNNING)
            logger.info(f"Task {self.task_id} resumed")

    def stop(self):
        """请求任务停止。"""
        self._stop_requested = True
        self._pause_event.set()  # 解除暂停阻塞
        logger.info(f"Task {self.task_id} stop requested")

    # --- 子类必须实现的抽象方法 ---

    @abstractmethod
    def validate(self) -> bool:
        """
        校验任务参数和配置。
        在 setup() 之前调用，确保任务可执行。

        返回:
            校验通过返回 True，否则返回 False

        异常:
            ValueError: 校验失败时抛出具体错误信息
        """
        pass

    def setup(self):
        """
        运行前准备任务。
        重写此方法以执行初始化操作，例如:
        - 加载模型
        - 连接资源
        - 分配内存

        此方法在异步 run() 之前同步执行。
        """
        pass

    @abstractmethod
    async def run(self) -> TaskResult:
        """
        执行主要任务逻辑。
        此异步方法应当:
        - 定期调用 check_paused() 或 async_check_paused()
        - 使用 update_progress() 更新进度
        - 完成后返回 TaskResult

        返回:
            包含任务结果的 TaskResult
        """
        pass

    def teardown(self):
        """
        任务完成后清理。
        重写此方法以执行清理操作，例如:
        - 释放资源
        - 保存结果
        - 关闭连接

        此方法在任务成功或失败时都会执行。
        """
        pass

    # --- 主执行方法 ---

    async def execute(self) -> TaskResult:
        """
        执行完整任务生命周期。
        此方法不应被重写。
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
    """当任务被请求停止时抛出。"""
    pass

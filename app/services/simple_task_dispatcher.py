"""简单任务分发器。

替代复杂的行为树引擎，直接发送任务指令给下位机。
上位机只负责任务触发和状态监控，下位机自主执行具体动作。
"""

import asyncio
import logging
from typing import Any, Dict, Optional, Callable
from dataclasses import dataclass
from datetime import datetime
import json

logger = logging.getLogger(__name__)


@dataclass
class TaskConfig:
    """任务配置"""
    task_type: str  # "sugar_harvest" 或 "push_mode"
    target_volume: float = 30.0  # 目标体积 (L)
    max_cycles: int = 10  # 最大循环次数
    navigation_point: Optional[tuple] = None  # 导航点坐标 (x, y)
    extra_params: Dict[str, Any] = None  # 额外参数


@dataclass
class TaskStatus:
    """任务状态"""
    task_id: str
    task_type: str
    status: str  # "idle", "running", "completed", "error", "stopped"
    current_step: str = ""  # 当前步骤
    progress: int = 0  # 进度 0-100
    message: str = ""  # 状态消息
    started_at: Optional[datetime] = None
    completed_at: Optional[datetime] = None
    error_info: Optional[str] = None


class SimpleTaskDispatcher:
    """简单任务分发器。

    核心职责：
    1. 接收任务请求，发送给下位机
    2. 监听下位机状态更新
    3. 管理任务生命周期

    不处理复杂的行为逻辑，由下位机自主执行。
    """

    def __init__(self, robot_controller, socket_io=None):
        """初始化任务分发器。

        Args:
            robot_controller: 机器人控制器，用于与下位机通信
            socket_io: Socket.IO 实例，用于广播状态更新
        """
        self.robot_controller = robot_controller
        self.socket_io = socket_io

        # 任务存储
        self._tasks: Dict[str, TaskStatus] = {}
        self._current_task_id: Optional[str] = None

        # 状态回调
        self._status_callbacks: list[Callable] = []

        # 运行状态
        self._running = False
        self._monitor_task: Optional[asyncio.Task] = None

        logger.info("SimpleTaskDispatcher initialized")

    async def start_task(self, config: TaskConfig) -> str:
        """启动新任务。

        Args:
            config: 任务配置

        Returns:
            task_id: 任务ID
        """
        # 生成任务ID
        import uuid
        task_id = f"task_{uuid.uuid4().hex[:8]}"

        # 如果有正在运行的任务，先停止
        if self._current_task_id and self._tasks.get(self._current_task_id):
            current_status = self._tasks[self._current_task_id].status
            if current_status == "running":
                logger.warning(f"Stopping current task {self._current_task_id} before starting new one")
                await self.stop_task(self._current_task_id)

        # 创建任务状态
        task_status = TaskStatus(
            task_id=task_id,
            task_type=config.task_type,
            status="running",
            current_step="等待下位机确认",
            progress=0,
            message="任务已启动，等待下位机响应",
            started_at=datetime.now()
        )
        self._tasks[task_id] = task_status
        self._current_task_id = task_id

        # 发送任务给下位机
        try:
            task_data = {
                "task_id": task_id,
                "task_type": config.task_type,
                "params": {
                    "target_volume": config.target_volume,
                    "max_cycles": config.max_cycles,
                    "navigation_point": config.navigation_point,
                    **(config.extra_params or {})
                },
                "timestamp": datetime.now().isoformat()
            }

            # 通过机器人控制器发送给下位机
            await self._send_task_to_lower_machine(task_data)

            logger.info(f"Task {task_id} ({config.task_type}) started and sent to lower machine")

            # 广播任务启动事件
            await self._broadcast_event("task_started", task_status)

        except Exception as e:
            logger.error(f"Failed to start task {task_id}: {e}")
            task_status.status = "error"
            task_status.error_info = str(e)
            task_status.completed_at = datetime.now()
            raise

        return task_id

    async def stop_task(self, task_id: str) -> bool:
        """停止指定任务。

        Args:
            task_id: 任务ID

        Returns:
            bool: 是否成功停止
        """
        if task_id not in self._tasks:
            logger.warning(f"Task {task_id} not found")
            return False

        task = self._tasks[task_id]
        if task.status not in ["running"]:
            logger.info(f"Task {task_id} is already {task.status}")
            return True

        try:
            # 发送停止命令给下位机
            stop_command = {
                "command": "stop_task",
                "task_id": task_id,
                "timestamp": datetime.now().isoformat()
            }
            await self._send_command_to_lower_machine(stop_command)

            task.status = "stopped"
            task.message = "任务已被手动停止"
            task.completed_at = datetime.now()

            logger.info(f"Task {task_id} stopped")

            # 广播任务停止事件
            await self._broadcast_event("task_stopped", task)

            return True

        except Exception as e:
            logger.error(f"Failed to stop task {task_id}: {e}")
            return False

    async def _send_task_to_lower_machine(self, task_data: dict):
        """发送任务给下位机。

        实际实现需要通过串口/网络发送给下位机。
        """
        # TODO: 实现实际的发送逻辑
        # 例如：通过串口发送 JSON 数据
        # await self.robot_controller.send_task(task_data)

        logger.debug(f"Sending task to lower machine: {task_data}")

        # 模拟发送（实际实现时替换）
        await asyncio.sleep(0.1)

    async def _send_command_to_lower_machine(self, command: dict):
        """发送命令给下位机。"""
        logger.debug(f"Sending command to lower machine: {command}")
        await asyncio.sleep(0.1)  # 模拟发送

    async def _broadcast_event(self, event_type: str, task: TaskStatus):
        """广播任务事件。"""
        event_data = {
            "event_type": event_type,
            "task_id": task.task_id,
            "task_type": task.task_type,
            "status": task.status,
            "current_step": task.current_step,
            "progress": task.progress,
            "message": task.message,
            "timestamp": datetime.now().isoformat()
        }

        # 通过 Socket.IO 广播
        if self.socket_io:
            await self.socket_io.emit("task_event", event_data)

        # 调用回调函数
        for callback in self._status_callbacks:
            try:
                callback(event_data)
            except Exception as e:
                logger.error(f"Error in status callback: {e}")

    def on_status_update(self, callback: Callable):
        """注册状态更新回调。"""
        self._status_callbacks.append(callback)

    def get_task_status(self, task_id: str) -> Optional[TaskStatus]:
        """获取任务状态。"""
        return self._tasks.get(task_id)

    def get_current_task(self) -> Optional[TaskStatus]:
        """获取当前正在运行的任务。"""
        if self._current_task_id:
            return self._tasks.get(self._current_task_id)
        return None

    async def handle_lower_machine_status(self, status_data: dict):
        """处理下位机状态更新。

        当下位机发送状态更新时调用此方法。
        """
        task_id = status_data.get("task_id")
        if not task_id or task_id not in self._tasks:
            logger.warning(f"Received status for unknown task: {task_id}")
            return

        task = self._tasks[task_id]

        # 更新任务状态
        task.status = status_data.get("status", task.status)
        task.current_step = status_data.get("current_step", task.current_step)
        task.progress = status_data.get("progress", task.progress)
        task.message = status_data.get("message", task.message)

        # 如果任务完成或出错，更新完成时间
        if task.status in ["completed", "error"]:
            task.completed_at = datetime.now()

        # 广播状态更新
        await self._broadcast_event("task_status_update", task)

        logger.debug(f"Updated task {task_id} status from lower machine: {task.status}")

    async def start(self):
        """启动任务分发器。"""
        self._running = True
        logger.info("SimpleTaskDispatcher started")

    async def stop(self):
        """停止任务分发器。"""
        self._running = False

        # 停止当前运行的任务
        if self._current_task_id:
            await self.stop_task(self._current_task_id)

        logger.info("SimpleTaskDispatcher stopped")

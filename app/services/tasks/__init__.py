"""RealSense 监控平台的任务系统。"""

from app.services.tasks.base_task import BaseTask
from app.services.tasks.registry import TaskRegistry, register_task

__all__ = ["BaseTask", "TaskRegistry", "register_task"]

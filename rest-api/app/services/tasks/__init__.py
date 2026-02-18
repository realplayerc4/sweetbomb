"""Task system for RealSense monitoring platform."""

from app.services.tasks.base_task import BaseTask
from app.services.tasks.registry import TaskRegistry, register_task

__all__ = ["BaseTask", "TaskRegistry", "register_task"]

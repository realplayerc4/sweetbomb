"""任务注册表。

负责任务的发现和创建，支持装饰器注册和插件式任务发现。
"""

from typing import Dict, List, Optional, Type, Callable
import logging

from app.models.task import TaskTypeInfo
from app.services.tasks.base_task import BaseTask


logger = logging.getLogger(__name__)


class TaskRegistry:
    """
    任务注册表（单例模式）。

    支持基于装饰器的注册和插件式任务发现。
    """

    _instance: Optional["TaskRegistry"] = None

    def __new__(cls) -> "TaskRegistry":
        """单例模式。"""
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._tasks: Dict[str, Type[BaseTask]] = {}
            cls._instance._initialized = False
        return cls._instance

    @classmethod
    def get_instance(cls) -> "TaskRegistry":
        """获取单例实例。"""
        return cls()

    def register(self, task_class: Type[BaseTask]) -> Type[BaseTask]:
        """
        注册任务类。

        Args:
            task_class: 要注册的任务类

        Returns:
            Type[BaseTask]: 同一个任务类（用于装饰器链式调用）

        Raises:
            ValueError: 如果任务类型已注册
        """
        task_type = task_class.task_type
        if task_type in self._tasks:
            logger.warning(f"Overwriting existing task type: {task_type}")

        self._tasks[task_type] = task_class
        logger.info(f"Registered task type: {task_type}")
        return task_class

    def get(self, task_type: str) -> Optional[Type[BaseTask]]:
        """
        根据类型获取任务类。

        Args:
            task_type: 任务类型标识符

        Returns:
            Type[BaseTask]: 任务类或 None（如果未找到）
        """
        return self._tasks.get(task_type)

    def list_types(self) -> List[str]:
        """获取所有已注册的任务类型列表。"""
        return list(self)tasks.keys())

    def get_type_info(self, task_type: str) -> Optional[TaskTypeInfo]:
        """
        获取任务类型的详细信息。

        Args:
            task_type: 任务类型标识符

        Returns:
            TaskTypeInfo: 任务类型信息或 None（如果未找到）
        """
        task_class = self.get(task_type)
        if not task_class:
            return None

        return TaskTypeInfo(
            task_type=task_class.task_type,
            name=task_class.name,
            description=task_class.description,
            category=task_class.category,
            requires_device=task_class.requires_device,
            params_schema=task_class.params_schema
        )

    def list_type_infos(self) -> List[TaskTypeInfo]:
        """获取所有已注册任务类型的详细信息列表。"""
        return [
            self.get_type_info(task_type)
            for task_type in self.list_types()
            if self.get_type_info(task_type) is not None
        ]

    def clear(self):
        """清除所有已注册的任务（用于测试）。"""
        self._tasks.clear()

    def __len__(self) -> int:
        """返回已注册任务数量。"""
        return len(self._tasks)

    def __contains__(self, task_type: str) -> bool:
        """检查任务类型是否已注册。"""
        return task_type in self._tasks


# 装饰器：用于注册任务
def register_task(task_class: Type[BaseTask]) -> Type[BaseTask]:
    """
    装饰器：用于向注册表注册任务类。

    用法：
        @register_task
        class MyTask(BaseTask):
            task_type = "my_task"
            name = "我的任务"
            description = "任务描述"
            ...
    """
    registry = TaskRegistry.get_instance()
    return registry.register(task_class)


# 便捷函数：获取注册表
def get_registry() -> TaskRegistry:
    """获取任务注册表单例。"""
    return TaskRegistry.get_instance()

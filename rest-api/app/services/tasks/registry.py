"""Task registry for discovering and creating tasks."""

from typing import Dict, List, Optional, Type, Callable
import logging

from app.models.task import TaskTypeInfo
from app.services.tasks.base_task import BaseTask


logger = logging.getLogger(__name__)


class TaskRegistry:
    """
    Singleton registry for task types.

    Supports decorator-based registration and plugin-style task discovery.
    """

    _instance: Optional["TaskRegistry"] = None

    def __new__(cls) -> "TaskRegistry":
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._tasks: Dict[str, Type[BaseTask]] = {}
            cls._instance._initialized = False
        return cls._instance

    @classmethod
    def get_instance(cls) -> "TaskRegistry":
        """Get the singleton registry instance."""
        return cls()

    def register(self, task_class: Type[BaseTask]) -> Type[BaseTask]:
        """
        Register a task class.

        Args:
            task_class: The task class to register

        Returns:
            The same task class (for decorator chaining)

        Raises:
            ValueError: If task_type is already registered
        """
        task_type = task_class.task_type
        if task_type in self._tasks:
            logger.warning(f"Overwriting existing task type: {task_type}")

        self._tasks[task_type] = task_class
        logger.info(f"Registered task type: {task_type}")
        return task_class

    def get(self, task_type: str) -> Optional[Type[BaseTask]]:
        """
        Get a task class by type.

        Args:
            task_type: The task type identifier

        Returns:
            The task class or None if not found
        """
        return self._tasks.get(task_type)

    def list_types(self) -> List[str]:
        """Get list of all registered task types."""
        return list(self._tasks.keys())

    def get_type_info(self, task_type: str) -> Optional[TaskTypeInfo]:
        """
        Get detailed information about a task type.

        Args:
            task_type: The task type identifier

        Returns:
            TaskTypeInfo or None if not found
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
        """Get detailed information about all registered task types."""
        return [
            self.get_type_info(task_type)
            for task_type in self.list_types()
            if self.get_type_info(task_type) is not None
        ]

    def clear(self):
        """Clear all registered tasks (useful for testing)."""
        self._tasks.clear()

    def __len__(self) -> int:
        return len(self._tasks)

    def __contains__(self, task_type: str) -> bool:
        return task_type in self._tasks


# Decorator for registering tasks
def register_task(task_class: Type[BaseTask]) -> Type[BaseTask]:
    """
    Decorator to register a task class with the registry.

    Usage:
        @register_task
        class MyTask(BaseTask):
            task_type = "my_task"
            ...
    """
    registry = TaskRegistry.get_instance()
    return registry.register(task_class)


# Convenience function to get registry
def get_registry() -> TaskRegistry:
    """Get the task registry singleton."""
    return TaskRegistry.get_instance()

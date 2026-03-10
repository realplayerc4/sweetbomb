"""导航服务接口抽象。

该模块定义了导航服务的抽象接口，用于解耦本系统与导航组的实现。
当前使用模拟实现，真实导航接口对接时可替换实现类。
"""

from abc import ABC, abstractmethod
from typing import Tuple, Optional
from dataclasses import dataclass
from enum import Enum


class NavigationStatus(str, Enum):
    """导航状态枚举。"""
    IDLE = "idle"
    PLANNING = "planning"
    MOVING = "moving"
    REACHED = "reached"
    FAILED = "failed"
    CANCELLED = "cancelled"


@dataclass
class NavigationResult:
    """导航执行结果。"""
    success: bool
    status: NavigationStatus
    final_position: Tuple[float, float, float]
    message: str = ""
    error: Optional[str] = None
    duration_seconds: float = 0.0


@dataclass
class NavigationTarget:
    """导航目标点。"""
    x: float
    y: float
    z: float = 0.0
    name: Optional[str] = None
    tolerance: float = 0.1  # 到达容差（米）


class INavigationService(ABC):
    """导航服务抽象接口。

    由另外工作组负责的导航模块通过此接口与本系统交互。
    """

    @abstractmethod
    async def navigate_to(
        self,
        target: NavigationTarget
    ) -> NavigationResult:
        """导航到指定坐标点。

        Args:
            target: 导航目标点，包含 x, y, z 坐标和可选的名称

        Returns:
            NavigationResult: 导航执行结果，包含最终位置和状态
        """
        pass

    @abstractmethod
    async def get_current_position(self) -> Tuple[float, float, float]:
        """获取当前位置。

        Returns:
            当前位置坐标 (x, y, z)
        """
        pass

    @abstractmethod
    async def get_navigation_status(self) -> NavigationStatus:
        """获取当前导航状态。

        Returns:
            NavigationStatus: 当前导航状态
        """
        pass

    @abstractmethod
    async def cancel_navigation(self) -> None:
        """取消当前导航任务。"""
        pass

    @abstractmethod
    def is_ready(self) -> bool:
        """检查导航服务是否就绪。

        Returns:
            bool: 导航服务是否可用
        """
        pass

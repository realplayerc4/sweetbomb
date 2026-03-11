"""模拟导航服务实现。

在真实导航硬件就绪前，使用模拟实现来开发和测试铲糖机器人功能。
"""

import asyncio
import logging
from typing import Tuple, Optional
from datetime import datetime

from app.services.navigation_interface import (
    INavigationService,
    NavigationStatus,
    NavigationResult,
    NavigationTarget,
)

logger = logging.getLogger(__name__)


class MockNavigationService(INavigationService):
    """模拟导航服务。

    模拟履带式机器人移动到指定坐标点的行为。
    """

    def __init__(self, initial_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)):
        """初始化模拟导航服务。

        Args:
            initial_position: 初始位置，默认为原点
        """
        self._position = list(initial_position)
        self._status = NavigationStatus.IDLE
        self._lock = asyncio.Lock()
        self._navigation_task: Optional[asyncio.Task] = None

    async def navigate_to(self, target: NavigationTarget) -> NavigationResult:
        """导航到指定坐标点（模拟）。

        模拟过程：
        1. 检查目标位置是否有效
        2. 计算移动距离和预计时间
        3. 模拟分步移动过程
        4. 返回最终结果

        Args:
            target: 导航目标点

        Returns:
            NavigationResult: 导航执行结果
        """
        async with self._lock:
            if self._status == NavigationStatus.MOVING:
                return NavigationResult(
                    success=False,
                    status=self._status,
                    final_position=tuple(self._position),
                    message="已有导航任务正在执行",
                )

            start_pos = self._position.copy()
            target_pos = [target.x, target.y, target.z]

            # 计算距离
            distance = sum(
                (a - b) ** 2 for a, b in zip(start_pos, target_pos)
            ) ** 0.5

            # 模拟移动速度：0.5 m/s
            speed = 0.5
            duration = distance / speed if distance > 0 else 1.0

            self._status = NavigationStatus.MOVING
            logger.info(
                f"导航开始: {target.name or target_pos} "
                f"距离: {distance:.2f}m, 预计时间: {duration:.2f}s"
            )

            start_time = datetime.now()

            try:
                # 分 20 步模拟移动
                steps = 20
                step_duration = duration / steps

                for i in range(steps):
                    await asyncio.sleep(step_duration)

                    # 线性插值更新位置
                    ratio = (i + 1) / steps
                    for j in range(3):
                        self._position[j] = start_pos[j] + (
                            target_pos[j] - start_pos[j]
                        ) * ratio

                    logger.debug(
                        f"导航进度: {ratio * 100:.0f}% "
                        f"当前位置: [{self._position[0]:.2f}, {self._position[1]:.2f}, {self._position[2]:.2f}]"
                    )

                self._status = NavigationStatus.REACHED
                elapsed = (datetime.now() - start_time).total_seconds()

                logger.info(
                    f"导航完成: 目标 {target.name or target_pos}, "
                    f"最终位置: {self._position}"
                )

                return NavigationResult(
                    success=True,
                    status=NavigationStatus.REACHED,
                    final_position=tuple(self._position),
                    message=f"成功到达目标点 {target.name or target_pos}",
                    duration_seconds=elapsed,
                )

            except asyncio.CancelledError:
                self._status = NavigationStatus.CANCELLED
                logger.info("导航任务已取消")
                return NavigationResult(
                    success=False,
                    status=NavigationStatus.CANCELLED,
                    final_position=tuple(self._position),
                    message="导航任务已取消",
                )

    async def get_current_position(self) -> Tuple[float, float, float]:
        """获取当前位置。"""
        return tuple(self._position)

    async def get_navigation_status(self) -> NavigationStatus:
        """获取当前导航状态。"""
        return self._status

    async def cancel_navigation(self) -> None:
        """取消当前导航任务。"""
        if self._status == NavigationStatus.MOVING:
            self._status = NavigationStatus.CANCELLED
            logger.info("导航任务取消请求已发送")

    def is_ready(self) -> bool:
        """检查导航服务是否就绪。"""
        return self._status != NavigationStatus.MOVING


# 全局单例
_mock_navigation_instance: Optional[MockNavigationService] = None


def get_mock_navigation() -> MockNavigationService:
    """获取模拟导航服务单例。"""
    global _mock_navigation_instance
    if _mock_navigation_instance is None:
        _mock_navigation_instance = MockNavigationService()
    return _mock_navigation_instance

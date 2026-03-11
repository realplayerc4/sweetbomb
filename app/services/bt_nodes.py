"""行为树节点定义。

行为树（Behavior Tree）是一种用于 AI 决策和任务控制的树状结构。
包含节点类型：
- ActionNode: 动作节点，执行具体操作
- ConditionNode: 条件节点，检查条件
- SequenceNode: 序列节点，顺序执行子节点
- SelectorNode: 选择节点，依次执行子节点直到成功
"""

import asyncio
import logging
from abc import ABC, abstractmethod
from enum import Enum
from typing import Any, Dict, List, Optional
from dataclasses import dataclass, field
from datetime import datetime

logger = logging.getLogger(__name__)


class NodeStatus(str, Enum):
    """节点执行状态。"""
    SUCCESS = "success"
    FAILURE = "failure"
    RUNNING = "running"
    IDLE = "idle"


@dataclass
class NodeContext:
    """节点执行上下文。"""
    robot_controller: Any = None  # RobotController 实例
    navigation_service: Any = None  # INavigationService 实例
    distance_analyzer: Any = None  # DistanceAnalyzer 实例
    task_params: Dict[str, Any] = field(default_factory=dict)
    blackboard: Dict[str, Any] = field(default_factory=dict)
    current_cycle: int = 0
    max_cycles: int = 10
    sugar_height: float = 0.0
    height_threshold: float = 0.20


class BTNode(ABC):
    """行为树节点基类。"""

    def __init__(self, name: str, children: Optional[List["BTNode"]] = None):
        """初始化节点。

        Args:
            name: 节点名称
            children: 子节点列表
        """
        self.name = name
        self.children = children or []
        self.status = NodeStatus.IDLE
        self._parent: Optional["BTNode"] = None

        # 设置子节点的父节点
        for child in self.children:
            child._parent = self

    @abstractmethod
    async def tick(self, context: NodeContext) -> NodeStatus:
        """执行节点逻辑。

        Args:
            context: 执行上下文

        Returns:
            NodeStatus: 执行状态
        """
        pass

    def reset(self):
        """重置节点状态。"""
        self.status = NodeStatus.IDLE
        for child in self.children:
            child.reset()

    def get_path(self) -> str:
        """获取节点路径（用于调试）。"""
        if self._parent:
            return f"{self._parent.get_path()}/{self.name}"
        return self.name

    async def broadcast_status(self, context: NodeContext):
        """广播节点状态变化。

        通过 Socket.IO 向前端推送当前执行节点。
        """
        from app.services.socketio import sio

        await sio.emit(
            "bt_node_status",
            {
                "node": self.name,
                "status": self.status,
                "path": self.get_path(),
                "timestamp": datetime.now().isoformat(),
            },
        )


class ActionNode(BTNode):
    """动作节点，执行具体操作。"""

    def __init__(self, name: str, action_fn):
        """初始化动作节点。

        Args:
            name: 节点名称
            action_fn: 异步动作函数，接收 context，返回 NodeStatus
        """
        super().__init__(name)
        self.action_fn = action_fn

    async def tick(self, context: NodeContext) -> NodeStatus:
        """执行动作。"""
        logger.info(f"执行动作节点: {self.name}")
        self.status = NodeStatus.RUNNING
        await self.broadcast_status(context)

        try:
            result = await self.action_fn(context)
            self.status = result
            await self.broadcast_status(context)
            return result
        except Exception as e:
            logger.error(f"动作节点 {self.name} 执行失败: {e}")
            self.status = NodeStatus.FAILURE
            await self.broadcast_status(context)
            return NodeStatus.FAILURE


class ConditionNode(BTNode):
    """条件节点，检查条件。"""

    def __init__(self, name: str, condition_fn):
        """初始化条件节点。

        Args:
            name: 节点名称
            condition_fn: 条件函数，接收 context，返回 bool
        """
        super().__init__(name)
        self.condition_fn = condition_fn

    async def tick(self, context: NodeContext) -> NodeStatus:
        """检查条件。"""
        logger.info(f"检查条件节点: {self.name}")

        try:
            result = self.condition_fn(context)
            self.status = NodeStatus.SUCCESS if result else NodeStatus.FAILURE
            await self.broadcast_status(context)

            logger.info(
                f"条件节点 {self.name} 结果: {result} ({self.status})"
            )
            return self.status
        except Exception as e:
            logger.error(f"条件节点 {self.name} 检查失败: {e}")
            self.status = NodeStatus.FAILURE
            await self.broadcast_status(context)
            return NodeStatus.FAILURE


class SequenceNode(BTNode):
    """序列节点，顺序执行子节点。

    所有子节点都成功才返回成功，任意子节点失败则返回失败。
    """

    async def tick(self, context: NodeContext) -> NodeStatus:
        """顺序执行子节点。"""
        logger.info(f"执行序列节点: {self.name}")
        self.status = NodeStatus.RUNNING
        await self.broadcast_status(context)

        for child in self.children:
            child_status = await child.tick(context)

            if child_status == NodeStatus.RUNNING:
                self.status = NodeStatus.RUNNING
                await self.broadcast_status(context)
                return NodeStatus.RUNNING

            if child_status == NodeStatus.FAILURE:
                self.status = NodeStatus.FAILURE
                await self.broadcast_status(context)
                return NodeStatus.FAILURE

        self.status = NodeStatus.SUCCESS
        await self.broadcast_status(context)
        return NodeStatus.SUCCESS


class SelectorNode(BTNode):
    """选择节点，依次执行子节点直到成功。

    任意子节点成功则返回成功，所有子节点都失败才返回失败。
    """

    async def tick(self, context: NodeContext) -> NodeStatus:
        """依次执行子节点。"""
        logger.info(f"执行选择节点: {self.name}")
        self.status = NodeStatus.RUNNING
        await self.broadcast_status(context)

        for child in self.children:
            child_status = await child.tick(context)

            if child_status == NodeStatus.RUNNING:
                self.status = NodeStatus.RUNNING
                await self.broadcast_status(context)
                return NodeStatus.RUNNING

            if child_status == NodeStatus.SUCCESS:
                self.status = NodeStatus.SUCCESS
                await self.broadcast_status(context)
                return NodeStatus.SUCCESS

        self.status = NodeStatus.FAILURE
        await self.broadcast_status(context)
        return NodeStatus.FAILURE


class RepeatNode(BTNode):
    """重复节点，重复执行子节点。

    Args:
        child: 子节点
        max_count: 最大重复次数，-1 表示无限循环
    """

    def __init__(self, name: str, child: BTNode, max_count: int = -1):
        super().__init__(name, [child])
        self.max_count = max_count
        self._current_count = 0

    async def tick(self, context: NodeContext) -> NodeStatus:
        """重复执行子节点。"""
        logger.info(
            f"执行重复节点: {self.name} (计数: {self._current_count}/{self.max_count})"
        )

        # 检查循环次数限制
        if self.max_count > 0 and self._current_count >= self.max_count:
            logger.info(f"重复节点达到最大次数: {self.max_count}")
            self.status = NodeStatus.SUCCESS
            await self.broadcast_status(context)
            return NodeStatus.SUCCESS

        self.status = NodeStatus.RUNNING
        await self.broadcast_status(context)

        # 执行子节点
        child = self.children[0]
        child_status = await child.tick(context)

        if child_status == NodeStatus.RUNNING:
            return NodeStatus.RUNNING

        if child_status == NodeStatus.SUCCESS:
            self._current_count += 1
            context.current_cycle = self._current_count

            # 重置子节点状态以便下次执行
            child.reset()

            # 继续循环
            return NodeStatus.RUNNING

        # 子节点失败
        self.status = NodeStatus.FAILURE
        await self.broadcast_status(context)
        return NodeStatus.FAILURE

    def reset(self):
        """重置节点状态。"""
        super().reset()
        self._current_count = 0


class DecoratorNode(BTNode):
    """装饰器节点基类。

    包装单个子节点，修改其行为。
    """

    def __init__(self, name: str, child: BTNode):
        super().__init__(name, [child])

    @abstractmethod
    async def tick(self, context: NodeContext) -> NodeStatus:
        """执行装饰器逻辑。"""
        pass


class InverterNode(DecoratorNode):
    """反转节点，反转子节点的结果。

    SUCCESS -> FAILURE, FAILURE -> SUCCESS
    """

    async def tick(self, context: NodeContext) -> NodeStatus:
        """反转子节点结果。"""
        child_status = await self.children[0].tick(context)

        if child_status == NodeStatus.RUNNING:
            return NodeStatus.RUNNING

        inverted = (
            NodeStatus.FAILURE if child_status == NodeStatus.SUCCESS
            else NodeStatus.SUCCESS
        )

        logger.info(f"反转节点 {self.name}: {child_status} -> {inverted}")
        return inverted

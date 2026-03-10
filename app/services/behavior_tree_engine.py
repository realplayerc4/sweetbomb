"""行为树引擎。

负责行为树的调度、执行和状态管理。
"""

import asyncio
import logging
from typing import Any, Dict, Optional
from datetime import datetime

from app.services.bt_nodes import (
    BTNode,
    SequenceNode,
    SelectorNode,
    RepeatNode,
    NodeContext,
    NodeStatus,
)
from app.services.bt_action_nodes import (
    create_navigate_to_sugar_node,
    create_check_shovel_flat_node,
    create_analyze_sugar_node,
    create_check_sugar_height_node,
    create_navigate_to_dump_node,
    create_dump_action_node,
    create_check_cycle_limit_node,
    create_return_to_home_node,
)
from app.services.bt_calculate_distance_node import create_calculate_approach_distance_node
from app.services.bt_scoop_action import create_scoop_and_return_node
from app.services.bt_dump_action import create_dump_and_return_node

logger = logging.getLogger(__name__)


class BehaviorTreeEngine:
    """行为树引擎。

    负责执行行为树，管理执行上下文，广播状态变化。
    """

    def __init__(self, root: BTNode, context: NodeContext):
        """初始化行为树引擎。

        Args:
            root: 行为树根节点
            context: 执行上下文
        """
        self.root = root
        self.context = context
        self._running = False
        self._task: Optional[asyncio.Task] = None
        self._stop_requested = False

    @classmethod
    def create_sugar_harvest_tree(cls, context: NodeContext) -> "BehaviorTreeEngine":
        """创建铲糖自主循环行为树。

        行为树结构：
        ```
        Repeat (主循环)
        ├── Sequence (单次循环)
        │   ├── NavigateToSugarPoint (导航到取糖点)
        │   ├── AnalyzeSugarDistance (分析糖堆)
        │   ├── CheckSugarHeight (检查糖堆高度)
        │   │   ├── SUCCESS (高度足够)
        │   │   │   ├── MoveForwardToScoop (前进铲糖)
        │   │   │   ├── ReverseToNavPoint (原路倒退)
        │   │   │   ├── NavigateToDumpPoint (导航到卸载点)
        │   │   │   ├── DumpAction (翻斗卸载)
        │   │   │   └── CheckCycleLimit (检查循环次数)
        │   │   │       ├── SUCCESS → 继续循环
        │   │   │       └── FAILURE → 停止循环
        │   │   └── FAILURE (高度不足)
        │   │       └── SwitchToPushMode (切换推垛模式)
        └── 达到上限 → 停止
        ```
        """
        # 创建单次铲糖流程的序列节点
        single_cycle = SequenceNode(
            "SugarHarvestCycle",
            children=[
                create_navigate_to_sugar_node(),
                create_analyze_sugar_node(),
                # 使用选择节点处理高度检查结果
                SelectorNode(
                    "HeightCheckSelector",
                    children=[
                        # 高度足够：执行铲糖流程
                        SequenceNode(
                            "HarvestSequence",
                            children=[
                                create_move_forward_to_scoop_node(),
                                create_reverse_to_nav_point_node(),
                                create_navigate_to_dump_node(),
                                create_dump_action_node(),
                                # 检查循环次数，达到上限时返回 FAILURE 终止循环
                                RepeatNode(
                                    "CycleLimitChecker",
                                    child=SequenceNode(
                                        "CheckAndContinue",
                                        children=[
                                            # 空节点，仅用于结构
                                            ActionNode(
                                                "CycleContinue",
                                                lambda ctx: asyncio.sleep(0)
                                            ),
                                        ],
                                    ),
                                    max_cycle=1,  # 这个逻辑在外层的 RepeatNode 处理
                                ),
                            ],
                        ),
                        # 高度不足：切换推垛模式
                        create_switch_to_push_mode_node(),
                    ],
                ),
            ],
        )

        # 注意：上面的选择节点逻辑需要调整
        # 重新设计：使用条件节点 + 选择节点
        check_height = create_check_sugar_height_node()

        # 高度足够时的铲糖流程
        harvest_sequence = SequenceNode(
            "HarvestSequence",
            children=[
                create_calculate_approach_distance_node(),  # 计算前进距离
                create_scoop_and_return_node(),  # 连贯动作：前进 → 翻转铲子 → 倒退
                create_dump_and_return_node(),  # 连贯动作：导航A点→举升→导航B点→倾倒→倒退→归零
            ],
        )

        # 构建更合理的树结构
        # 主循环：重复执行铲糖流程，直到达到循环上限
        main_tree = RepeatNode(
            "SugarHarvestMainLoop",
            child=SequenceNode(
                "FullHarvestCycle",
                children=[
                    create_navigate_to_sugar_node(),
                    create_check_shovel_flat_node(),
                    create_analyze_sugar_node(),
                    # 条件检查：高度是否足够
                    SelectorNode(
                        "HeightCheck",
                        children=[
                            # 高度足够 -> 继续铲糖
                            SequenceNode(
                                "ContinueHarvest",
                                children=[
                                    harvest_sequence,
                                ],
                            ),
                            # 高度不足 -> 回桩（返回充电桩）
                            create_return_to_home_node(),
                        ],
                    ),
                ],
            ),
            max_count=context.max_cycles,
        )

        return cls(main_tree, context)

    async def start(self) -> NodeStatus:
        """启动行为树执行。

        Returns:
            NodeStatus: 最终执行状态
        """
        if self._running:
            logger.warning("行为树已在运行中")
            return NodeStatus.RUNNING

        self._running = True
        self._stop_requested = False

        logger.info("启动行为树执行")
        await self._broadcast_event("bt_started", {"tree": "SugarHarvestMainLoop"})

        try:
            # 执行行为树
            final_status = await self._execute()

            logger.info(f"行为树执行完成: {final_status}")
            await self._broadcast_event(
                "bt_completed",
                {
                    "status": final_status,
                    "cycles": self.context.current_cycle,
                    "sugar_height": self.context.sugar_height,
                },
            )

            return final_status

        except asyncio.CancelledError:
            logger.info("行为树执行被取消")
            await self._broadcast_event("bt_cancelled", {})
            return NodeStatus.FAILURE

        except Exception as e:
            logger.error(f"行为树执行异常: {e}", exc_info=True)
            await self._broadcast_event("bt_error", {"error": str(e)})
            return NodeStatus.FAILURE

        finally:
            self._running = False

    async def _execute(self) -> NodeStatus:
        """执行行为树的内部方法。"""
        status = NodeStatus.RUNNING

        # 循环执行根节点，直到状态不再是 RUNNING
        while status == NodeStatus.RUNNING and not self._stop_requested:
            status = await self.root.tick(self.context)

            # 如果是 RUNNING 状态，稍等后继续
            if status == NodeStatus.RUNNING:
                await asyncio.sleep(0.1)

        return status

    async def stop(self) -> None:
        """请求停止行为树执行。"""
        if not self._running:
            logger.warning("行为树未在运行")
            return

        logger.info("请求停止行为树执行")
        self._stop_requested = True

        await self._broadcast_event("bt_stop_requested", {})

    def reset(self) -> None:
        """重置行为树状态。"""
        self.root.reset()
        self.context.current_cycle = 0
        self.context.blackboard.clear()
        self._stop_requested = False
        logger.info("行为树状态已重置")

    async def _broadcast_event(self, event_type: str, data: Dict[str, Any]) -> None:
        """广播行为树事件。"""
        from app.services.socketio import sio

        await sio.emit(
            "bt_event",
            {
                "event_type": event_type,
                "timestamp": datetime.now().isoformat(),
                **data,
            },
        )

    @property
    def is_running(self) -> bool:
        """是否正在运行。"""
        return self._running


def create_sugar_harvest_engine(
    robot_controller,
    navigation_service,
    distance_analyzer,
    task_params: Dict[str, Any],
) -> BehaviorTreeEngine:
    """创建铲糖行为树引擎的便捷函数。

    Args:
        robot_controller: 机器人控制器实例
        navigation_service: 导航服务实例
        distance_analyzer: 距离分析器实例
        task_params: 任务参数

    Returns:
        BehaviorTreeEngine: 配置好的行为树引擎
    """
    # 创建执行上下文
    context = NodeContext(
        robot_controller=robot_controller,
        navigation_service=navigation_service,
        distance_analyzer=distance_analyzer,
        task_params=task_params,
        max_cycles=task_params.get("max_cycles", 10),
        height_threshold=task_params.get("height_threshold_m", 0.20),
    )

    # 创建行为树引擎
    return BehaviorTreeEngine.create_sugar_harvest_tree(context)

"""计算前进距离节点。

该节点负责根据糖堆分析结果计算前进铲糖的精确距离，
并将结果保存到黑板供后续节点使用。
"""

import logging

from app.services.bt_nodes import (
    ActionNode,
    NodeContext,
    NodeStatus,
)
from app.services.distance_analyzer import DistanceAnalysisResult

logger = logging.getLogger(__name__)


def create_calculate_approach_distance_node() -> ActionNode:
    """创建计算前进距离动作节点。

    根据糖堆分析结果计算前进铲糖的距离，并保存到 blackboard。

    计算逻辑:
        move_distance = analysis.distance_m - approach_offset
        其中 approach_offset 默认为 0.05m (5cm)

    黑板输出:
        - approach_distance: 计算后的前进距离 (float, 单位: 米)

    Returns:
        ActionNode: 计算前进距离的动作节点
    """

    async def calculate_action(context: NodeContext) -> NodeStatus:
        # 获取距离分析结果
        analysis: DistanceAnalysisResult = context.blackboard.get("distance_analysis")
        if not analysis:
            logger.error("缺少距离分析结果，无法计算前进距离")
            return NodeStatus.FAILURE

        # 计算前进距离（减去偏移量，避免撞到糖堆）
        approach_offset = context.task_params.get("approach_offset_m", 0.05)
        move_distance = analysis.distance_m - approach_offset

        if move_distance <= 0:
            logger.warning(f"计算的前进距离非正: {move_distance:.3f}m，使用默认值 0.1m")
            move_distance = 0.1

        # 保存计算结果到 blackboard
        context.blackboard["approach_distance"] = move_distance

        logger.info(
            f"计算前进距离完成: {move_distance:.3f}m "
            f"(原始距离={analysis.distance_m:.3f}m, 偏移={approach_offset}m)"
        )

        return NodeStatus.SUCCESS

    return ActionNode("CalculateApproachDistance", calculate_action)

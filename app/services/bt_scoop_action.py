"""铲糖连贯动作节点。

将前进铲糖、翻转铲子、倒退回原位整合为一个连贯的动作模块。
"""

import asyncio
import logging

from app.services.bt_nodes import (
    ActionNode,
    NodeContext,
    NodeStatus,
)
from app.services.distance_analyzer import DistanceAnalysisResult

logger = logging.getLogger(__name__)


def create_scoop_and_return_node() -> ActionNode:
    """创建铲糖并返回动作节点。

    连贯执行以下动作：
    1. 根据计算的距离前进到糖堆
    2. 翻转铲子（举升铲斗）完成铲糖
    3. 倒退回到原位

    依赖黑板数据：
        - approach_distance: 前进距离（由 CalculateApproachDistance 计算）
        - nav_point_position: 导航点位置（用于计算倒退距离）

    Returns:
        ActionNode: 铲糖并返回的动作节点
    """

    async def scoop_and_return_action(context: NodeContext) -> NodeStatus:
        # ========== 第1步：前进到糖堆 ==========
        move_distance = context.blackboard.get("approach_distance")
        if move_distance is None:
            logger.error("缺少前进距离，请先执行 CalculateApproachDistance 节点")
            return NodeStatus.FAILURE

        logger.info(f"[铲糖动作] 第1步：前进 {move_distance:.3f}m 到糖堆")

        success = await context.robot_controller.move_distance(
            distance=move_distance,
            speed=0.3  # 铲糖时使用较低速度
        )

        if not success:
            logger.error("[铲糖动作] 前进失败")
            return NodeStatus.FAILURE

        logger.info("[铲糖动作] 前进完成，到达糖堆")

        # ========== 第2步：翻转铲子（举升铲斗）==========
        logger.info("[铲糖动作] 第2步：翻转铲子举升铲斗")

        scoop_position = context.task_params.get("scoop_position", 90.0)
        await context.robot_controller.set_servo_angle("lift", scoop_position)

        # 等待铲斗稳定
        await asyncio.sleep(0.5)

        logger.info(f"[铲糖动作] 铲斗举升到 {scoop_position}°，铲糖完成")

        # ========== 第3步：倒退回原位 ==========
        logger.info("[铲糖动作] 第3步：倒退回原位")

        # 获取之前记录的导航点位置
        nav_position = context.blackboard.get("nav_point_position")
        if not nav_position:
            logger.warning("未记录导航点位置，使用默认倒退距离 0.5m")
            reverse_distance = 0.5
        else:
            # 获取当前位置
            current_position = await context.robot_controller.get_status()
            current = current_position.current_position

            # 计算 X 轴方向的距离（假设机器人主要沿 X 轴移动）
            reverse_distance = current[0] - nav_position[0]

        if reverse_distance < 0:
            logger.warning(f"计算的倒退距离为负: {reverse_distance:.3f}m，使用默认值 0.5m")
            reverse_distance = 0.5

        logger.info(f"[铲糖动作] 倒退回原位: 距离={reverse_distance:.3f}m")

        # 执行倒退
        success = await context.robot_controller.move_distance(
            distance=-reverse_distance,  # 负数表示倒退
            speed=0.3
        )

        if not success:
            logger.error("[铲糖动作] 倒退失败")
            return NodeStatus.FAILURE

        logger.info("[铲糖动作] 倒退完成，已回到原位")

        # ========== 铲糖动作全部完成 ==========
        logger.info("[铲糖动作] 全部完成：前进 → 翻转铲子 → 倒退")

        return NodeStatus.SUCCESS

    return ActionNode("ScoopAndReturn", scoop_and_return_action)

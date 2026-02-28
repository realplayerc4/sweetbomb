"""铲糖行为树动作节点。

定义铲糖自主循环所需的具体动作节点：
- 导航到取糖点
- 分析糖堆距离和高度
- 检查糖堆高度
- 前进铲糖
- 原路倒退
- 导航到卸载点
- 翻斗卸载
- 检查循环次数
"""

import logging
from typing import Tuple

from app.services.bt_nodes import (
    ActionNode,
    ConditionNode,
    NodeContext,
    NodeStatus,
)
from app.services.navigation_interface import NavigationTarget
from app.services.distance_analyzer import DistanceAnalysisResult

logger = logging.getLogger(__name__)


# ==================== 动作节点工厂函数 ====================

def create_navigate_to_sugar_node() -> ActionNode:
    """创建导航到取糖点动作节点。"""

    async def navigate_action(context: NodeContext) -> NodeStatus:
        nav_point = context.task_params.get("navigation_point")
        if not nav_point:
            logger.error("缺少 navigation_point 参数")
            return NodeStatus.FAILURE

        target = NavigationTarget(
            x=nav_point[0],
            y=nav_point[1],
            z=0.0,
            name="取糖点"
        )

        logger.info(f"导航到取糖点: {nav_point}")

        result = await context.navigation_service.navigate_to(target)

        if result.success:
            # 记录导航点位置，用于后续倒退返回
            context.blackboard["nav_point_position"] = result.final_position
            return NodeStatus.SUCCESS
        else:
            logger.error(f"导航失败: {result.error or result.message}")
            return NodeStatus.FAILURE

    return ActionNode("NavigateToSugarPoint", navigate_action)


def create_analyze_sugar_node() -> ActionNode:
    """创建分析糖堆动作节点。"""

    async def analyze_action(context: NodeContext) -> NodeStatus:
        logger.info("开始分析糖堆距离和高度")

        # 获取点云数据（这里从任务参数或上下文获取）
        point_cloud = context.task_params.get("point_cloud")
        if point_cloud is None:
            logger.warning("点云数据未提供，使用模拟数据")

            # 模拟点云数据（用于测试）
            import numpy as np
            np.random.seed(42)
            n_points = 100
            point_cloud = np.random.rand(n_points, 3) * 2  # 2m 范围内的随机点
            point_cloud[:, 0] += 1.0  # X 轴偏移，模拟距离 1m

        # 使用距离分析器分析
        result = await context.distance_analyzer.analyze_async(point_cloud)

        if result is None:
            logger.error("距离分析失败")
            return NodeStatus.FAILURE

        # 保存分析结果到黑板
        context.blackboard["distance_analysis"] = result
        context.sugar_height = result.sugar_height_m

        logger.info(
            f"糖堆分析完成: "
            f"距离={result.distance_m:.3f}m, "
            f"高度={result.sugar_height_m:.3f}m"
        )

        return NodeStatus.SUCCESS

    return ActionNode("AnalyzeSugarDistance", analyze_action)


def create_check_sugar_height_node() -> ConditionNode:
    """创建检查糖堆高度条件节点。

    Returns:
        SUCCESS if 高度 >= 20cm (继续铲糖)
        FAILURE if 高度 < 20cm (切换推垛模式)
    """

    def check_condition(context: NodeContext) -> NodeStatus:
        height = context.sugar_height
        threshold = context.height_threshold

        is_high_enough = height >= threshold

        logger.info(
            f"糖堆高度检查: {height:.3f}m >= {threshold:.3f}m = {is_high_enough}"
        )

        return is_high_enough

    return ConditionNode("CheckSugarHeight", check_condition)


def create_switch_to_push_mode_node() -> ActionNode:
    """创建切换到推垛模式动作节点。"""

    async def switch_action(context: NodeContext) -> NodeStatus:
        logger.warning("糖堆高度不足，需要切换到推垛模式")
        logger.info("推垛模式将在后续版本实现")

        # 标记需要切换模式
        context.blackboard["switch_to_push_mode"] = True

        # 当前先返回 FAILURE 终止循环
        # 后续可实现完整的推垛模式行为树
        return NodeStatus.FAILURE

    return ActionNode("SwitchToPushMode", switch_action)


def create_move_forward_to_scoop_node() -> ActionNode:
    """创建前进铲糖动作节点。"""

    async def scoop_action(context: NodeContext) -> NodeStatus:
        # 获取距离分析结果
        analysis: DistanceAnalysisResult = context.blackboard.get("distance_analysis")
        if not analysis:
            logger.error("缺少距离分析结果")
            return NodeStatus.FAILURE

        # 计算前进距离（减去偏移量，避免撞到糖堆）
        approach_offset = context.task_params.get("approach_offset_m", 0.05)
        move_distance = analysis.distance_m - approach_offset

        if move_distance <= 0:
            logger.warning(f"计算的前进距离非正: {move_distance:.3f}m，使用默认值 0.1m")
            move_distance = 0.1

        logger.info(f"前进铲糖: 距离={move_distance:.3f}m")

        # 执行前进
        success = await context.robot_controller.move_distance(
            distance=move_distance,
            speed=0.3  # 铲糖时使用较低速度
        )

        if not success:
            logger.error("前进失败")
            return NodeStatus.FAILURE

        # 铲斗举升到铲取位置
        scoop_position = context.task_params.get("scoop_position", 90.0)
        await context.robot_controller.set_servo_angle("lift", scoop_position)

        logger.info("前进铲糖完成")
        return NodeStatus.SUCCESS

    return ActionNode("MoveForwardToScoop", scoop_action)


def create_reverse_to_nav_point_node() -> ActionNode:
    """创建原路倒退动作节点。"""

    async def reverse_action(context: NodeContext) -> NodeStatus:
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
            logger.warning(f"计算的倒退距离为负: {reverse_distance:.3f}m")
            reverse_distance = 0.5

        logger.info(f"原路倒退: 距离={reverse_distance:.3f}m")

        # 执行倒退
        success = await context.robot_controller.move_distance(
            distance=-reverse_distance,  # 负数表示倒退
            speed=0.3
        )

        if not success:
            logger.error("倒退失败")
            return NodeStatus.FAILURE

        logger.info("原路倒退完成")
        return NodeStatus.SUCCESS

    return ActionNode("ReverseToNavPoint", reverse_action)


def create_navigate_to_dump_node() -> ActionNode:
    """创建导航到卸载点动作节点。"""

    async def navigate_action(context: NodeContext) -> NodeStatus:
        dump_point = context.task_params.get("dump_point")
        if not dump_point:
            logger.error("缺少 dump_point 参数")
            return NodeStatus.FAILURE

        target = NavigationTarget(
            x=dump_point[0],
            y=dump_point[1],
            z=0.0,
            name="卸载点"
        )

        logger.info(f"导航到卸载点: {dump_point}")

        result = await context.navigation_service.navigate_to(target)

        if result.success:
            return NodeStatus.SUCCESS
        else:
            logger.error(f"导航到卸载点失败: {result.error or result.message}")
            return NodeStatus.FAILURE

    return ActionNode("NavigateToDumpPoint", navigate_action)


def create_dump_action_node() -> ActionNode:
    """创建翻斗卸载动作节点。"""

    async def dump_action(context: NodeContext) -> NodeStatus:
        logger.info("开始翻斗卸载")

        # 获取倾倒位置
        dump_position = context.task_params.get("dump_position", 135.0)

        # 翻斗到倾倒位置
        await context.robot_controller.set_servo_angle("dump", dump_position)
        await asyncio.sleep(1.0)  # 等待倾倒完成

        # 复位
        await context.robot_controller.set_servo_angle("dump", 0.0)
        await context.robot_controller.set_servo_angle("lift", 0.0)
        await asyncio.sleep(0.5)

        logger.info("翻斗卸载完成")
        return NodeStatus.SUCCESS

    return ActionNode("DumpAction", dump_action)


def create_check_cycle_limit_node() -> ConditionNode:
    """创建检查循环次数条件节点。

    Returns:
        SUCCESS if 未达到上限 (继续循环)
        FAILURE if 达到上限 (停止循环)
    """

    def check_condition(context: NodeContext) -> bool:
        current = context.current_cycle
        max_cycles = context.max_cycles

        can_continue = current < max_cycles

        logger.info(
            f"循环次数检查: {current}/{max_cycles}, 继续={can_continue}"
        )

        return can_continue

    return ConditionNode("CheckCycleLimit", check_condition)


# 导入 asyncio
import asyncio

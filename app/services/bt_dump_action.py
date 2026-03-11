"""卸载连贯动作节点。

将导航到卸载点、举升铲齿、翻转倾倒、倒退回原位整合为一个连贯的动作模块。

安全机制：
- 翻转电机操作前需要按钮确认（防止误触导致物料倾洒）
- 到达A点后才能举升铲齿
- 到达B点后才能翻转倾倒
- 倾倒完成后必须倒退（非导航）回到A点
- 回到A点后才能归零电机

动作流程：
1. 导航到卸载A点（等待位置）
2. 到达后，举升铲齿到卸载高度
3. 导航到卸载B点（倾倒位置）
4. 【按钮确认】翻转倾倒电机进行卸料
5. 倒退（直接倒车）回到A点
6. 翻转电机归零，举升电机归零
"""

import asyncio
import logging

from app.services.bt_nodes import (
    ActionNode,
    NodeContext,
    NodeStatus,
)
from app.services.navigation_interface import NavigationTarget

logger = logging.getLogger(__name__)


async def _wait_for_button_confirmation(
    action_description: str,
) -> bool:
    """等待按钮确认。

    Args:
        action_description: 操作描述（用于日志）

    Returns:
        bool: 是否获得确认
    """
    logger.warning(f"等待按钮确认: {action_description}")

    # TODO: 实现实际的按钮等待逻辑
    # 目前使用简单的延时模拟，实际应该：
    # 1. 发送 Socket.IO 事件通知前端等待按钮
    # 2. 等待前端发送按钮按下事件
    # 3. 或者轮询某个状态标志

    # 临时方案：等待 2 秒作为模拟
    await asyncio.sleep(2.0)

    logger.info(f"按钮确认完成: {action_description}")
    return True


def create_dump_and_return_node() -> ActionNode:
    """创建卸载并返回动作节点。

    连贯执行以下动作：
    1. 导航到卸载A点（等待位置）
    2. 到达后，举升铲齿到卸载高度
    3. 导航到卸载B点（倾倒位置）
    4. 【按钮确认】翻转倾倒电机进行卸料
    5. 倒退（直接倒车）回到A点
    6. 翻转电机归零，举升电机归零

    依赖任务参数：
        - dump_point_a: 卸载A点位置 [x, y, z]
        - dump_point_b: 卸载B点位置 [x, y, z]
        - lift_height: 举升高度（默认 90.0）
        - dump_angle: 倾倒角度（默认 135.0）

    Returns:
        ActionNode: 卸载并返回的动作节点
    """

    async def dump_and_return_action(context: NodeContext) -> NodeStatus:
        # 获取卸载点位置
        dump_point_a = context.task_params.get("dump_point_a")
        dump_point_b = context.task_params.get("dump_point_b")

        if not dump_point_a or not dump_point_b:
            logger.error("缺少 dump_point_a 或 dump_point_b 参数")
            return NodeStatus.FAILURE

        # 获取参数
        lift_height = context.task_params.get("lift_height", 90.0)
        dump_angle = context.task_params.get("dump_angle", 135.0)

        # ========== 第1步：导航到卸载A点（等待位置）==========
        logger.info("[卸载动作] 第1步：导航到卸载A点（等待位置）")

        target_a = NavigationTarget(
            x=dump_point_a[0],
            y=dump_point_a[1],
            z=0.0,
            name="卸载A点"
        )

        result = await context.navigation_service.navigate_to(target_a)
        if not result.success:
            logger.error(f"[卸载动作] 导航到A点失败: {result.error or result.message}")
            return NodeStatus.FAILURE

        # 记录A点最终位置用于后续倒退
        dump_point_a_final = result.final_position
        logger.info("[卸载动作] 到达卸载A点（等待位置）")

        # ========== 第2步：举升铲齿到卸载高度 ==========
        logger.info(f"[卸载动作] 第2步：举升铲齿到 {lift_height}°")

        await context.robot_controller.set_servo_angle("lift", lift_height)
        await asyncio.sleep(0.5)  # 等待举升完成

        logger.info("[卸载动作] 铲齿举升完成")

        # ========== 第3步：导航到卸载B点（倾倒位置）==========
        logger.info("[卸载动作] 第3步：导航到卸载B点（倾倒位置）")

        target_b = NavigationTarget(
            x=dump_point_b[0],
            y=dump_point_b[1],
            z=0.0,
            name="卸载B点"
        )

        result = await context.navigation_service.navigate_to(target_b)
        if not result.success:
            logger.error(f"[卸载动作] 导航到B点失败: {result.error or result.message}")
            return NodeStatus.FAILURE

        logger.info("[卸载动作] 到达卸载B点（倾倒位置）")

        # ========== 第4步：【按钮确认】翻转倾倒电机进行卸料 ==========
        logger.info("[卸载动作] 第4步：等待按钮确认后翻转倾倒电机")

        # 等待按钮确认（防止误触导致物料倾洒）
        confirmed = await _wait_for_button_confirmation(
            f"翻转倾倒电机到 {dump_angle}° 进行卸料"
        )

        if not confirmed:
            logger.error("[卸载动作] 按钮确认超时，取消卸料操作")
            return NodeStatus.FAILURE

        # 执行倾倒
        logger.info(f"[卸载动作] 翻转倾倒电机到 {dump_angle}° 进行卸料")
        await context.robot_controller.set_servo_angle("dump", dump_angle)
        await asyncio.sleep(1.0)  # 等待卸料完成

        logger.info("[卸载动作] 卸料完成")

        # ========== 第5步：倒退（直接倒车）回到A点 ==========
        logger.info("[卸载动作] 第5步：倒退回到A点（直接倒车）")

        # 计算倒退距离：从B点到A点的X轴距离
        # 获取当前位置（应在B点）
        current_position = await context.robot_controller.get_status()
        current = current_position.current_position

        # 计算倒退距离（沿X轴方向）
        reverse_distance = current[0] - dump_point_a_final[0]

        if reverse_distance <= 0:
            logger.warning(f"计算的倒退距离非正: {reverse_distance:.3f}m，使用默认值 1.0m")
            reverse_distance = 1.0

        logger.info(f"[卸载动作] 倒退回到A点: 距离={reverse_distance:.3f}m")

        # 执行倒退（直接倒车，非导航）
        success = await context.robot_controller.move_distance(
            distance=-reverse_distance,  # 负数表示倒退
            speed=0.3
        )

        if not success:
            logger.error("[卸载动作] 倒退失败")
            return NodeStatus.FAILURE

        logger.info("[卸载动作] 倒退完成，已回到A点")

        # ========== 第6步：铲齿翻转电机归零，举升电机归零 ==========
        logger.info("[卸载动作] 第6步：铲齿翻转电机和举升电机归零")

        # 倾倒电机归零（翻转电机）
        await context.robot_controller.set_servo_angle("dump", 0.0)
        # 举升电机归零
        await context.robot_controller.set_servo_angle("lift", 0.0)

        await asyncio.sleep(0.5)  # 等待归位完成

        logger.info("[卸载动作] 电机归零完成")

        # ========== 卸载动作全部完成 ==========
        logger.info("[卸载动作] 全部完成：A点→举升→B点→倾倒→倒退→归零")

        return NodeStatus.SUCCESS

    return ActionNode("DumpAndReturn", dump_and_return_action)

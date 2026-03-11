"""铲糖循环任务。

将铲糖行为树集成到任务系统中，支持任务调度、进度追踪和状态管理。
"""

import asyncio
import logging
from typing import Any, Dict, Optional

from app.models.task import TaskResult, TaskStatus
from app.services.tasks.base_task import BaseTask
from app.services.tasks.registry import register_task
from app.services.robot_controller import get_robot_controller
from app.services.mock_navigation import get_mock_navigation
from app.services.distance_analyzer import get_distance_analyzer
from app.services.behavior_tree_engine import create_sugar_harvest_engine, BehaviorTreeEngine

logger = logging.getLogger(__name__)


@register_task
class SugarHarvestTask(BaseTask):
    """铲糖自主循环任务。

    使用行为树控制机器人完成以下流程：
    1. 导航到取糖点
    2. RealSense 分析糖堆距离和高度
    3. 检查糖堆高度（< 20cm 时切换推垛模式）
    4. 前进铲糖
    5. 原路倒退
    6. 导航到卸载点
    7. 翻斗卸载
    8. 重复直到达到最大循环次数
    """

    task_type = "sugar_harvest"
    name = "铲糖自主循环"
    description = "使用行为树控制机器人自动铲取糖堆并运输到卸载点"
    category = "robot"
    requires_device = True  # 需要 RealSense 设备进行距离分析

    params_schema = {
        "type": "object",
        "properties": {
            "navigation_point": {
                "type": "array",
                "items": {"type": "number"},
                "minItems": 2,
                "maxItems": 2,
                "description": "取糖导航点 [x, y]"
            },
            "dump_point": {
                "type": "array",
                "items": {"type": "number"},
                "minItems": 2,
                "maxItems": 2,
                "description": "卸载点 [x, y]"
            },
            "max_cycles": {
                "type": "number",
                "default": 10,
                "minimum": 1,
                "maximum": 100,
                "description": "最大循环次数"
            },
            "height_threshold_m": {
                "type": "number",
                "default": 0.20,
                "minimum": 0.05,
                "maximum": 1.0,
                "description": "切换推垛模式的高度阈值（米）"
            },
            "bucket_width_m": {
                "type": "number",
                "default": 0.6,
                "description": "铲斗宽度（米）"
            },
            "approach_offset_m": {
                "type": "number",
                "default": 0.05,
                "description": "到达距离前的偏移量"
            },
            "scoop_position": {
                "type": "number",
                "default": 90.0,
                "minimum": 0,
                "maximum": 180,
                "description": "铲取位置（度）"
            },
            "dump_position": {
                "type": "number",
                "default": 135.0,
                "minimum": 0,
                "maximum": 180,
                "description": "倾倒位置（度）"
            },
        },
        "required": ["navigation_point", "dump_point"],
    }

    def __init__(
        self,
        task_id: str,
        config,
        params: Dict[str, Any],
        device_id: Optional[str] = None
    ):
        super().__init__(task_id, config, params, device_id)

        # 提取参数
        self.navigation_point = params.get("navigation_point")
        self.dump_point = params.get("dump_point")
        self.max_cycles = params.get("max_cycles", 10)
        self.height_threshold_m = params.get("height_threshold_m", 0.20)
        self.bucket_width_m = params.get("bucket_width_m", 0.6)
        self.approach_offset_m = params.get("approach_offset_m", 0.05)
        self.scoop_position = params.get("scoop_position", 90.0)
        self.dump_position = params.get("dump_position", 135.0)

        # 行为树引擎
        self._bt_engine: Optional[BehaviorTreeEngine] = None

    def validate(self) -> bool:
        """验证任务参数。"""
        if not self.navigation_point or len(self.navigation_point) != 2:
            raise ValueError("navigation_point 必须是包含 2 个元素的数组 [x, y]")

        if not self.dump_point or len(self.dump_point) != 2:
            raise ValueError("dump_point 必须是包含 2 个元素的数组 [x, y]")

        if self.max_cycles < 1 or self.max_cycles > 100:
            raise ValueError("max_cycles 必须在 1-100 之间")

        if self.height_threshold_m < 0.05 or self.height_threshold_m > 1.0:
            raise ValueError("height_threshold_m 必须在 0.05-1.0 之间")

        return True

    def setup(self):
        """任务初始化。"""
        logger.info(f"铲糖任务 {self.task_id} 初始化")

        # 获取服务实例
        self.robot_controller = get_robot_controller()
        self.navigation_service = get_mock_navigation()
        self.distance_analyzer = get_distance_analyzer()

        # 配置距离分析器
        self.distance_analyzer.set_bucket_width(self.bucket_width_m)
        self.distance_analyzer.set_height_threshold(self.height_threshold_m)

    async def run(self) -> TaskResult:
        """执行铲糖任务。"""
        self.update_progress(
            current_step=0,
            total_steps=self.max_cycles,
            message="初始化铲糖自主循环..."
        )

        try:
            # 构建任务参数
            task_params = {
                "navigation_point": self.navigation_point,
                "dump_point": self.dump_point,
                "max_cycles": self.max_cycles,
                "height_threshold_m": self.height_threshold_m,
                "bucket_width_m": self.bucket_width_m,
                "approach_offset_m": self.approach_offset_m,
                "scoop_position": self.scoop_position,
                "dump_position": self.dump_position,
            }

            # 创建行为树引擎
            self._bt_engine = create_sugar_harvest_engine(
                robot_controller=self.robot_controller,
                navigation_service=self.navigation_service,
                distance_analyzer=self.distance_analyzer,
                task_params=task_params,
            )

            # 启动行为树
            self.update_progress(message="启动铲糖行为树...")

            # 监控行为树执行并更新进度
            final_status = await self._run_with_progress_update()

            # 获取执行结果
            context = self._bt_engine.context

            if final_status.value == "success":
                return TaskResult(
                    success=True,
                    message=f"铲糖循环完成，共执行 {context.current_cycle} 次",
                    data={
                        "cycles_completed": context.current_cycle,
                        "final_sugar_height": context.sugar_height,
                        "switched_to_push_mode": context.blackboard.get(
                            "switch_to_push_mode", False
                        ),
                    },
                )
            else:
                return TaskResult(
                    success=False,
                    message=f"铲糖循环未正常完成: {final_status}",
                    data={
                        "cycles_completed": context.current_cycle,
                        "final_status": final_status.value,
                    },
                )

        except Exception as e:
            logger.error(f"铲糖任务执行失败: {e}", exc_info=True)
            return TaskResult(
                success=False,
                message="铲糖任务执行失败",
                error=str(e),
            )

    async def _run_with_progress_update(self):
        """运行行为树并更新进度。"""
        from app.services.bt_nodes import NodeStatus

        # 启动行为树监控任务
        monitor_task = asyncio.create_task(self._monitor_progress())

        try:
            # 运行行为树
            result = await self._bt_engine.start()
            return result

        finally:
            # 取消监控任务
            monitor_task.cancel()
            try:
                await monitor_task
            except asyncio.CancelledError:
                pass

    async def _monitor_progress(self):
        """监控行为树进度并更新任务进度。"""
        last_cycle = 0

        while True:
            await asyncio.check()
            await asyncio.sleep(0.5)

            if self._bt_engine is None:
                break

            context = self._bt_engine.context

            # 循环次数变化时更新进度
            if context.current_cycle != last_cycle:
                last_cycle = context.current_cycle

                # 判断是否需要切换推垛模式
                needs_push_mode = context.sugar_height < context.height_threshold

                status_msg = (
                    f"执行循环 {last_cycle}/{self.max_cycles}, "
                    f"糖堆高度: {context.sugar_height * 100:.1f}cm"
                )

                if needs_push_mode:
                    status_msg += " (即将切换推垛模式)"

                self.update_progress(
                    current_step=last_cycle,
                    total_steps=self.max_cycles,
                    message=status_msg,
                )

    def teardown(self):
        """清理资源。"""
        if self._bt_engine:
            self._bt_engine.reset()
        logger.info(f"铲糖任务 {self.task_id} 清理完成")

    async def stop(self):
        """停止任务。"""
        super().stop()
        if self._bt_engine and self._bt_engine.is_running:
            await self._bt_engine.stop()

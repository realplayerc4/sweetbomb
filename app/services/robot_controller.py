"""机器人控制器。

支持履带式机器人的运动控制（前后左右）和伺服控制（铲斗举升、翻斗）。
当前为模拟模式，预留真实硬件扩展点。
"""

import asyncio
import logging
from enum import Enum
from typing import Dict, Optional, Tuple
from dataclasses import dataclass, field
from datetime import datetime
import threading

logger = logging.getLogger(__name__)


class RobotState(str, Enum):
    """机器人状态枚举。"""
    IDLE = "idle"
    MOVING = "moving"
    SCOOPING = "scooping"
    DUMPING = "dumping"
    ERROR = "error"
    EMERGENCY_STOP = "emergency_stop"


class MoveDirection(str, Enum):
    """移动方向枚举。"""
    FORWARD = "forward"
    BACKWARD = "backward"
    LEFT = "left"
    RIGHT = "right"
    ROTATE_LEFT = "rotate_left"
    ROTATE_RIGHT = "rotate_right"
    STOP = "stop"


@dataclass
class ServoState:
    """伺服状态。"""
    servo_id: str
    name: str
    current_angle: float = 0.0  # 当前角度（度）
    target_angle: float = 0.0   # 目标角度（度）
    is_moving: bool = False


@dataclass
class RobotStatus:
    """机器人状态。"""
    state: RobotState
    battery_level: float
    current_position: Tuple[float, float, float]
    orientation: Tuple[float, float, float]  # Roll, Pitch, Yaw (度)
    left_track_speed: float
    right_track_speed: float
    servos: Dict[str, ServoState] = field(default_factory=dict)
    timestamp: datetime = field(default_factory=datetime.now)


class RobotController:
    """机器人控制器（模拟模式）。

    功能：
    - 履带式运动控制（前后左右、原地旋转）
    - 伺服电机控制（铲斗举升、翻斗）
    - 状态查询
    - 紧急停止

    真实硬件就绪后，可通过替换硬件接口实现类来对接。
    """

    _instance: Optional["RobotController"] = None
    _lock = threading.Lock()

    def __new__(cls):
        """单例模式。"""
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        """初始化机器人控制器。"""
        if hasattr(self, "_initialized"):
            return

        self._state = RobotState.IDLE
        self._position = [0.0, 0.0, 0.0]
        self._orientation = [0.0, 0.0, 0.0]  # Roll, Pitch, Yaw
        self._left_track_speed = 0.0
        self._right_track_speed = 0.0
        self._battery_level = 100.0

        # 伺服状态
        self._servos: Dict[str, ServoState] = {
            "lift": ServoState(servo_id="lift", name="铲斗举升", current_angle=0.0),
            "dump": ServoState(servo_id="dump", name="翻斗", current_angle=0.0),
        }

        self._emergency_stop = False
        self._move_task: Optional[asyncio.Task] = None
        self._lock = asyncio.Lock()

        self._initialized = True
        logger.info("机器人控制器初始化完成（模拟模式）")

    @classmethod
    def get_instance(cls) -> "RobotController":
        """获取单例实例。"""
        return cls()

    async def move(
        self,
        direction: MoveDirection,
        speed: float = 0.5,
        duration: float = 1.0
    ) -> bool:
        """移动机器人。

        Args:
            direction: 移动方向
            speed: 速度 (0.0 - 1.0)
            duration: 持续时间（秒）

        Returns:
            bool: 是否成功执行
        """
        if self._emergency_stop:
            logger.warning("紧急停止已激活，无法执行移动指令")
            return False

        async with self._lock:
            if self._state == RobotState.MOVING:
                logger.warning("机器人正在移动中")
                return False

            self._state = RobotState.MOVING
            logger.info(f"开始移动: 方向={direction}, 速度={speed}, 持续时间={duration}s")

            try:
                # 根据方向设置履带速度
                if direction == MoveDirection.FORWARD:
                    self._left_track_speed = speed
                    self._right_track_speed = speed
                elif direction == MoveDirection.BACKWARD:
                    self._left_track_speed = -speed
                    self._right_track_speed = -speed
                elif direction == MoveDirection.LEFT:
                    self._left_track_speed = speed * 0.5
                    self._right_track_speed = speed
                elif direction == MoveDirection.RIGHT:
                    self._left_track_speed = speed
                    self._right_track_speed = speed * 0.5
                elif direction == MoveDirection.ROTATE_LEFT:
                    self._left_track_speed = -speed * 0.5
                    self._right_track_speed = speed * 0.5
                elif direction == MoveDirection.ROTATE_RIGHT:
                    self._left_track_speed = speed * 0.5
                    self._right_track_speed = -speed * 0.5
                elif direction == MoveDirection.STOP:
                    self._left_track_speed = 0.0
                    self._right_track_speed = 0.0

                # 模拟移动过程
                steps = 10
                step_duration = duration / steps

                for _ in range(steps):
                    await asyncio.sleep(step_duration)
                    await self._update_position(direction, speed, step_duration)

                    if self._emergency_stop:
                        break

                # 停止
                self._left_track_speed = 0.0
                self._right_track_speed = 0.0

                if self._emergency_stop:
                    self._state = RobotState.EMERGENCY_STOP
                    logger.warning("移动过程中触发紧急停止")
                else:
                    self._state = RobotState.IDLE
                    logger.info(f"移动完成: 当前位置 {self._position}")

                return True

            except Exception as e:
                logger.error(f"移动失败: {e}")
                self._state = RobotState.ERROR
                return False

    async def _update_position(
        self, direction: MoveDirection, speed: float, dt: float
    ):
        """更新位置（模拟）。"""
        move_distance = speed * dt

        if direction == MoveDirection.FORWARD:
            # 假设机器人朝向 X 轴正方向
            self._position[0] += move_distance
        elif direction == MoveDirection.BACKWARD:
            self._position[0] -= move_distance
        elif direction == MoveDirection.LEFT:
            self._position[1] -= move_distance * 0.5
        elif direction == MoveDirection.RIGHT:
            self._position[1] += move_distance * 0.5
        elif direction == MoveDirection.ROTATE_LEFT:
            self._orientation[2] -= move_distance * 10  # Yaw 旋转
        elif direction == MoveDirection.ROTATE_RIGHT:
            self._orientation[2] += move_distance * 10

        # 角度归一化
        self._orientation[2] = (self._orientation[2] + 180) % 360 - 180

    async def stop(self) -> bool:
        """紧急停止。

        Returns:
            bool: 是否成功停止
        """
        logger.warning("触发紧急停止")
        self._emergency_stop = True
        self._left_track_speed = 0.0
        self._right_track_speed = 0.0
        self._state = RobotState.EMERGENCY_STOP
        return True

    async def reset_emergency_stop(self) -> bool:
        """重置紧急停止状态。

        Returns:
            bool: 是否成功重置
        """
        logger.info("重置紧急停止状态")
        self._emergency_stop = False
        self._state = RobotState.IDLE
        return True

    async def set_servo_angle(
        self, servo_id: str, angle: float
    ) -> bool:
        """设置伺服角度。

        Args:
            servo_id: 伺服 ID ("lift" 或 "dump")
            angle: 目标角度（度）

        Returns:
            bool: 是否成功设置
        """
        if servo_id not in self._servos:
            logger.error(f"未知的伺服 ID: {servo_id}")
            return False

        servo = self._servos[servo_id]

        # 角度限制
        angle = max(0.0, min(180.0, angle))

        logger.info(f"设置伺服 {servo.name}: {angle:.1f}°")
        servo.target_angle = angle
        servo.is_moving = True

        # 模拟伺服运动（每秒 60 度）
        start_angle = servo.current_angle
        angle_diff = angle - start_angle
        duration = abs(angle_diff) / 60.0
        steps = 10
        step_duration = duration / steps

        for i in range(steps):
            await asyncio.sleep(step_duration)
            ratio = (i + 1) / steps
            servo.current_angle = start_angle + angle_diff * ratio

        servo.current_angle = angle
        servo.is_moving = False
        logger.info(f"伺服 {servo.name} 到达目标角度")
        return True

    async def scoop(self) -> bool:
        """执行铲取动作。

        流程：铲斗举升到铲取位置

        Returns:
            bool: 是否成功执行
        """
        self._state = RobotState.SCOOPING
        logger.info("执行铲取动作")

        # 铲斗举升到铲取位置
        await self.set_servo_angle("lift", 90.0)
        await asyncio.sleep(0.5)

        self._state = RobotState.IDLE
        logger.info("铲取动作完成")
        return True

    async def dump(self) -> bool:
        """执行倾倒动作。

        流程：翻斗到倾倒位置，然后复位

        Returns:
            bool: 是否成功执行
        """
        self._state = RobotState.DUMPING
        logger.info("执行倾倒动作")

        # 翻斗到倾倒位置
        await self.set_servo_angle("dump", 135.0)
        await asyncio.sleep(1.0)

        # 复位
        await self.set_servo_angle("dump", 0.0)
        await self.set_servo_angle("lift", 0.0)
        await asyncio.sleep(0.5)

        self._state = RobotState.IDLE
        logger.info("倾倒动作完成")
        return True
    async def dock(self) -> bool:
        """执行回桩动作。

        流程：机器人回到原点 [0,0,0]，复位所有伺服。

        Returns:
            bool: 是否成功执行
        """
        self._state = RobotState.MOVING
        logger.info("执行回桩对准...")

        # 1. 复位伺服
        await self.set_servo_angle("dump", 0.0)
        await self.set_servo_angle("lift", 0.0)
        
        # 2. 模拟移动回到原点
        dist_to_origin = (self._position[0]**2 + self._position[1]**2)**0.5
        if dist_to_origin > 0:
            logger.info(f"正在从 {self._position} 回到原点...")
            # 简单模拟：分 5 步回到 [0,0,0]
            for i in range(5):
                await asyncio.sleep(0.4)
                ratio = (i + 1) / 5
                self._position[0] *= (1 - ratio)
                self._position[1] *= (1 - ratio)
            
        self._position = [0.0, 0.0, 0.0]
        self._orientation = [0.0, 0.0, 0.0]
        
        self._state = RobotState.IDLE
        logger.info("回桩对准完成，已就位。")
        return True


    def get_status(self) -> RobotStatus:
        """获取机器人状态。

        Returns:
            RobotStatus: 当前状态
        """
        return RobotStatus(
            state=self._state,
            battery_level=self._battery_level,
            current_position=tuple(self._position),
            orientation=tuple(self._orientation),
            left_track_speed=self._left_track_speed,
            right_track_speed=self._right_track_speed,
            servos=self._servos.copy(),
            timestamp=datetime.now(),
        )

    async def move_distance(
        self, distance: float, speed: float = 0.5
    ) -> bool:
        """移动指定距离。

        Args:
            distance: 距离（米），正数为前进，负数为后退
            speed: 速度 (0.0 - 1.0)

        Returns:
            bool: 是否成功执行
        """
        direction = MoveDirection.FORWARD if distance >= 0 else MoveDirection.BACKWARD
        duration = abs(distance) / speed if speed > 0 else 1.0
        return await self.move(direction, speed, duration)


# 全局访问函数
def get_robot_controller() -> RobotController:
    """获取机器人控制器单例。"""
    return RobotController.get_instance()

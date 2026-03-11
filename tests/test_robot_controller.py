"""
机器人控制器单元测试

本模块测试机器人控制器的核心功能，包括：
- 状态管理
- 移动控制（前进/后退）
- 紧急停止
- 伺服控制
- 动作执行（铲取/倾倒）
- 距离移动
"""

import pytest
import asyncio
from app.services.robot_controller import (
    RobotController,
    RobotState,
    MoveDirection,
    get_robot_controller,
)


@pytest.fixture
def robot_controller():
    """创建用于测试的机器人控制器实例"""
    controller = RobotController()
    # 重置状态
    controller._state = RobotState.IDLE
    controller._position = [0.0, 0.0, 0.0]
    controller._emergency_stop = False
    return controller


class TestRobotController:
    """机器人控制器测试套件"""

    def test_singleton_instance(self):
        """测试 get_robot_controller 返回相同实例（单例模式）"""
        controller1 = get_robot_controller()
        controller2 = get_robot_controller()
        assert controller1 is controller2

    def test_initial_state(self, robot_controller):
        """测试初始机器人状态"""
        status = robot_controller.get_status()
        assert status.state == RobotState.IDLE
        assert status.battery_level == 100.0
        assert status.current_position == (0.0, 0.0, 0.0)
        assert status.left_track_speed == 0.0
        assert status.right_track_speed == 0.0

    @pytest.mark.asyncio
    async def test_move_forward(self, robot_controller):
        """测试向前移动"""
        success = await robot_controller.move(
            direction=MoveDirection.FORWARD,
            speed=0.5,
            duration=0.1,
        )
        assert success is True
        status = robot_controller.get_status()
        assert status.state == RobotState.IDLE
        # X 位置应该增加了
        assert status.current_position[0] > 0

    @pytest.mark.asyncio
    async def test_move_backward(self, robot_controller):
        """测试向后移动"""
        success = await robot_controller.move(
            direction=MoveDirection.BACKWARD,
            speed=0.5,
            duration=0.1,
        )
        assert success is True
        status = robot_controller.get_status()
        assert status.current_position[0] < 0

    @pytest.mark.asyncio
    async def test_stop(self, robot_controller):
        """测试紧急停止"""
        success = await robot_controller.stop()
        assert success is True
        status = robot_controller.get_status()
        assert status.state == RobotState.EMERGENCY_STOP

    @pytest.mark.asyncio
    async def test_reset_emergency_stop(self, robot_controller):
        """测试重置紧急停止"""
        await robot_controller.stop()
        await robot_controller.reset_emergency_stop()
        status = robot_controller.get_status()
        assert status.state == RobotState.IDLE

    @pytest.mark.asyncio
    async def test_set_servo_angle(self, robot_controller):
        """测试设置伺服角度"""
        success = await robot_controller.set_servo_angle("lift", 90.0)
        assert success is True
        status = robot_controller.get_status()
        assert status.servos["lift"].current_angle == 90.0

    @pytest.mark.asyncio
    async def test_set_invalid_servo(self, robot_controller):
        """测试设置无效的伺服 ID"""
        success = await robot_controller.set_servo_angle("invalid", 90.0)
        assert success is False

    @pytest.mark.asyncio
    async def test_scoop_action(self, robot_controller):
        """测试铲取动作"""
        success = await robot_controller.scoop()
        assert success is True
        status = robot_controller.get_status()
        assert status.servos["lift"].current_angle == 90.0

    @pytest.mark.asyncio
    async def test_dump_action(self, robot_controller):
        """测试倾倒动作"""
        success = await robot_controller.dump()
        assert success is True
        status = robot_controller.get_status()
        # 倾倒后，伺服应该重置
        assert status.servos["dump"].current_angle == 0.0
        assert status.servos["lift"].current_angle == 0.0

    @pytest.mark.asyncio
    async def test_move_distance_positive(self, robot_controller):
        """测试移动正距离"""
        success = await robot_controller.move_distance(distance=0.5, speed=0.5)
        assert success is True
        status = robot_controller.get_status()
        assert status.current_position[0] > 0

    @pytest.mark.asyncio
    async def test_move_distance_negative(self, robot_controller):
        """测试移动负距离（向后）"""
        success = await robot_controller.move_distance(distance=-0.5, speed=0.5)
        assert success is True
        status = robot_controller.get_status()
        assert status.current_position[0] < 0

    @pytest.mark.asyncio
    async def test_emergency_stop_blocks_movement(self, robot_controller):
        """测试紧急停止阻移动"""
        await robot_controller.stop()
        success = await robot_controller.move(
            direction=MoveDirection fanc.FORWARD,
            speed=0.5,
            duration=0.1,
        )
        assert success is False

"""Unit tests for the robot controller."""

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
    """Create a robot controller instance for testing."""
    controller = RobotController()
    # Reset state
    controller._state = RobotState.IDLE
    controller._position = [0.0, 0.0, 0.0]
    controller._emergency_stop = False
    return controller


class TestRobotController:
    """Test suite for RobotController."""

    def test_singleton_instance(self):
        """Test that get_robot_controller returns the same instance."""
        controller1 = get_robot_controller()
        controller2 = get_robot_controller()
        assert controller1 is controller2

    def test_initial_state(self, robot_controller):
        """Test initial robot state."""
        status = robot_controller.get_status()
        assert status.state == RobotState.IDLE
        assert status.battery_level == 100.0
        assert status.current_position == (0.0, 0.0, 0.0)
        assert status.left_track_speed == 0.0
        assert status.right_track_speed == 0.0

    @pytest.mark.asyncio
    async def test_move_forward(self, robot_controller):
        """Test moving forward."""
        success = await robot_controller.move(
            direction=MoveDirection.FORWARD,
            speed=0.5,
            duration=0.1,
        )
        assert success is True
        status = robot_controller.get_status()
        assert status.state == RobotState.IDLE
        # X position should have increased
        assert status.current_position[0] > 0

    @pytest.mark.asyncio
    async def test_move_backward(self, robot_controller):
        """Test moving backward."""
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
        """Test emergency stop."""
        success = await robot_controller.stop()
        assert success is True
        status = robot_controller.get_status()
        assert status.state == RobotState.EMERGENCY_STOP

    @pytest.mark.asyncio
    async def test_reset_emergency_stop(self, robot_controller):
        """Test resetting emergency stop."""
        await robot_controller.stop()
        await robot_controller.reset_emergency_stop()
        status = robot_controller.get_status()
        assert status.state == RobotState.IDLE

    @pytest.mark.asyncio
    async def test_set_servo_angle(self, robot_controller):
        """Test setting servo angle."""
        success = await robot_controller.set_servo_angle("lift", 90.0)
        assert success is True
        status = robot_controller.get_status()
        assert status.servos["lift"].current_angle == 90.0

    @pytest.mark.asyncio
    async def test_set_invalid_servo(self, robot_controller):
        """Test setting invalid servo ID."""
        success = await robot_controller.set_servo_angle("invalid", 90.0)
        assert success is False

    @pytest.mark.asyncio
    async def test_scoop_action(self, robot_controller):
        """Test scoop action."""
        success = await robot_controller.scoop()
        assert success is True
        status = robot_controller.get_status()
        assert status.servos["lift"].current_angle == 90.0

    @pytest.mark.asyncio
    async def test_dump_action(self, robot_controller):
        """Test dump action."""
        success = await robot_controller.dump()
        assert success is True
        status = robot_controller.get_status()
        # After dump, servos should be reset
        assert status.servos["dump"].current_angle == 0.0
        assert status.servos["lift"].current_angle == 0.0

    @pytest.mark.asyncio
    async def test_move_distance_positive(self, robot_controller):
        """Test moving positive distance."""
        success = await robot_controller.move_distance(distance=0.5, speed=0.5)
        assert success is True
        status = robot_controller.get_status()
        assert status.current_position[0] > 0

    @pytest.mark.asyncio
    async def test_move_distance_negative(self, robot_controller):
        """Test moving negative distance (backward)."""
        success = await robot_controller.move_distance(distance=-0.5, speed=0.5)
        assert success is True
        status = robot_controller.get_status()
        assert status.current_position[0] < 0

    @pytest.mark.asyncio
    async def test_emergency_stop_blocks_movement(self, robot_controller):
        """Test that emergency stop blocks movement."""
        await robot_controller.stop()
        success = await robot_controller.move(
            direction=MoveDirection.FORWARD,
            speed=0.5,
            duration=0.1,
        )
        assert success is False

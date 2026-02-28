"""Unit tests for behavior tree nodes."""

import pytest
import asyncio
from app.services.bt_nodes import (
    BTNode,
    NodeContext,
    NodeStatus,
    ActionNode,
    ConditionNode,
    SequenceNode,
    SelectorNode,
    RepeatNode,
)


@pytest.fixture
def mock_context():
    """Create a mock node context."""
    return NodeContext(
        robot_controller=None,
        navigation_service=None,
        distance_analyzer=None,
        task_params={},
        blackboard={},
        current_cycle=0,
        max_cycles=5,
        sugar_height=0.3,
        height_threshold=0.20,
    )


class TestActionNode:
    """Test suite for ActionNode."""

    @pytest.mark.asyncio
    async def test_action_success(self, mock_context):
        """Test action node returning success."""

        async def success_action(ctx):
            return NodeStatus.SUCCESS

        node = ActionNode("TestAction", success_action)
        status = await node.tick(mock_context)

        assert status == NodeStatus.SUCCESS
        assert node.status == NodeStatus.SUCCESS

    @pytest.mark.asyncio
    async def test_action_failure(self, mock_context):
        """Test action node returning failure."""

        async def failure_action(ctx):
            return NodeStatus.FAILURE

        node = ActionNode("TestAction", failure_action)
        status = await node.tick(mock_context)

        assert status == NodeStatus.FAILURE

    @pytest.mark.asyncio
    async def test_action_exception(self, mock_context):
        """Test action node handling exception."""

        async def exception_action(ctx):
            raise ValueError("Test error")

        node = ActionNode("TestAction", exception_action)
        status = await node.tick(mock_context)

        assert status == NodeStatus.FAILURE


class TestConditionNode:
    """Test suite for ConditionNode."""

    @pytest.mark.asyncio
    async def test_condition_true(self, mock_context):
        """Test condition node returning true."""

        def true_condition(ctx):
            return True

        node = ConditionNode("TestCondition", true_condition)
        status = await node.tick(mock_context)

        assert status == NodeStatus.SUCCESS

    @pytest.mark.asyncio
    async def test_condition_false(self, mock_context):
        """Test condition node returning false."""

        def false_condition(ctx):
            return False

        node = ConditionNode("TestCondition", false_condition)
        status = await node.tick(mock_context)

        assert status == NodeStatus.FAILURE


class TestSequenceNode:
    """Test suite for SequenceNode."""

    @pytest.mark.asyncio
    async def test_sequence_all_success(self, mock_context):
        """Test sequence node with all children succeeding."""

        async def success_action(ctx):
            return NodeStatus.SUCCESS

        children = [
            ActionNode(f"Action{i}", success_action)
            for i in range(3)
        ]
        node = SequenceNode("TestSequence", children)
        status = await node.tick(mock_context)

        assert status == NodeStatus.SUCCESS

    @pytest.mark.asyncio
    async def test_sequence_one_failure(self, mock_context):
        """Test sequence node with one child failing."""

        async def make_action(i):
            async def action(ctx):
                if i == 1:
                    return NodeStatus.FAILURE
                return NodeStatus.SUCCESS
            return action

        children = [
            ActionNode(f"Action{i}", make_action(i))
            for i in range(3)
        ]
        node = SequenceNode("TestSequence", children)
        status = await node.tick(mock_context)

        assert status == NodeStatus.FAILURE

    @pytest.mark.asyncio
    async def test_sequence_with_running(self, mock_context):
        """Test sequence node with a running child."""

        call_count = {"count": 0}

        async def mixed_action(ctx):
            call_count["count"] += 1
            if call_count["count"] == 1:
                return NodeStatus.RUNNING
            return NodeStatus.SUCCESS

        children = [
            ActionNode("RunningAction", mixed_action),
            ActionNode("SuccessAction", lambda ctx: asyncio.sleep(0) or NodeStatus.SUCCESS),
        ]
        node = SequenceNode("TestSequence", children)

        status = await node.tick(mock_context)
        assert status == NodeStatus.RUNNING


class TestSelectorNode:
    """Test suite for SelectorNode."""

    @pytest.mark.asyncio
    async def test_selector_first_success(self, mock_context):
        """Test selector node with first child succeeding."""

        async def success_action(ctx):
            return NodeStatus.SUCCESS

        async def failure_action(ctx):
            return NodeStatus.FAILURE

        children = [
            ActionNode("SuccessAction", success_action),
            ActionNode("FailureAction", failure_action),
        ]
        node = SelectorNode("TestSelector", children)
        status = await node.tick(mock_context)

        assert status == NodeStatus.SUCCESS

    @pytest.mark.asyncio
    async def test_selector_all_failure(self, mock_context):
        """Test selector node with all children failing."""

        async def failure_action(ctx):
            return NodeStatus.FAILURE

        children = [
            ActionNode(f"Action{i}", failure_action)
            for i in range(3)
        ]
        node = SelectorNode("TestSelector", children)
        status = await node.tick(mock_context)

        assert status == NodeStatus.FAILURE


class TestRepeatNode:
    """Test suite for RepeatNode."""

    @pytest.mark.asyncio
    async def test_repeat_with_limit(self, mock_context):
        """Test repeat node with execution limit."""

        async def success_action(ctx):
            return NodeStatus.SUCCESS

        child = ActionNode("SuccessAction", success_action)
        node = RepeatNode("TestRepeat", child, max_count=3)

        # First tick should return RUNNING
        status = await node.tick(mock_context)
        assert status == NodeStatus.RUNNING
        assert node._current_count == 1

        # Second tick
        status = await node.tick(mock_context)
        assert status == NodeStatus.RUNNING
        assert node._current_count == 2

        # Third tick
        status = await node.tick(mock_context)
        assert status == NodeStatus.RUNNING
        assert node._current_count == 3

        # Fourth tick should return SUCCESS (limit reached)
        status = await node.tick(mock_context)
        assert status == NodeStatus.SUCCESS

    @pytest.mark.asyncio
    async def test_repeat_with_child_failure(self, mock_context):
        """Test repeat node with child failure."""

        async def failure_action(ctx):
            return NodeStatus.FAILURE

        child = ActionNode("FailureAction", failure_action)
        node = RepeatNode("TestRepeat", child, max_count=3)

        status = await node.tick(mock_context)
        assert status == NodeStatus.FAILURE

    @pytest.mark.asyncio
    async def test_repeat_updates_context(self, mock_context):
        """Test that repeat node updates context cycle count."""

        async def success_action(ctx):
            ctx.current_cycle += 1
            return NodeStatus.SUCCESS

        child = ActionNode("SuccessAction", success_action)
        node = RepeatNode("TestRepeat", child, max_count=2)

        await node.tick(mock_context)
        await node.tick(mock_context)

        assert mock_context.current_cycle == 2

    def test_repeat_reset(self, mock_context):
        """Test repeat node reset."""

        async def success_action(ctx):
            return NodeStatus.SUCCESS

        child = ActionNode("SuccessAction", success_action)
        node = RepeatNode("TestRepeat", child, max_count=3)

        # Simulate some execution
        node._current_count = 2

        # Reset
        node.reset()

        assert node._current_count == 0
        assert node.status == NodeStatus.IDLE


class TestNodePath:
    """Test suite for node path tracking."""

    def test_node_path_without_parent(self):
        """Test node path for root node."""
        node = ActionNode("Root", lambda ctx: asyncio.sleep(0) or NodeStatus.SUCCESS)
        assert node.get_path() == "Root"

    def test_node_path_with_parent(self):
        """Test node path for child node."""
        parent = SequenceNode("Parent", [])
        child = ActionNode("Child", lambda ctx: asyncio.sleep(0) or NodeStatus.SUCCESS)

        # Manually set parent (normally done in SequenceNode.__init__)
        child._parent = parent

        assert child.get_path() == "Parent/Child"

    def test_nested_node_path(self):
        """Test node path for nested nodes."""
        root = SequenceNode("Root", [])
        parent = SequenceNode("Parent", [])
        child = ActionNode("Child", lambda ctx: asyncio.sleep(0) or NodeStatus.SUCCESS)

        child._parent = parent
        parent._parent = root

        assert child.get_path() == "Root/Parent/Child"

"""
行为树节点单元测试

本模块测试行为树引擎的核心功能，包括：
- 节点状态管理
- 操作节点执行
- 条件节点评估
- 复合节点（序列、选择器、重复）
- 节点路径跟踪
"""

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
    """创建 mock 节点上下文"""
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
    """操作节点测试套件"""

    @pytest.mark.asyncio
    async def test_action_success(self, mock_context):
        """测试返回成功的操作节点"""

        async def success_action(ctx):
            return NodeStatus.SUCCESS

        node = ActionNode("TestAction", success_action)
        status = await node.tick(mock_context)

        assert status == NodeStatus.SUCCESS
        assert node.status == NodeStatus.SUCCESS

    @pytest.mark.asyncio
    async def test_action_failure(self, mock_context):
        """测试返回失败的操作节点"""

        async def failure_action(ctx):
            return NodeStatus.FAILURE

        node = ActionNode("TestAction", failure_action)
        status = await node.tick(mock_context)

        assert status == NodeStatus.FAILURE

    @pytest.mark.asyncio
    async def test_action_exception(self, mock_context):
        """测试操作节点异常处理"""

        async def exception_action(ctx):
            raise ValueError("Test error")

        node = ActionNode("TestAction", exception_action)
        status = await node.tick(mock_context)

        assert status == NodeStatus.FAILURE


class TestConditionNode:
    """条件节点测试套件"""

    @pytest.mark.asyncio
    async def test_condition_true(self, mock_context):
        """测试返回 true 的条件节点"""

        def true_condition(ctx):
            return True

        node = ConditionNode("TestCondition", true_condition)
        status = await node.tick(mock_context)

        assert status == NodeStatus.SUCCESS

    @pytest.mark.asyncio
    async def test_condition_false(self, mock_context):
        """测试返回 false 的条件节点"""

        def false_condition(ctx):
            return False

        node = ConditionNode("TestCondition", false_condition)
        status = await node.tick(mock_context)

        assert status == NodeStatus.FAILURE


class TestSequenceNode:
    """序列节点测试套件

    序列节点按顺序执行所有子节点，
    只有所有子节点都成功时才返回成功。
    """

    @pytest.mark.asyncio
    async def test_sequence_all_success(self, mock_context):
        """测试所有子节点都成功的序列节点"""

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
        """测试有一个子节点失败的序列节点"""

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
        """测试有运行中子节点的序列节点"""

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
    """选择器节点测试套件

    选择器节点依次执行子节点，
    直到第一个子节点成功为止。
    """

    @pytest.mark.asyncio
    async def test_selector_first_success(self, mock_context):
        """测试第一个子节点成功的选择器节点"""

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
        """测试所有子节点都失败的选择器节点"""

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
    """重复节点测试套件

    重复节点按指定次数重复执行子节点。
    """

    @pytest.mark.asyncio
    async def test_repeat_with_limit(self, mock_context):
        """测试带有执行次数限制的重复节点"""

        async def success_action(ctx):
            return NodeStatus.SUCCESS

        child = ActionNode("SuccessAction", success_action)
        node = RepeatNode("TestRepeat", child, max_count=3)

        # 第一次 tick 应该返回 RUNNING
        status = await node.tick(mock_context)
        assert status == NodeStatus.RUNNING
        assert node._current_count == 1

        # 第二次 tick
        status = await node.tick(mock_context)
        assert status == NodeStatus.RUNNING
        assert node._current_count == 2

        # 第三次 tick
        status = await node.tick(mock_context)
        assert status == NodeStatus.RUNNING
        assert node._current_count == 3

        # 第四次 tick 应该返回 SUCCESS（达到限制）
        status = await node.tick(mock_context)
        assert status == NodeStatus.SUCCESS

    @pytest.mark.asyncio
    async def test_repeat_with_child_failure(self, mock_context):
        """测试子节点失败的重复节点"""

        async def failure_action(ctx):
            return NodeStatus.FAILURE

        child = ActionNode("FailureAction", failure_action)
        node = RepeatNode("TestRepeat", child, max_count=3)

        status = await node.tick(mock_context)
        assert status == NodeStatus.FAILURE

    @pytest.mark.asyncio
    async def test_repeat_updates_context(self, mock_context):
        """测试重复节点更新上下文周期计数"""

        async def success_action(ctx):
            ctx.current_cycle += 1
            return NodeStatus.SUCCESS

        child = ActionNode("SuccessAction", success_action)
            node = RepeatNode("TestRepeat", child, max_count=2)

        await node.tick(mock_context)
        await node.tick(mock_context)

        assert mock_context.current_cycle == 2

    def test_repeat_reset(self, mock_context):
        """测试重复节点重置"""

        async def success_action(ctx):
            return NodeStatus.SUCCESS

        child = ActionNode("SuccessAction", success_action)
        node = RepeatNode("TestRepeat", child, max_count=3)

        # 模拟一些执行
        node._current_count = 2

        # 重置
        node.reset()

        assert node._current_count == 0
        assert node.status == NodeStatus.IDLE


class TestNodePath:
    """节点路径跟踪测试套件"""

    def test_node_path_without_parent(self):
        """测试根节点的节点路径"""
        node = ActionNode("Root", lambda ctx: asyncio.sleep(0) or NodeStatus.SUCCESS)
        assert node.get_path() == "Root"

    def test_node_path_with_parent(self):
        """测试子节点的节点路径"""
        parent = SequenceNode("Parent", [])
        child = ActionNode("Child", lambda ctx: asyncio.sleep(0) or NodeStatus.SUCCESS)

        # 手动设置父节点（通常在 SequenceNode.__init__ 中完成）
        child._parent = parent

        assert child.get_path() == "Parent/Child"

    def test_nested_node_path(self):
        """测试嵌套节点的节点路径"""
        root = SequenceNode("Root", [])
        parent = SequenceNode("Parent", [])
        child = ActionNode("Child", lambda ctx: asyncio.sleep(0) or NodeStatus.SUCCESS)

        child._parent = parent
        parent._parent = root

        assert child.get_path() == "Root/Parent/Child"

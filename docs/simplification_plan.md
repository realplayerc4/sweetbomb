# 行为树简化计划

## 当前架构问题

当前行为树架构过于复杂：
- 上位机 Python 行为树节点过多
- 下位机只负责执行，逻辑在上位机
- 通信频繁，延迟较大

## 目标架构

将复杂逻辑下放到下位机，上位机只保留简单的任务触发：

```
┌─────────────────┐     简单任务指令     ┌─────────────────┐
│   上位机 (PC)   │ ──────────────────> │   下位机 (MCU)  │
│                 │                     │                 │
│  - 任务启动     │ <────────────────── │  - 自主决策     │
│  - 状态监控     │    状态/完成通知    │  - 动作执行     │
│  - 紧急停止     │                     │  - 异常处理     │
└─────────────────┘                     └─────────────────┘
```

## 具体修改

### 1. 删除文件 (待删除)
- `app/services/behavior_tree_engine.py` - 复杂行为树引擎
- `app/services/bt_nodes.py` - 基础节点类
- `app/services/bt_action_nodes.py` - 动作节点
- `app/services/bt_scoop_action.py` - 铲糖动作
- `app/services/bt_dump_action.py` - 卸载动作
- `app/services/bt_calculate_distance_node.py` - 距离计算

### 2. 简化后的核心接口

#### 后端 → 下位机 协议
```json
{
  "cmd": "start_task",
  "task_id": "task_001",
  "task_type": "sugar_harvest",
  "params": {
    "target_volume": 30,
    "max_cycles": 10,
    "sugar_point": [1.5, 0.5]
  }
}
```

#### 下位机 → 后端 状态
```json
{
  "task_id": "task_001",
  "status": "running",
  "current_step": "approach_sugar",
  "progress": 45,
  "error_code": null
}
```

### 3. 保留的最小化后端代码

- `app/services/simple_task_dispatcher.py` - 简单任务分发
- `app/services/serial_communication.py` - 串口通信 (与下位机)
- `app/api/endpoints/robot.py` - 简化后的 API

### 4. 前端调整

前端只需要：
- 任务启动/停止按钮
- 简单状态显示 (空闲/运行中/完成/错误)
- 进度百分比

删除复杂的行为树可视化。

## 实施步骤

1. **验证下位机能力** - 确认下位机可以处理完整逻辑
2. **创建简单任务接口** - 编写 `simple_task_dispatcher.py`
3. **逐步删除旧代码** - 按依赖顺序删除行为树相关文件
4. **更新前端** - 简化 UI 显示
5. **测试验证** - 完整流程测试

## 风险控制

- 保留 Git 历史，必要时可回滚
- 分阶段实施，每步都有可用版本
- 提前与下位机团队确认接口协议
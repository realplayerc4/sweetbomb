# 行为树简化方案

## 架构变更

### 当前架构（复杂）
```
上位机 Python 行为树
├── 导航到取糖点
├── 分析糖堆距离
├── 检查糖堆高度
├── 前进铲糖
├── 原路倒退
├── 导航到卸载点
├── 翻斗卸载
└── 检查循环次数
```

### 目标架构（简化）
```
上位机任务触发器
├── 任务1: 铲糖自主循环 (start_sugar_harvest)
└── 任务2: 推垛模式 (start_push_mode)
         ↓
    简单任务指令
         ↓
    下位机自主执行
```

## 接口设计

### 上位机 → 下位机：任务开始

```json
{
  "task_type": "sugar_harvest",
  "task_id": "uuid-xxx",
  "params": {
    "target_volume": 30,
    "max_cycles": 10,
    "navigation_point": [1.5, 0.5]
  },
  "timestamp": 1699999999
}
```

### 下位机 → 上位机：状态更新

```json
{
  "task_id": "uuid-xxx",
  "status": "running",
  "current_step": "navigating_to_sugar",
  "progress": 45,
  "details": {
    "current_position": [1.2, 0.4],
    "target_position": [1.5, 0.5],
    "remaining_distance": 0.3
  },
  "timestamp": 1699999999
}
```

## 后端简化

### 删除文件
- `app/services/behavior_tree_engine.py` - 复杂行为树引擎
- `app/services/bt_action_nodes.py` - 详细动作节点
- `app/services/bt_nodes.py` - 行为树节点基类
- `app/services/bt_*.py` - 其他行为树相关文件

### 保留文件
- `app/services/task_manager.py` - 任务管理（简化版）
- `app/services/robot_controller.py` - 机器人控制接口
- `app/services/navigation_service.py` - 导航服务（如果需要）

### 新增文件
- `app/services/simple_task_dispatcher.py` - 简单任务分发器

## 前端调整

### 简化显示
- 保留两个选项卡：铲糖主循环 / 推垛模式
- 显示当前任务状态：空闲 / 运行中 / 已完成 / 错误
- 显示当前步骤（下位机回传）
- 简单控制：启动 / 停止

### 移除复杂显示
- 移除行为树可视化
- 移除详细节点状态
- 移除复杂控制面板

## 实施步骤

1. **后端简化**
   - 创建 `simple_task_dispatcher.py`
   - 简化 `task_manager.py`
   - 删除旧的行为树文件
   - 更新 API 端点

2. **前端简化**
   - 简化 `BehaviorTreeViz.tsx`
   - 更新状态显示
   - 简化控制按钮

3. **测试验证**
   - 测试任务启动
   - 测试状态更新
   - 测试停止功能

## 风险与考虑

- **下位机能力**：确保下位机有足够能力处理复杂逻辑
- **调试难度**：简化后调试可能更困难（日志记录要做好）
- **回滚方案**：保留简化前的代码分支，以备需要回滚

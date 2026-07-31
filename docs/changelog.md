# 变更日志 (Changelog)

> 本文档记录所有显著变更、功能更新和问题修复。
> 遵循 [Keep a Changelog](https://keepachangelog.com/) 格式和 [Semantic Versioning](https://semver.org/)。

---

## 变更类型标签

| 标签 | 说明 |
|------|------|
| `[Added]` | 新功能 |
| `[Changed]` | 现有功能的变更 |
| `[Deprecated]` | 即将移除的功能 |
| `[Removed]` | 移除的功能 |
| `[Fixed]` | 问题修复 |
| `[Security]` | 安全修复 |
| `[Docs]` | 文档更新 |
| `[Refactor]` | 代码重构 |

---

## [Unreleased]

### [Added]

*暂无新功能*

### [Changed]

*暂无变更*

### [Fixed]

*暂无修复*

---

## [2.1.0] - 2026-07-31

### [Added]

- **YOLOv8 目标检测服务**
  - 新增 `app/services/yolo_detector.py` (480 行) - 非阻塞 ONNX 推理引擎
  - 支持 RGB 和深度流实时检测，daemon 线程运行
  - 环境变量配置：`YOLO_ENABLED`, `YOLO_MODEL_PATH`, `YOLO_CONFIDENCE`, `YOLO_MAX_FPS`
  - 模型路径默认 `/home/jetson/sweetbomb/models/yolov8n/best.onnx`
  - `models/yolov8n/.gitkeep` 模型占位目录

- **任务系统恢复**
  - 新增 `app/models/task.py` (121 行) - 任务数据模型 (TaskStatus, TaskPriority, TaskConfig, TaskInfo 等)
  - 新增 `app/api/endpoints/tasks.py` (243 行) - 任务 CRUD API 端点
  - 新增 `app/services/task_manager.py` (379 行) - 任务生命周期管理器 (单例模式)
  - 新增 `app/services/tasks/` 目录 - 任务实现包 (base_task.py, registry.py, implementations/)
  - 新增 `app/services/simple_task_dispatcher.py` (278 行) - 简化任务分发器

- **行为树系统恢复**
  - 恢复 `app/services/behavior_tree_engine.py` (292 行)
  - 恢复 `app/services/bt_nodes.py` (320 行)
  - 恢复 `app/services/bt_action_nodes.py` (402 行)
  - 恢复 `app/services/bt_dump_action.py` (203 行)
  - 恢复 `app/services/bt_scoop_action.py` (105 行)
  - 新增 `app/services/bt_calculate_distance_node.py` (60 行) - 距离计算节点

- **SWNFP 前端任务系统重构**
  - 重构 `useTaskStore` - 简化为重量目标+循环进度模型
  - 任务循环流程：nav_to_pick → 等待完成 → nav_to_drop → 等待完成 → 循环
  - 新增 `useMap.ts` 和 `usePathMap.ts` hooks
  - 新增 `mapApi.ts` 和 `pathMapApi.ts` 服务
  - 重构 `robotApi.ts` - 添加 pause/resume/navToPick/navToDrop 接口

- **SWNFP 前端地图组件重构**
  - `MapMonitor.tsx` 大幅重构 (672 行 diff) - 基于后端地图图片的可视化
  - 移除 `MapOverview.tsx` (合并到 MapMonitor)
  - 移除 `DraggableModule.tsx` (布局简化)

- **SWNFP 前端任务面板重构**
  - `TaskPanel.tsx` 重构 (631 行 diff) - 任务控制主界面
  - 移除 `TaskCreateModal.tsx` (功能合并到 TaskPanel)
  - 新增 `HarvestControl.tsx` - 铲糖循环控制面板

- **SWNFP 前端设备监控增强**
  - `WorkingDevices.tsx` 增强 (145 行 diff) - 1号装载机标记为"本机"
  - `SensorMonitor.tsx` 支持单机器人/多机器人模式切换
  - `DeviceStatsChart.tsx` 新增设备统计图表

- **新增后端服务**
  - `app/services/yolo_detector.py` - YOLOv8 检测服务
  - `app/services/simple_task_dispatcher.py` - 简化任务分发器
  - `app/services/bt_calculate_distance_node.py` - BT 距离计算节点

- **新增文档**
  - `docs/behavior_tree_analysis_report.md` (386 行) - 行为树后端代码分析报告
  - `docs/robot_communication_protocol.md` (200 行) - 铲糖机器人与上位机通讯协议

### [Changed]

- **深度距离阈值扩展**
  - 将深度距离阈值从 6m 扩展到 15m (`3fb1fdf6`)

- **camera_to_teeth 固定为物理值**
  - 将 `camera_to_teeth` 固定为 1m（物理固定值，不再从前端传入）(`8cbda6de`)

- **TCP 通讯协议完善**
  - 仅 taskFinish 消息回复下位机，其他消息不回复
  - 举升角度范围修正为 60-124 度（前端显示）
  - "铲齿深度"标签改为"相机高度"

- **部署方式变更**
  - 移除 PM2 配置，改用 systemd 服务
  - 适合嵌入式 Linux（Jetson）稳定运行
  - 更新 CLAUDE.md、README.md 文档

- **cameraCheck 距离发送行为变更**
  - 始终发送距离给下位机，包括 0 值（之前仅在 move_distance > 0 时发送）

### [Fixed]

- **move_distance 无物料时显示 0.3m**
  - 根因：仅检查 `material_distance is not None`，未检查 `has_material`
  - 当点云噪点落在工作范围内时，`material_distance=0.0` 导致 `move_distance=0+0.3=0.3`
  - 修复：增加 `has_material` 检查，无物料时 `move_distance=0.0`

- **camera_to_teeth 单位不一致导致点云过滤错误**
  - 根因：前端传入 mm（1020），点云坐标是 m，导致工作范围计算错误
  - 原 `min_x=1020, max_x=1022` 过滤掉了所有点云（实际坐标 0.5~2m）
  - 修复：在 `stream_controller.py` 中将 `camera_to_teeth` 从 mm 转换为 m

- **nearest_x 工作范围过滤错误**
  - 根因：X 范围从 `camera_to_teeth` 开始，过滤掉了相机到铲齿之间的物料点
  - 修复：X 范围改为 `camera_to_teeth + 0.3 ~ ∞`（从铲齿前方 0.3m 开始）

- **nearest_x 噪点干扰**
  - 问题：孤立噪点被误判为最近物料点
  - 修复：增加双重噪点过滤
    - 高度过滤：只保留 `Z >= z1 + 0.05m` 的点（过滤地面噪点）
    - 密度过滤：只保留半径 0.1m 内有 ≥5 个邻居点的点（过滤孤立噪点）

- **numpy 序列化错误**
  - `PointCloudAnalysisResponse` 中 `nearest_point` 转换为 Python float
  - `material_distance` 为 None 时不再触发格式化错误

- **点云分析 Volume 显示 0.00L**
  - 分析参数（teeth_height、camera_to_teeth）未从前端同步导致 ROI 计算错误
  - 需通过 settings API 同步前端参数

### [Removed]

- PM2 相关配置和命令文件（ecosystem.config.cjs、pm2-* 命令）
- 前端遗留组件：`MapOverview.tsx`, `DraggableModule.tsx`, `TaskCreateModal.tsx` (功能合并)

### [Docs]

- 更新 `docs/admin-frontend-architecture.md` - 重构 v2.0 版本，反映 SWNFP 前端实际代码结构
- 更新 `memory-bank/project-overview.md` - 修正项目名称、版本、行为树说明
- 更新 `memory-bank/architecture.md` - 修正目录结构、新增缺失模块说明
- 更新 `memory-bank/progress.md` - 补充 YOLOv8 和 SWNFP 合并里程碑

---

## [2.0.0] - 2026-03-20

### [Removed]

- **移除任务系统 (Task Management)**
  - 删除 `TaskManager` 及相关服务
  - 删除 `/api/tasks` 端点
  - 删除任务数据模型
  - 移除 Socket.IO 任务事件

- **移除行为树系统 (Behavior Tree)**
  - 删除 `bt_nodes.py` 行为树节点
  - 删除 `bt_action_nodes.py` 动作节点
  - 删除 `bt_dump_action.py` 倾倒动作
  - 删除 `bt_scoop_action.py` 铲糖动作
  - 删除 `behavior_tree_engine.py` 引擎

- **移除路径点系统 (Waypoints)**
  - 删除 `/api/waypoints` 端点
  - 删除路径点管理服务

- **移除机器人控制系统 (Robot Control)**
  - 删除 `/api/robot` 端点
  - 删除 `RobotTCPServer`
  - 删除机器人命令协议

### [Changed]

- 简化架构，专注 RealSense 视频流和点云处理
- WebRTC 成为主要视频传输方式
- 保留 Socket.IO 用于元数据推送

### [Migration Guide]

如需机器人控制功能，请使用独立的机器人控制服务。

---

## [1.2.0] - 2026-03-03

### [Added]

- **铲糖行为树完整实现**
  - `DumpAndReturn` 连贯动作节点（导航A点→举升→导航B点→倾倒→倒退→归零）
  - `ScoopAndReturn` 连贯动作节点（前进→翻转铲子→倒退）
  - `AnalyzeSugarDistance` 距离分析节点
  - `CheckShovelFlat` 车铲平整度检查节点
  - `ReturnToHome` 回桩充电节点

- **按钮确认安全机制**
  - 倾倒动作需要前端按钮确认
  - 30秒超时保护
  - Socket.IO 事件通知

- **IMU 数据处理**
  - 3D 姿态估计可视化
  - Roll/Pitch 实时计算
  - 重力向量可视化

- **3x2 完美网格布局**
  - 仪表盘黄金比例布局
  - 视觉中心偏移（+8px 黄金胶囊偏移）
  - 字重加粗优化
  - 品牌色 `#FD802E` 注入

### [Changed]

- **点云渲染优化**
  - 粒子大小调整为 4.5
  - 探测范围扩展至 6m
  - 使用 `NormalBlending` 避免 `AdditiveBlending` 过曝
  - `OrthographicCamera` 俯视图严格锁定视锥
  - `frustumCulled: false` 解决视角拉远点云消失问题

- **倾倒流程重构**
  - 废弃单一卸载点配置 (`dump_point`)
  - 引入双卸载点设计 (`dump_point_a` 和 `dump_point_b`)
  - A点：等待/归零位置，B点：倾倒位置
  - 新增位置保护：到达A点才能举升，到达B点才能倾倒
  - 新增归零保护：回到A点才能归零电机，防止空中归零导致物料掉落

- **结构优化**
  - 工程目录全链路物理分离
  - 前端组件模块化重构
  - 状态管理集中化

### [Fixed]

- 修复 Socket.IO 连接 403 错误
- 修复点云在俯视角下消失问题
- 修复铲糖任务高度阈值判断逻辑
- 修复倾倒动作安全确认超时问题

### [Refactor]

- `bt_nodes.py` 核心节点重设计
- `bt_scoop_action.py` 铲糖动作优化
- `bt_dump_action.py` 倾倒动作重构
- 前端组件解耦

### [Docs]

- 新增行为树规范文档
- 更新 API 接口文档
- 完善铲糖任务配置说明

---

## [1.1.0] - 2026-02-20

### [Added]

- **行为树基础框架**
  - `BTNode` 基类设计
  - `ActionNode` 动作节点
  - `ConditionNode` 条件节点
  - `SequenceNode` 序列节点
  - `SelectorNode` 选择节点
  - `RepeatNode` 重复节点
  - `DecoratorNode` 装饰器节点
  - `InverterNode` 反转节点

- **3D 点云可视化**
  - Three.js 集成
  - 深度图转点云
  - 实时渲染
  - 性能优化

- **任务管理系统**
  - 任务状态机
  - 并发控制
  - 进度追踪

### [Changed]

- 前端架构重构
- 组件库升级
- 状态管理优化

### [Fixed]

- WebRTC 连接稳定性
- 内存泄漏问题

---

## [1.0.0] - 2026-02-10

### [Added]

- **基础架构**
  - FastAPI 后端框架
  - React 前端框架
  - WebRTC 视频流
  - Socket.IO 实时通信

- **RealSense 集成**
  - 设备发现与枚举
  - 流启动/停止
  - 传感器选项配置
  - IMU 数据读取

- **Web 界面**
  - 视频流显示
  - 设备配置面板
  - 参数调节滑块
  - 状态监控

### [Security]

- CORS 配置
- 输入验证
- 错误信息脱敏

---

## 版本历史

```
v2.1.0 (2026-07-31)  YOLOv8检测, 任务系统恢复, 行为树恢复, SWNFP前端重构
v2.0.0 (2026-03-20)  移除任务/行为树/路径点/机器人控制, 简化架构
v1.2.0 (2026-03-03)  铲糖行为树完整实现，倾倒流程重构，UI 黄金版本
v1.1.0 (2026-02-20)  行为树基础框架，3D 点云可视化，任务管理系统
v1.0.0 (2026-02-10)  基础架构，RealSense 集成，Web 界面
```

---

## 废弃功能

| 版本 | 功能 | 替代方案 | 移除日期 |
|------|------|----------|----------|
| 1.2.0 | `dump_point` 配置 | `dump_point_a` + `dump_point_b` | 待定 |

---

*文档版本: v2.1*
*最后更新: 2026-07-31*

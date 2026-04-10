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

- **地图栅格旋转功能**
  - 新增全局 theta 旋转参数，支持手动输入角度值（默认 -178.0472°，顺时针旋转）
  - 后端 `map_converter` 服务新增 `coordinate_rotate` 函数，对所有栅格点应用旋转
  - 后端 `/api/map/{name}.png` 支持 `theta` 参数（度），自动转换弧度后生成旋转PNG
  - 后端 `/api/map/{name}/info` 支持 `theta` 参数，返回旋转后边界 `rotated_bounds` 和图片像素尺寸 `img_width_px/img_height_px`
  - 缓存机制支持不同 theta 值的独立缓存（文件名带 `_theta{x.xxxx}` 后缀）
  - 前端 MapPanel 右上角添加 theta 输入框，可手动修改角度

- **地图车辆位置指示器**
  - 在地图上叠加显示带箭头的银色方块，表示车辆实时位置和朝向
  - 方块尺寸与真实车体比例一致：宽 800mm，长 1400mm
  - 根据地图分辨率自动计算像素尺寸，跟随地图缩放
  - XYZA 数值表示车辆正中心，角度 A 为车头与X轴夹角
  - 通过 `imageLoaded` 状态 + `onLoad` 回调确保图片加载后才计算位置

- **机器人控制按钮新协议**
  - `POST /api/robot/scoop` → 发送铲取任务（Type=pick）
  - `POST /api/robot/dump` → 发送倾倒任务（Type=drop）
  - `POST /api/robot/dock` → 发送回桩任务（Type=charge）
  - `POST /api/robot/stop` → 发送取消任务命令（cancelTask）
  - `POST /api/robot/pause` → 发送暂停任务命令（pauseTask）
  - `POST /api/robot/resume` → 发送取消暂停命令（pauseCancel）
  - `POST /api/robot/nav-pick` → 导航到取货点（Type=allPick）
  - `POST /api/robot/nav-drop` → 导航到卸货点（Type=allDrop）
  - 新协议格式: `{MessageType=task\nTaskId=xxx\nType=xxxx\n}`
  - TaskId 生成器（线程安全）: `YYYYMMDDHHMMSS` + 3位序号

- **按钮互锁与任务状态跟踪**
  - 铲取/倾倒/回桩按钮互锁：按下任意一个时，另外两个锁定
  - 收到下位机 `{MessageType=taskFinish TaskId=xxx}` 后解除锁定
  - 后端通过 Socket.IO 广播 `robot_task_finish` 事件
  - 暂停/停止按钮不受互锁影响，可用于取消任务

- **任务系统导航按钮**
  - BehaviorTreeViz 中「导航到取糖点」胶囊按钮可点击，发送 allPick
  - BehaviorTreeViz 中「导航到卸载点」胶囊按钮可点击，发送 allDrop
  - 任务运行时两个导航按钮自动禁用

### [Changed]

- **机器人控制面板底部按钮**
  - 五个按钮：铲取、倾倒、暂停、停止、回桩
  - 铲取/倾倒/回桩在任务执行中互锁禁用
  - 暂停按钮调用 pauseTask 协议，仅在任务运行时可点击
  - 举升进度条范围更新为 60°~124°

- **TCP 通讯回复规则**
  - 只有收到 taskFinish 时回复下位机，其他报文不再回复

- **机器人状态卡片速度显示**
  - "设定速度"改为"车辆速度"
  - 使用 status.speed 实时更新数值，替代原来的固定值 0.5 m/s

- **地图坐标显示**
  - X/Y 显示原始机器人数据（mm），不应用 theta 旋转转换
  - X/Y 单位为整数（mm），A 单位为度（1位小数）

### [Fixed]

- **修复 matplotlib 导入失败导致地图PNG转换失败**
  - 根因：venv 中 matplotlib 3.5.1（系统包）与 NumPy 2.2.6 版本不兼容
  - 解决：在 venv 中安装新版 matplotlib 3.10.8

- **修复前端 auto_cycle 轮询 404 错误**
  - 前端每 2 秒轮询 `/api/robot/auto_cycle/status`，后端无此端点
  - 移除该轮询，消除大量 404 控制台错误

- **修复前端 MapInfo 数据结构与后端 API 不匹配**
  - 后端 `/api/map/` 返回 `filename, size_bytes, modified_time` 等字段
  - 前端 `MapInfo` 接口期望 `resolution, origin_x, grid_width` 等字段
  - 统一接口定义，新增 `MapDetailInfo` 接口包含旋转后边界信息

- **修复车辆指示器不显示问题**
  - 根因：`useMemo` 依赖 `imageRef.current`，但 ref 在渲染时为 null 且 ref 变化不触发重渲染
  - 解决：添加 `imageLoaded` 状态 + `onLoad` 回调，确保图片加载完成后才计算位置

### [Refactor]

- **地图 API 后端重构**
  - `map_converter.py` 新增 `coordinate_rotate` 旋转函数
  - `to_png` 方法支持 theta 参数，对所有栅格点应用旋转后生成 PNG
  - `convert_map` 便捷函数透传 theta 参数
  - `map.py` API 缓存 key 包含 theta 值，支持不同角度独立缓存

- **前端 MapPanel 重构**
  - 分离 `robotStatus`（原始坐标显示）和 `vehicleData`（地图指示器定位）
  - `useMapImage` hook 支持 thetaDeg 参数传递给后端
  - `useMapInfo` hook 支持 thetaDeg 参数获取旋转后边界
  - 移除未使用的 `coordinateRotate` 前端函数（后端统一处理）

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

*文档版本: v1.2*
*最后更新: 2026-03-03*

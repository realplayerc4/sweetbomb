# 变更日志 (Changelog)

> 本文档记录项目的所有重要变更。
> 遵循 [Keep a Changelog](https://keepachangelog.com/) 格式。

---

## [1.2.0] - 2026-02-27

### Added

- **仪表盘 3x2 完美矩阵**: 重构 Grid 布局为 3列 x 2行，确保所有卡片宽度完全对齐且垂直居中。
- **空间配置与地图解耦**: 将 `SpatialConfigPanel` 和 `MapPanel` 提取为独立组件，提升代码可维护性。
- **全局状态脚部 (Footer)**: 引入底部状态栏，整合电池、CPU、温度及信号强度展示，并支持等距排布。

### Changed

- **视觉体验黄金优化**:
  - **偏移量对齐**: 所有视图状态胶囊统一采用 `top-[10px]` 偏移量，改善边缘压迫感。
  - **文字与图标**: 胶囊内文字全部加粗，图标统一为石墨橙品牌色 (`#FD802E`)。
  - **背景色一致性**: 点云图及切面图背景统一更新为石墨深色 (`#1c1c1e`)。
- **点云性能调优**: 粒子大小从 1.5 回调至 **4.5**，增强视觉饱满度。
- **测距范围拓展**: 后端 `rs_manager.py` 探测范围从 3m 扩展至 **6m**。
- **布局间距**: 统一卡片间隙为 **30px**。

### Fixed

- **点云显示稳定性**: 重新启用 `frustumCulled: false` 机制，彻底解决视角拉远时点云消失问题。
- **空间配置胶囊重叠**: 重构容器 DOM 结构，通过 `pt-16` 解决胶囊遮挡首个滑块的问题。

---

## [Unreleased]

### Added

- **行为树铲糖动作模块重构**:
  - 新增 `bt_calculate_distance_node.py` - 独立计算前进距离节点
  - 新增 `bt_scoop_action.py` - 连贯铲糖动作模块（前进→翻转→倒退）
  - 新增 `bt_dump_action.py` - 连贯卸载动作模块（A点→举升→B点→确认→倾倒→倒退→归零）
  - 路径 B 从"切换推垛模式"改为"回桩"（返回充电桩）

- **卸载动作按钮确认机制**:
  - 翻转倾倒电机前需人工按钮确认，防止误触导致物料倾洒
  - 新增 `_wait_for_button_confirmation()` 辅助函数

- **前端配置参数扩展**:
  - 新增 `dump_point_a` 和 `dump_point_b` 配置（卸载A点和B点）
  - 新增 `lift_height` 配置（举升高度，默认90°）
  - 保留 `dump_point` 作为向后兼容（标记为废弃）

### Changed

- **行为树结构优化**:
  - 铲糖流程从分散节点改为 `ScoopAndReturn` 整合节点
  - 卸载流程从分散节点改为 `DumpAndReturn` 整合节点
  - 倒退动作改为直接控制机器人倒退（非导航模式）
  - 距离计算与动作执行分离，便于日志和调试

- **卸载流程重构**:
  - 原流程：导航到卸载点 → 翻斗卸载 → 复位
  - 新流程：导航A点 → 举升 → 导航B点 → 【按钮确认】→ 倾倒 → 倒退回A点 → 归零

### Fixed

- **点云显示优化**:
  - **颜色调整**: 点云粒子颜色从紫色改为石墨橙品牌色 (`#FD802E`)。
  - **视角修正**: 相机初始位置改为俯视角度 (0, 5, 0)，XY 平面在下，Z 轴朝上。
  - **坐标系对齐**: 后端点云数据增加 RealSense → Robot (Z-up) 坐标变换。
- **紧急恢复与 UI 重塑 (Golden Restore)**:
  - 修复了由于 `git checkout` 导致的 UI 代码回退与编译冲突问题。
  - **点云稳定性修复**: 重新启用 `frustumCulled: false` 机制，彻底解决视角拉远时点云消失的问题。
  - **坐标系对齐**: 重新注入 RealSense -> Robot (Z-up) 坐标变换逻辑。
  - **CSS 修复**: 修正 `index.css` 指令排序，解决 Vite 构建报错。
  - **风格同步**: 将 UI 完全恢复至最新的 Formant 工业风（极简圆角、大间距、Full-bleed 视窗）。

### Added

### Changed

- 视觉渲染重构:
  - 缩减并硬编码 BEVSlice 图仅截取前向 `1.0m~3.0m` 区域且置中摄像机，丢弃（`discard`）容差区间外噪点。
  - 修正 PointCloudView 因为使用 `THREE.AdditiveBlending` 导致粒子密集呈现高光过曝发白的问题，回退至 `NormalBlending`。
  - 变更默认 3D 高亮散点颜色为紫色球体 (`#A855F7`)，且将外围控制组件标头全部重写为字重 `bold` 的石墨橙品牌色 (`#FD802E`)。
- 清理无用代码:
  - 删除未完成的 `security.py`
  - 移除未使用的 OAuth2 依赖
  - 清理未使用的 Pydantic 模型 (Device, Sensor, Option 等)
  - 移除未使用的 `hole_filling_filter`
  - 删除注释掉的安全相关代码
- 修复测试: 移除 `mock_dependencies.py` 中的 oauth2_scheme 引用

### Removed

- `app/core/security.py` - 未完成的 TODO 代码
- `config.py` 中的安全配置 (SECRET_KEY, ALGORITHM, ACCESS_TOKEN_EXPIRE_MINUTES)
- `dependencies.py` 中的 `oauth2_scheme`
- 多个未使用的模型基类

---

## [Unreleased]

### Added

- 新增 `device_discovery.py` 子模块，负责设备发现与枚举。
- 新增 `sensor_control.py` 子模块，负责传感器选项与信息查询。
- 新增 `stream_controller.py` 子模块，负责视频流启停与帧处理。
- 新增 `point_cloud_processor.py` 子模块，负责点云状态管理。

### Changed

- 重构 `rs_manager.py`，从巨石类（954行）转变为基于 Facade 门面模式的委派调用层，核心逻辑拆分至四个独立子模块，对外 API 保持 100% 兼容。

### Removed

- 删除了被标记弃用的前端文件 `ui/frontend/src/app/components/RobotDisplay.tsx`。

## [1.1.0] - 2025-02-23

### Added

- 任务管理系统
  - `BaseTask` 抽象基类
  - `TaskManager` 单例管理器
  - 任务注册表 (`registry.py`)
  - Socket.IO 任务事件广播
- 内置任务类型
  - `object_detection`: YOLOv8 目标检测
  - `point_cloud_analysis`: 点云分析
  - `data_collection`: 数据采集
- 前端任务面板 (`TaskPanel.tsx`)
- 任务管理 Hook (`useTaskManager.ts`)
- 任务 API 客户端 (`taskApi.ts`)

### Changed

- 重构状态面板，优化状态信息展示
- 移除机器人显示组件

### Fixed

- WebRTC 流的清理与停止逻辑优化

---

## [1.0.0] - 2025-01-XX

### Added

- 核心功能
  - RealSense 设备发现与管理
  - RGB/Depth 双流视频传输
  - WebRTC 低延迟视频流
  - Socket.IO 实时元数据推送
- 后端架构
  - FastAPI RESTful API
  - RealSense 管理器 (`rs_manager.py`)
  - WebRTC 管理器
  - 深度滤波器链
  - 点云生成
- 前端架构
  - React + TypeScript
  - TailwindCSS 样式系统
  - Three.js 3D 点云视图
  - 控制面板 (参数配置)
  - 状态面板 (系统状态)
- 文档
  - 中英双语 README
  - API 文档 (FastAPI 自动生成)

---

## 版本说明

### 版本号规则

- **MAJOR**: 不兼容的 API 变更
- **MINOR**: 向后兼容的功能新增
- **PATCH**: 向后兼容的问题修复

### 变更类型

- `Added`: 新增功能
- `Changed`: 功能变更
- `Deprecated`: 即将废弃
- `Removed`: 已移除
- `Fixed`: 问题修复
- `Security`: 安全相关

---

*当前版本: 1.1.0*
*最后更新: 2025-02-24*

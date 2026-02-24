# 变更日志 (Changelog)

> 本文档记录项目的所有重要变更。
> 遵循 [Keep a Changelog](https://keepachangelog.com/) 格式。

---

## [Unreleased]

### Added

- 四文件架构: `spec.md`, `knowledge.md`, `changelog.md`
- 多 Agent 角色配置: `.agent/` 目录

### Changed

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

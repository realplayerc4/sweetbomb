# Architecture

> 系统架构与模块职责定义

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      Frontend Layer                          │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │
│  │   React     │  │  Three.js   │  │     Socket.IO       │ │
│  │   App       │  │  PointCloud │  │     Client          │ │
│  └──────┬──────┘  └──────┬──────┘  └──────────┬──────────┘ │
└─────────┼────────────────┼───────────────────┼─────────────┘
          │                │                   │
          ▼                ▼                   ▼
┌─────────────────────────────────────────────────────────────┐
│                      API Gateway                             │
│                    FastAPI + Uvicorn                         │
│                     (Port 8000)                              │
└───────────────────────────┬─────────────────────────────────┘
                            │
          ┌─────────────────┼─────────────────┐
          ▼                 ▼                 ▼
┌──────────────┐  ┌──────────────┐  ┌──────────────┐
│ rs_manager   │  │task_manager  │  │webrtc_manager│
│ (设备管理)    │  │(任务管理)     │  │(视频流管理)  │
└──────┬───────┘  └──────────────┘  └──────────────┘
       │
       ▼
┌─────────────────────────────────────────────────────────────┐
│                    Hardware Layer                            │
│              pyrealsense2 + Intel RealSense D400             │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│                 Robot TCP Server (Port 9090)                 │
│                                                              │
│  ┌─────────────┐    TCP Protocol    ┌──────────────────┐   │
│  │   Backend   │ <────────────────> │  Robot Controller │   │
│  │  (FastAPI)  │   Port 9090        │    (嵌入式)        │   │
│  └─────────────┘                    └──────────────────┘   │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

---

## Module Responsibilities

### 1. rs_manager (设备管理器)

| 职责 | 描述 |
|------|------|
| 设备发现 | 监控设备连接/断开事件 |
| 状态管理 | 维护设备运行状态 |
| 参数控制 | 读写传感器选项 |
| 流控制 | 启动/停止数据流 |
| 点云分析 | 计算体积、距离、高度 |

**依赖**: pyrealsense2
**线程安全**: 是 (使用锁保护共享状态)

---

### 2. task_manager (任务管理器)

| 职责 | 描述 |
|------|------|
| 任务调度 | 管理任务生命周期 |
| 并发控制 | 限制最大并发数 |
| 进度追踪 | 实时更新任务进度 |
| 事件发布 | 通过 Socket.IO 广播任务事件 |

**依赖**: asyncio, Socket.IO
**限制**: 最大 4 个并发任务

---

### 3. webrtc_manager (视频流管理器)

| 职责 | 描述 |
|------|------|
| 连接管理 | WebRTC PeerConnection 生命周期 |
| 视频编码 | H.264/VP8 编码 |
| 帧处理 | RGB/Depth 帧格式转换 |
| 深度伪彩 | Jet colormap 映射 |

**依赖**: aiortc
**延迟要求**: < 100ms

---

### 4. robot_tcp_server (机器人TCP服务器) ⚠️ 重要

| 职责 | 描述 |
|------|------|
| 连接管理 | 管理机器人TCP连接（端口9090） |
| 协议解析 | 解析UTF-8文本协议报文 |
| 状态同步 | 实时同步机器人状态数据 |
| 任务下发 | 发送任务、命令到机器人 |
| 相机联动 | 发送相机检测距离到机器人 |

**协议格式**:
```
{
MessageType=xxx
Key1=Value1
Key2=Value2
}
```

**支持的消息类型**:
- `status` - 状态查询/上报
- `task` - 任务下发
- `taskFinish` - 任务完成
- `cancelTask` - 取消任务
- `command` - 控制命令
- `remoteControl` - 远程控制
- `cameraCheckDistance` - 相机检测距离

**依赖**: asyncio
**端口**: 9090

---

### 5. behavior_tree_engine (行为树引擎)

| 职责 | 描述 |
|------|------|
| 节点执行 | 顺序、选择、动作、条件节点执行 |
| 状态管理 | 节点状态转换 (IDLE→RUNNING→SUCCESS/FAILURE) |
| 黑板共享 | 节点间数据共享机制 |
| 事件广播 | 节点状态变化实时推送前端 |

**核心节点类型**: Sequence, Selector, Action, Condition, Repeat, Inverter

---

### 6. distance_analyzer (距离分析器)

| 职责 | 描述 |
|------|------|
| 点云处理 | 深度图转点云数据 |
| 距离计算 | 糖堆距离智能测算 |
| 高度检测 | 糖堆高度识别 |
| 策略决策 | 铲糖策略判断（高/矮糖堆） |

**依赖**: NumPy, Open3D (可选)

---

## Communication Protocols

| 通道 | 协议 | 用途 | 延迟 |
|------|------|------|------|
| 视频流 | WebRTC | RGB/Depth 实时传输 | < 100ms |
| 元数据 | Socket.IO | 设备状态、任务事件 | < 500ms |
| 控制 | REST API | 参数配置、任务管理 | < 1s |
| 机器人 | **TCP/IP (端口 9090)** | 机器人控制、状态同步 | < 50ms |

### 机器人TCP协议详解

**服务器配置**:
- 监听地址: `0.0.0.0:9090`
- 编码: UTF-8
- 报文格式: 文本协议，以 `{` 开头 `}` 结尾

**报文结构**:
```
{
MessageType=<类型>
Key1=Value1
Key2=Value2
...
}
```

**状态数据字段**:
| 字段 | 类型 | 说明 |
|------|------|------|
| Mode | string | 模式: auto/handle/remote |
| Status | string | 状态: idle/running |
| Charge | float | 电量 0-100 |
| Speed | float | 速度 m/s |
| X/Y/Z | int | 坐标 mm |
| A | float | 车体角度 ° |
| Boom | float | 臂位置 mm |
| Bucket | float | 铲斗角度 ° |

**心跳机制**:
- 机器人定时发送状态查询报文
- 服务端回复完整状态数据
- 服务端10秒无报文判定为超时断开

---

## Directory Structure

```
ros2realsense/
├── memory-bank/           # AI 上下文核心
│   ├── project-overview.md
│   ├── architecture.md      # ← 本文档
│   ├── tech-stack.md
│   └── progress.md
├── docs/
│   ├── agent/             # Agent 角色配置
│   ├── guides/            # 使用指南
│   ├── api/               # API 规范与文档
│   └── changelog.md       # 变更日志
├── app/                   # 后端核心业务服务
│   ├── api/
│   │   ├── router.py      # API 路由聚合
│   │   ├── dependencies.py # 依赖注入
│   │   └── endpoints/     # API 端点实现
│   │       ├── devices.py
│   │       ├── sensors.py
│   │       ├── options.py
│   │       ├── streams.py
│   │       ├── point_cloud.py
│   │       ├── webrtc.py
│   │       ├── tasks.py
│   │       ├── waypoints.py
│   │       └── robot.py     # ← 机器人控制 (HTTP + TCP)
│   ├── core/              # 核心配置
│   ├── models/            # 数据模型
│   └── services/          # 业务逻辑
│       ├── rs_manager.py
│       ├── webrtc_manager.py
│       ├── robot_tcp_server.py  # ← TCP 服务器 (端口 9090)
│       ├── behavior_tree_engine.py
│       └── ...
├── ui/                    # 前端 React 源码
│   └── frontend/
│       └── src/app/
│           ├── components/       # React 组件
│           │   ├── Dashboard.tsx
│           │   ├── RobotControlPanel.tsx
│           │   ├── VideoView.tsx
│           │   └── ...
│           ├── services/         # API 客户端
│           │   ├── api.ts          # 设备/WebRTC API
│           │   ├── robotApi.ts     # 机器人 API
│           │   └── taskApi.ts      # 任务 API
│           ├── hooks/            # 自定义 Hooks
│           └── config.ts         # API 配置
├── tests/                 # 单元与集成测试
├── deploy/                # 运维与部署脚本
├── rest-api/              # API 文档
│   ├── spec.md            # ← API 规范
│   └── knowledge.md       # ← 本文档
├── memory-bank/           # AI 上下文核心
├── docs/                  # 项目文档
├── ecosystem.config.cjs   # PM2 配置文件
└── main.py                # 应用入口
```

---

## Data Flow

```
用户操作 → React UI → REST API → Service Layer → Hardware
                              ↓
                         Socket.IO → 实时更新 UI

机器人 ←→ TCP Server (Port 9090) ←→ FastAPI Backend
```

---

## 重要变更历史

| 日期 | 变更内容 | 影响 |
|-----|---------|------|
| 2024-03-17 | 机器人通信从 HTTP REST 改为 TCP 协议 (端口 9090) | 重大架构变更 |
| 2024-03-15 | 新增行为树引擎支持铲糖自主任务 | 新功能 |
| 2024-03-10 | 新增 WebRTC 视频流传输 | 新功能 |

---

*Version: 2.0.0*
*Last Updated: 2025-03-18*

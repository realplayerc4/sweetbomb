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

**依赖**: pyrealsense2
**线程安全**: 是 (使用锁保护共享状态)

### 2. task_manager (任务管理器)

| 职责 | 描述 |
|------|------|
| 任务调度 | 管理任务生命周期 |
| 并发控制 | 限制最大并发数 |
| 进度追踪 | 实时更新任务进度 |
| 事件发布 | 通过 Socket.IO 广播任务事件 |

**依赖**: asyncio, Socket.IO
**限制**: 最大 4 个并发任务

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

## Communication Protocols

| 通道 | 协议 | 用途 | 延迟 |
|------|------|------|------|
| 视频流 | WebRTC | RGB/Depth 实时传输 | < 100ms |
| 元数据 | Socket.IO | 设备状态、任务事件 | < 500ms |
| 控制 | REST API | 参数配置、任务管理 | < 1s |

---

## Directory Structure

```
ros2realsense/
├── memory-bank/           # AI 上下文核心
│   ├── project-overview.md
│   ├── architecture.md
│   ├── tech-stack.md
│   └── progress.md
├── docs/
│   ├── agent/             # Agent 角色配置
│   ├── guides/            # 使用指南
│   ├── api/               # API 规范与文档 (spec.md, knowledge.md)
│   └── changelog.md       # 变更日志
├── app/                   # 后端核心业务服务 (原 rest-api/app)
│   ├── routers/           # API 路由
│   ├── services/          # 业务逻辑
│   └── models/            # 数据模型
├── tests/                 # 单元与全链路集成测试
├── deploy/                # 运维与远程节点部署脚本集合
├── ui/                    # 前端 React 源码与编译产物
├── tools/                 # 脱机独立静态 HTML 调试器
└── logs/                  # 系统日志归档仓
```

---

## Data Flow

```
用户操作 → React UI → REST API → Service Layer → Hardware
                              ↓
                         Socket.IO → 实时更新 UI
```

---

*Version: v1.0*
*Last Updated: 2025-02-24*

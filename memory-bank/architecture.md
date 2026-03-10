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

### 4. robot_controller (机器人控制器)

| 职责 | 描述 |
|------|------|
| 运动控制 | 履带式移动控制（八方向） |
| 伺服控制 | 铲斗举升/翻转控制 |
| 状态获取 | 电池、位置、姿态实时反馈 |
| 安全保护 | 紧急停止机制 |

**通信方式**: WebSocket + 串口代理

### 5. behavior_tree_engine (行为树引擎)

| 职责 | 描述 |
|------|------|
| 节点执行 | 顺序、选择、动作、条件节点执行 |
| 状态管理 | 节点状态转换 (IDLE→RUNNING→SUCCESS/FAILURE) |
| 黑板共享 | 节点间数据共享机制 |
| 事件广播 | 节点状态变化实时推送前端 |

**核心节点类型**: Sequence, Selector, Action, Condition, Repeat, Inverter

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
| 机器人 | WebSocket | 运动控制、状态反馈 | < 100ms |

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
│   └── frontend/
│       └── src/app/
│           ├── components/       # React 组件
│           │   ├── Dashboard.tsx        # 主仪表盘 (3x2 网格布局)
│           │   ├── RobotControlPanel.tsx # 机器人控制面板
│           │   ├── TaskPanel.tsx         # 任务管理面板
│           │   ├── PointCloud.tsx        # 3D 点云视图
│           │   ├── VideoView.tsx         # RGB/Depth 视频视图
│           │   ├── SliceView.tsx         # BEV 俯视切片
│           │   ├── MapPanel.tsx          # 航点地图面板
│           │   └── SugarHarvestPanel.tsx # 铲糖任务面板
│           ├── services/         # API 客户端
│           ├── hooks/            # 自定义 Hooks
│           └── App.tsx              # 应用入口
├── tests/                        # 单元与集成测试
├── deploy/                       # 运维与部署脚本
├── memory-bank/                  # AI 上下文核心
├── docs/                         # 项目文档
├── ecosystem.config.cjs          # PM2 配置文件
└── README.md                     # 项目说明
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

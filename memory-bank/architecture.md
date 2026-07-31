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
└──────┬───────┘  └──────┬───────┘  └──────┬───────┘
       │                 │                  │
       ▼                 ▼                  ▼
┌──────────────┐  ┌──────────────┐  ┌──────────────┐
│robot_tcp_    │  │yolo_detector │  │map_converter │
│server        │  │(YOLOv8检测)  │  │(地图转换)    │
│(TCP通讯)     │  │              │  │              │
└──────────────┘  └──────────────┘  └──────────────┘
       │                 │                  │
       └─────────────────┼─────────────────┘
                         │
                         ▼
              ┌──────────────────────┐
              │  Hardware Layer      │
              │ pyrealsense2 +       │
              │ Robot TCP (9090)     │
              └──────────────────────┘
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

### 7. map_converter (地图转换器)

| 职责 | 描述 |
|------|------|
| 地图读取 | 读取 txt 栅格地图文件 |
| 坐标转换 | 网格坐标 → 世界坐标 (mm) |
| 旋转支持 | theta 角度旋转（支持任意角度） |
| 图片生成 | PNG/SVG 格式输出 |
| 缓存管理 | 不同 theta 值独立缓存 |

**依赖**: matplotlib, numpy
**输出**: PNG/SVG 地图图片 + 边界信息

### 8. path_map_manager (路径图管理器)

| 职责 | 描述 |
|------|------|
| 站点解析 | 解析 path_map.json 站点配置 |
| 坐标计算 | 取货站圆形分布算法 |
| 坐标转换 | 站点坐标 → 地图像素坐标 |
| 缓存管理 | 站点数据缓存 |

**依赖**: numpy
**核心算法**: 逆时针圆形分布站位生成

### 9. robot_tcp_server (机器人 TCP 服务)

| 职责 | 描述 |
|------|------|
| TCP 监听 | 端口 9090 监听机器人连接 |
| 报文解析 | `{MessageType=xxx\nField=value\n}` 格式 |
| 状态查询 | 定时发送 `status` 查询 |
| 任务下发 | pick/drop/charge/allPick/allDrop/cancelTask/pauseTask |
| 相机距离 | 每 250ms 发送 `cameraCheck` 距离 |
| 回复规则 | 仅 taskFinish 回复，其他不回复 |

**通信方式**: TCP/IP，上位机作为服务端

### 10. yolo_detector (YOLOv8 检测服务)

| 职责 | 描述 |
|------|------|
| 模型推理 | ONNX YOLOv8n 模型，OpenCV DNN 后端 |
| 双流检测 | RGB 和深度流分别检测 |
| 标签过滤 | RGB 流检测 `RGB_SweetBomb`，深度流检测 `Depth_SweetBomb` |
| 帧标注 | 在 RGB 帧上绘制检测框和标签 |
| 结果缓存 | 按 device_id + stream_type 缓存最新结果 |

**依赖**: OpenCV (cv2.dnn), numpy
**模型路径**: `/home/jetson/sweetbomb/models/yolov8n/best.onnx`
**环境变量**: `YOLO_ENABLED`, `YOLO_CONFIDENCE`, `YOLO_MAX_FPS`

### 11. simple_task_dispatcher (简化任务分发器)

| 职责 | 描述 |
|------|------|
| 任务触发 | 接收任务请求，发送给下位机 |
| 状态监控 | 监听下位机状态更新 |
| 生命周期 | 管理任务运行/停止/错误状态 |

**设计**: 替代复杂行为树，上位机只负责任务触发和状态监控，下位机自主执行具体动作

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
sweetbomb/
├── app/                   # 后端核心业务服务
│   ├── api/
│   │   ├── dependencies.py
│   │   ├── endpoints/     # API 路由层
│   │   │   ├── devices.py      # 设备管理 API
│   │   │   ├── streams.py      # WebRTC 信令 API
│   │   │   ├── robot.py        # 机器人控制 API
│   │   │   ├── tasks.py        # 任务管理 API
│   │   │   ├── point_cloud.py  # 点云数据 API
│   │   │   ├── path_map.py     # 路径图 API
│   │   │   ├── map.py          # 地图 API
│   │   │   └── webrtc.py       # WebRTC API
│   │   └── router.py           # 主路由聚合
│   ├── core/           # 核心配置
│   │   ├── config.py
│   │   ├── errors.py
│   │   └── logging_config.py
│   ├── models/         # 数据模型
│   │   ├── device.py
│   │   ├── option.py
│   │   ├── sensor.py
│   │   ├── stream.py
│   │   ├── task.py
│   │   ├── waypoint.py
│   │   ├── path_map.py
│   │   └── webrtc.py
│   └── services/       # 业务逻辑
│       ├── rs_manager.py           # RealSense 设备管理
│       ├── stream_controller.py    # 视频流控制
│       ├── webrtc_manager.py       # WebRTC 管理
│       ├── robot_controller.py     # 机器人运动控制
│       ├── robot_tcp_server.py     # TCP 通讯服务
│       ├── task_manager.py         # 任务生命周期管理
│       ├── simple_task_dispatcher.py  # 简化任务分发
│       ├── behavior_tree_engine.py # 行为树引擎
│       ├── bt_nodes.py             # 行为树基础节点
│       ├── bt_action_nodes.py      # 行为树动作节点
│       ├── bt_dump_action.py       # 倾倒动作
│       ├── bt_scoop_action.py      # 铲糖动作
│       ├── bt_calculate_distance_node.py  # 距离计算节点
│       ├── distance_analyzer.py    # 距离分析
│       ├── point_cloud_analyzer.py # 点云分析
│       ├── point_cloud_processor.py
│       ├── map_converter.py        # 地图转换 (txt→PNG/SVG)
│       ├── path_map_manager.py     # 路径图/站点管理
│       ├── coordinate_transform.py # 坐标转换
│       ├── navigation_interface.py # 导航接口
│       ├── mock_navigation.py      # Mock 导航
│       ├── metadata_socket_server.py  # Socket.IO 服务
│       ├── socketio.py
│       ├── yolo_detector.py        # YOLOv8 检测服务
│       ├── device_discovery.py
│       ├── sensor_control.py
│       ├── waypoint_manager.py
│       └── tasks/                  # 任务实现包
│           ├── base_task.py
│           ├── registry.py
│           └── implementations/
│               ├── sugar_harvest_task.py
│               ├── point_cloud_analysis_task.py
│               ├── object_detection_task.py
│               ├── navigation_task.py
│               ├── data_collection_task.py
│               └── sequential_task_queue.py
├── ui/                        # 前端源码
│   ├── frontend/              # SW 前端 (React 18 + TailwindCSS + Three.js)
│   │   └── src/app/
│   │       ├── components/    # React 组件
│   │       ├── services/      # API 客户端
│   │       └── hooks/         # 自定义 Hooks
│   └── admin-frontend/        # SWNFP 前端 (React 19 + Ant Design 6 + Zustand)
│       └── src/
│           ├── components/    # 管理后台组件
│           ├── services/      # API 服务层
│           ├── stores/        # Zustand 状态管理
│           └── hooks/         # 自定义 Hooks
├── tests/                 # 单元与集成测试
├── deploy/                # 运维与部署脚本
├── memory-bank/           # AI 上下文核心
├── docs/                  # 项目文档
├── models/yolov8n/        # YOLOv8 ONNX 模型目录
├── main.py                # 后端入口
├── config.py              # 全局配置
├── ecosystem.config.cjs   # PM2 配置 (遗留)
├── start-simple.sh        # 开发启动脚本
└── README.md              # 项目说明
```

---

## Data Flow

```
用户操作 → React UI → REST API → Service Layer → Hardware
                              ↓
                         Socket.IO → 实时更新 UI
```

---

*Version: v2.1.0*
*Last Updated: 2026-07-31*

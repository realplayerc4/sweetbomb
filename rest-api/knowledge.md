# Technical Knowledge

> RealSense Web Monitor - 技术知识库

---

## 通信协议详解

### 1. TCP 机器人通信协议 (端口 9090)

**连接配置**:
```
地址: 0.0.0.0:9090
编码: UTF-8
心跳: 10秒超时检测
```

**报文格式**:
```
{
MessageType=<消息类型>
Key1=Value1
Key2=Value2
...
}
```

**消息类型定义**:

| 类型 | 方向 | 说明 |
|-----|------|------|
| `status` | 双向 | 状态查询/上报 |
| `task` | 下行 | 任务下发 |
| `taskFinish` | 上行 | 任务完成通知 |
| `cancelTask` | 下行 | 取消任务 |
| `command` | 下行 | 控制命令 |
| `remoteControl` | 下行 | 远程控制 |
| `cameraCheckDistance` | 下行 | 相机检测距离 |

**机器人状态字段**:

| 字段 | 类型 | 说明 |
|-----|------|------|
| Mode | string | auto/handle/remote |
| Status | string | idle/running |
| Charge | float | 电量 0-100 |
| Speed | float | 速度 m/s |
| X/Y/Z | int | 坐标 mm |
| A | float | 车体角度 ° |
| Boom | float | 臂位置 mm |
| Bucket | float | 铲斗角度 ° |
| Fault | int | 故障码 |
| FaultLevel | int | 故障等级 |

---

### 2. WebRTC 信令协议

**信令流程**:
```
Client                        Server
  |                              |
  |--- POST /webrtc/offer ------>|
  |<-- {session_id, sdp, type} --|
  |                              |
  |--- POST /webrtc/answer ----->|
  |                              |
  |--- POST /webrtc/ice-candidates ->|
  |<-- {candidates} -------------|
  |                              |
```

**数据通道**:
- Stream Types: `color`, `depth`
- Codecs: H.264, VP8
- 延迟要求: < 100ms

---

### 3. Socket.IO 元数据通道

**命名空间**: `/` (默认)

**事件类型**:

| 事件 | 方向 | 数据 | 说明 |
|-----|------|------|------|
| `task_event` | 服务端→客户端 | TaskEvent | 任务状态变更 |
| `robot_status_update` | 服务端→客户端 | RobotStatus | 机器人状态更新 |
| `connect` | 双向 | - | 连接建立 |
| `disconnect` | 双向 | - | 连接断开 |

---

## 技术实现细节

### 1. RealSense 设备管理

**关键类**: `RealSenseManager` (`app/services/rs_manager.py`)

**功能**:
- 设备发现和枚举
- 传感器配置 (曝光、增益、激光功率)
- 流控制 (RGB/Depth/IR)
- 点云生成和分析

**线程安全**: 使用 `threading.Lock` 保护共享状态

### 2. WebRTC 管理器

**关键类**: `WebRTCManager` (`app/services/webrtc_manager.py`)

**功能**:
- PeerConnection 生命周期管理
- H.264/VP8 视频编码
- RGB/Depth 帧格式转换
- Jet 伪彩色映射 (深度可视化)

**依赖**: `aiortc`

### 3. 机器人 TCP 服务器

**关键类**: `RobotTCPServer` (`app/services/robot_tcp_server.py`)

**配置**:
```python
host = "0.0.0.0"
port = 9090
encoding = "utf-8"
heartbeat_timeout = 10  # 秒
```

**功能**:
- 多机器人连接管理
- 协议解析 (文本协议)
- 状态同步
- 任务下发

### 4. 行为树引擎

**关键类**: `BehaviorTreeEngine` (`app/services/behavior_tree_engine.py`)

**节点类型**:
- `SequenceNode` - 顺序执行
- `SelectorNode` - 选择执行
- `ActionNode` - 动作节点
- `ConditionNode` - 条件节点
- `RepeatNode` - 重复节点
- `InverterNode` - 取反节点

**铲糖任务行为树**:
```
RepeatNode (max_count=cycles)
└── SequenceNode
    ├── NavigateToSugarPoint
    ├── CheckShovelFlat
    ├── AnalyzeSugarDistance
    └── SelectorNode (HeightCheck)
        ├── SequenceNode (高度足够)
        │   ├── CalculateApproachDistance
        │   ├── ScoopAndReturn
        │   └── DumpAndReturn
        └── ReturnToHome (高度不足)
```

---

## 配置文件

### 后端配置 (`config.py`)

```python
class Settings:
    PROJECT_NAME: str = "RealSense Web Monitor"
    API_V1_STR: str = "/api/v1"
    CORS_ORIGINS: List[str] = ["*"]
```

### 前端配置 (`ui/frontend/src/app/config.ts`)

```typescript
const API_PORT = '8000';
const HOST = isDev
    ? `${window.location.protocol}//${window.location.hostname}:${API_PORT}`
    : window.location.origin;

export const API_BASE = `${HOST}/api`;  // 开发环境: http://localhost:8000/api
export const SOCKET_URL = HOST;
```

---

## 调试与开发

### 启动服务

```bash
# 使用 PM2 启动所有服务
pm2 start ecosystem.config.cjs

# 或单独启动
pm2 start sweetbomb-8000  # 后端
pm2 start sweetbomb-5173  # 前端
```

### 查看日志

```bash
pm2 logs
pm2 logs sweetbomb-8000
pm2 logs sweetbomb-5173
```

### 测试 TCP 连接

```bash
# 使用 telnet 测试机器人 TCP 服务器
telnet localhost 9090

# 发送测试报文
{
MessageType=status
}
```

---

## 相关文件

| 文件 | 说明 |
|------|------|
| `app/services/robot_tcp_server.py` | TCP 服务器实现 |
| `app/api/endpoints/robot.py` | HTTP API 端点 |
| `app/services/behavior_tree_engine.py` | 行为树引擎 |
| `main.py` | 应用入口，TCP 服务器启动 |

---

*Version: 1.0.0*
*Last Updated: 2025-03-18*

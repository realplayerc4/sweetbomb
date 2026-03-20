# 项目规范 (Specification)

> 本文档定义项目的架构规范、API契约和组件规范。
> 所有开发工作必须遵循此规范，修改规范需通过评审。

---

## 1. 系统架构规范

### 1.1 分层架构

```
┌─────────────────────────────────────────────────────────┐
│                    Frontend Layer                        │
│  React + TypeScript + TailwindCSS + Three.js            │
├─────────────────────────────────────────────────────────┤
│                    API Gateway                           │
│  FastAPI Router + Dependencies Injection                 │
├─────────────────────────────────────────────────────────┤
│                    Service Layer                         │
│  rs_manager | webrtc_manager | sensor_control           │
├─────────────────────────────────────────────────────────┤
│                    Hardware Layer                        │
│  pyrealsense2 | Intel RealSense D400                    │
└─────────────────────────────────────────────────────────┘
```

### 1.2 通信协议规范

| 通道 | 协议 | 用途 | 延迟要求 |
|------|------|------|----------|
| 视频流 | WebRTC | RGB/Depth 视频传输 | < 100ms |
| 元数据 | Socket.IO | 设备状态、流事件 | < 500ms |
| 控制 | REST API | 参数配置、设备管理 | < 1s |

---

## 2. API 规范

### 3.1 RESTful 端点规范

#### 设备管理 `/api/devices`

| Method | Endpoint | 描述 | 响应码 |
|--------|----------|------|--------|
| GET | `/api/devices` | 枚举所有设备 | 200 |
| GET | `/api/devices/{device_id}` | 获取设备详情 | 200, 404 |

#### 传感器管理 `/api/devices/{device_id}/sensors`

| Method | Endpoint | 描述 | 响应码 |
|--------|----------|------|--------|
| GET | `/sensors` | 列出传感器 | 200 |
| GET | `/sensors/{sensor_id}/options` | 获取选项 | 200 |
| PUT | `/sensors/{sensor_id}/options` | 更新选项 | 200, 400 |

#### 流管理 `/api/devices/{device_id}/stream`

| Method | Endpoint | 描述 | 响应码 |
|--------|----------|------|--------|
| POST | `/stream/start` | 启动流 | 200, 409 |
| POST | `/stream/stop` | 停止流 | 200 |

### 3.2 WebRTC 端点

| Method | Endpoint | 描述 |
|--------|----------|------|
| POST | `/api/webrtc/offer` | 创建 WebRTC offer |
| POST | `/api/webrtc/ice-candidate` | 交换 ICE candidate |

### 3.3 Socket.IO 事件规范

#### 服务端 -> 客户端

| 事件名 | 载荷 | 描述 |
|--------|------|------|
| `device_connected` | `Device` | 设备连接 |
| `device_disconnected` | `{device_id: str}` | 设备断开 |
| `metadata` | `FrameMetadata` | 帧元数据 |
| `task_event` | `TaskEvent` | 任务事件 |
| `bt_event` | `{event_type, timestamp, ...}` | 行为树事件 |
| `bt_node_status` | `{node, status, path, timestamp}` | 节点状态变化 |

#### 任务事件类型

| event_type | 描述 |
|------------|------|
| `created` | 任务创建 |
| `started` | 任务启动 |
| `progress` | 进度更新 |
| `paused` | 任务暂停 |
| `resumed` | 任务恢复 |
| `stopped` | 任务停止 |
| `completed` | 任务完成 |
| `failed` | 任务失败 |

---

## 4. 数据模型规范

### 4.1 Device 模型

```python
class Device:
    id: str              # 设备唯一标识
    name: str            # 设备名称
    serial: str          # 序列号
    firmware: str        # 固件版本
    sensors: List[Sensor]
```

### 4.2 Task 模型

```python
class Task:
    id: str                    # 任务唯一标识 (UUID)
    task_type: str             # 任务类型标识
    device_id: Optional[str]   # 关联设备
    status: TaskStatus         # pending|running|paused|completed|failed
    progress: ProgressInfo     # 进度信息
    params: Dict[str, Any]     # 任务参数
    result: Optional[TaskResult]
```

### 4.3 SugarHarvestConfig 配置模型

```typescript
interface SugarHarvestConfig {
  // 导航点
  navigation_point: [number, number];  // [x, y] 取糖点坐标

  // 卸载点（废弃，保留向后兼容）
  dump_point?: [number, number];       // [x, y] 卸载点坐标

  // 卸载A点和B点（新增）
  dump_point_a: [number, number];     // [x, y] 卸载A点（等待位置）
  dump_point_b: [number, number];     // [x, y] 卸载B点（倾倒位置）

  // 机械参数
  bucket_width_m: number;             // 铲斗宽度（米）
  approach_offset_m: number;          // 接近偏移量（米）

  // 伺服角度
  scoop_position: number;               // 铲取角度（度）
  dump_position: number;                // 倾倒角度（度）
  lift_height?: number;                 // 举升高度（度，默认90）
  dump_angle?: number;                  // 倾倒角度（度，默认135）

  // 循环控制
  max_cycles: number;                   // 最大循环次数
  height_threshold_m: number;           // 推垛模式切换高度（米）

  // 回桩配置
  home_point?: [number, number];        // 充电桩位置 [x, y]
}
```

---

## 5. 约束与限制

### 5.1 并发限制

- 最大并发任务数: **4**
- 最大并发视频流: **2** (RGB + Depth)

### 5.2 硬件要求

- RealSense D400 系列 (D415/D435/D455)
- USB 3.0+ 接口 (高分辨率流)
- Ubuntu 20.04/22.04

### 5.3 命名约定

- API 路由: `snake_case`
- 前端组件: `PascalCase`
- 变量/函数: `camelCase` (前端), `snake_case` (后端)

---

## 6. 版本兼容性

| 组件 | 最低版本 | 推荐版本 |
|------|----------|----------|
| Python | 3.8 | 3.10+ |
| Node.js | 16 | 18+ |
| pyrealsense2 | 2.50 | 2.56+ |
| FastAPI | 0.100 | 0.115+ |

---

## 7. 安全规范

### 7.1 倾倒动作按钮确认机制

```
DumpAndReturn 节点执行到第4步时：
│
├─→ 发送 Socket.IO 事件 "bt_waiting_for_button"
│   Payload: { action: "dump_confirm", timeout: 30 }
│
├─→ 前端显示按钮确认弹窗
│   "请确认倾倒操作 - 翻转电机到 135°"
│   [确认] [取消]
│
├─→ 等待用户点击确认按钮
│   超时时间：30秒
│
├─→ 收到确认后执行倾倒动作
│   未收到确认则返回 FAILURE
│
└─→ 继续执行第5步（倒退）
```

### 7.2 安全规则

| 规则 | 说明 |
|------|------|
| **物料保护** | 铲糖/卸载过程中，翻转电机操作前必须按钮确认 |
| **位置保护** | 只有到达A点后才能举升，到达B点后才能倾倒 |
| **倒退安全** | 倾倒完成后必须倒退回A点（直接倒车，非导航） |
| **归零保护** | 回到A点后才能归零电机，防止空中归零导致物料掉落 |
| **车铲检查** | 每次循环开始前检查车铲是否放平（容差 ±2°） |

---

*规范版本: v1.2*
*最后更新: 2026-03-03*

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
│  rs_manager | task_manager | webrtc_manager              │
├─────────────────────────────────────────────────────────┤
│                    Hardware Layer                        │
│  pyrealsense2 | Intel RealSense D400                     │
└─────────────────────────────────────────────────────────┘
```

### 1.2 通信协议规范

| 通道 | 协议 | 用途 | 延迟要求 |
|------|------|------|----------|
| 视频流 | WebRTC | RGB/Depth 视频传输 | < 100ms |
| 元数据 | Socket.IO | 设备状态、任务事件 | < 500ms |
| 控制 | REST API | 参数配置、任务管理 | < 1s |

---

## 2. API 规范

### 2.1 RESTful 端点规范

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

#### 任务管理 `/api/tasks`

| Method | Endpoint | 描述 | 响应码 |
|--------|----------|------|--------|
| GET | `/api/tasks/types` | 获取任务类型 | 200 |
| POST | `/api/tasks/` | 创建任务 | 201, 400 |
| GET | `/api/tasks/` | 列出任务 | 200 |
| GET | `/api/tasks/{task_id}` | 任务详情 | 200, 404 |
| POST | `/api/tasks/{task_id}/start` | 启动任务 | 200, 409 |
| POST | `/api/tasks/{task_id}/pause` | 暂停任务 | 200 |
| POST | `/api/tasks/{task_id}/resume` | 恢复任务 | 200 |
| POST | `/api/tasks/{task_id}/stop` | 停止任务 | 200 |
| DELETE | `/api/tasks/{task_id}` | 删除任务 | 204, 404 |

### 2.2 数据模型规范

#### Device 模型
```python
class Device:
    id: str              # 设备唯一标识
    name: str            # 设备名称
    serial: str          # 序列号
    firmware: str        # 固件版本
    sensors: List[Sensor]
```

#### Task 模型
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

---

## 3. 组件规范

### 3.1 后端组件规范

#### Service 层规范
- **单例模式**: `rs_manager`, `task_manager` 必须是单例
- **线程安全**: 所有共享状态必须使用锁保护
- **异步优先**: I/O 操作必须使用 async/await

#### Task 系统规范
```python
class BaseTask(ABC):
    task_type: str           # 唯一标识
    name: str                # 显示名称
    description: str         # 任务描述
    category: str            # 分类
    requires_device: bool    # 是否需要设备

    @abstractmethod
    def validate(self) -> bool: ...

    @abstractmethod
    async def run(self) -> TaskResult: ...
```

### 3.2 前端组件规范

#### 组件分类
| 类型 | 目录 | 职责 |
|------|------|------|
| 页面组件 | `components/` | 业务逻辑组件 |
| UI组件 | `components/ui/` | 可复用基础组件 |
| Hooks | `hooks/` | 状态逻辑复用 |
| Services | `services/` | API 调用封装 |

#### 命名规范
- 组件文件: PascalCase (e.g., `TaskPanel.tsx`)
- Hook 文件: camelCase with `use` prefix (e.g., `useTaskManager.ts`)
- 服务文件: camelCase (e.g., `taskApi.ts`)

---

## 4. Socket.IO 事件规范

### 4.1 服务端 -> 客户端

| 事件名 | 载荷 | 描述 |
|--------|------|------|
| `device_connected` | `Device` | 设备连接 |
| `device_disconnected` | `{device_id: str}` | 设备断开 |
| `metadata` | `FrameMetadata` | 帧元数据 |
| `task_event` | `TaskEvent` | 任务事件 |

### 4.2 任务事件类型

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

## 5. 约束与限制

### 5.1 并发限制
- 最大并发任务数: **4**
- 最大并发视频流: **2** (RGB + Depth)

### 5.2 硬件要求
- RealSense D400 系列 (D415/D435/D455)
- USB 3.0+ 接口 (高分辨率流)
- Ubuntu 20.04/22.04

### 5.3 命名约定
- API 路由: snake_case
- 前端组件: PascalCase
- 变量/函数: camelCase (前端), snake_case (后端)

---

## 6. 版本兼容性

| 组件 | 最低版本 | 推荐版本 |
|------|----------|----------|
| Python | 3.8 | 3.10+ |
| Node.js | 16 | 18+ |
| pyrealsense2 | 2.50 | 2.56+ |
| FastAPI | 0.100 | 0.115+ |

---

*规范版本: v1.0*
*最后更新: 2025-02-24*

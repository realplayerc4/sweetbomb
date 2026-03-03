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

## 7. 行为树铲糖流程规范

### 7.1 行为树结构

```
RepeatNode (SugarHarvestMainLoop)
└── SequenceNode (FullHarvestCycle)
    ├── NavigateToSugarPoint       [导航到取糖点]
    ├── CheckShovelFlat            [检查车铲是否放平]
    ├── AnalyzeSugarDistance       [分析糖堆距离和高度]
    └── SelectorNode (HeightCheck)
        ├── 路径 A: 高度足够 → SequenceNode
        │   ├── CalculateApproachDistance  [计算前进距离]
        │   ├── ScoopAndReturn           [连贯动作：前进→翻转→倒退]
        │   ├── NavigateToDumpPoint      [导航到卸载点]
        │   └── DumpAction               [翻斗卸载]
        └── 路径 B: 高度不足 → ReturnToHome [回桩]
```

#### 核心设计思想

| 设计要点 | 说明 |
|----------|------|
| **条件分支** | 使用 SelectorNode 实现"如果高度足够则铲糖，否则回桩"的决策逻辑 |
| **动作封装** | 将"前进→翻转→倒退"三个机械动作封装为 ScoopAndReturn 单一节点，保证原子性 |
| **距离预计算** | 前进距离在 ScoopAndReturn 之前独立计算，便于日志记录和参数调整 |
| **失败处理** | 路径 B 直接回桩，避免机器人在低糖堆区域空转 |

### 7.2 动作节点规范

#### CalculateApproachDistance

| 属性 | 值 |
|------|-----|
| **职责** | 根据糖堆分析结果计算前进铲糖的距离 |
| **输入** | `distance_analysis` (黑板) |
| **输出** | `approach_distance` (黑板) |
| **参数** | `approach_offset_m` (默认 0.05m) |
| **计算** | `move_distance = distance_m - approach_offset` |

#### ScoopAndReturn

| 属性 | 值 |
|------|-----|
| **职责** | 连贯执行铲糖动作：前进 → 翻转铲子 → 倒退 |
| **输入** | `approach_distance`, `nav_point_position` (黑板) |
| **第1步** | 前进：根据 `approach_distance` 前进到糖堆 |
| **第2步** | 翻转：举升铲斗到 `scoop_position` (默认 90°) |
| **第3步** | 倒退：直接倒退回原位（非导航） |

#### ReturnToHome

| 属性 | 值 |
|------|-----|
| **职责** | 当糖堆高度不足时，导航回充电桩 |
| **参数** | `home_point` (任务参数) |
| **行为** | 导航到充电桩位置并广播事件 |

#### DumpAndReturn

| 属性 | 值 |
|------|-----|
| **职责** | 连贯执行卸载动作：导航A点→举升→导航B点→【按钮确认】→倾倒→倒退→归零 |
| **输入** | `dump_point_a`, `dump_point_b` (任务参数) |
| **参数** | `lift_height` (默认 90.0°), `dump_angle` (默认 135.0°) |
| **第1步** | 导航到卸载A点（等待位置） |
| **第2步** | 举升铲齿到 `lift_height` |
| **第3步** | 导航到卸载B点（倾倒位置） |
| **第4步** | 【按钮确认】翻转倾倒电机到 `dump_angle` 进行卸料 |
| **第5步** | 倒退（直接倒车）回到A点 |
| **第6步** | 倾倒电机归零，举升电机归零 |
| **安全机制** | 倾倒前需按钮确认，防止误触导致物料倾洒 |

### 7.3 前端配置参数

#### SugarHarvestConfig 接口

```typescript
interface SugarHarvestConfig {
  // 导航点
  navigation_point: [number, number];  // [x, y] 取糖点坐标

  // 卸载点（废弃，保留向后兼容）
  dump_point?: [number, number];       // [x, y] 卸载点坐标

  // 卸载A点和B点（新增）
  dump_point_a: [number, number];      // [x, y] 卸载A点（等待位置）
  dump_point_b: [number, number];      // [x, y] 卸载B点（倾倒位置）

  // 机械参数
  bucket_width_m: number;             // 铲斗宽度（米）
  approach_offset_m: number;            // 接近偏移量（米）

  // 伺服角度
  scoop_position: number;               // 铲取角度（度）
  dump_position: number;                // 倾倒角度（度）
  lift_height?: number;                 // 举升高度（度，默认90）

  // 循环控制
  max_cycles: number;                   // 最大循环次数
  height_threshold_m: number;           // 推垛模式切换高度（米）
}
```

### 7.4 黑板数据流

```yaml
NodeContext.blackboard:
├── nav_point_position       (NavigateToSugarPoint 记录，ScoopAndReturn 使用)
├── distance_analysis        (AnalyzeSugarDistance 记录)
├── approach_distance        (CalculateApproachDistance 记录，ScoopAndReturn 使用)
└── switch_to_push_mode      (废弃，现为回桩)
```

#### 节点数据依赖图

```
NavigateToSugarPoint
│   └── 输出: nav_point_position ──────────────┐
│                                               ▼
AnalyzeSugarDistance                            ScoopAndReturn
│   └── 输出: distance_analysis ────┐          (使用: nav_point_position,
│                                    ▼           approach_distance)
CalculateApproachDistance                          │
│   └── 输出: approach_distance ─────┘           ▼
│                                          DumpAndReturn
│                                          (使用: dump_point_a/b 参数)
```

### 7.5 安全机制与按钮确认

#### 倾倒动作按钮确认流程

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

#### 安全规则

| 规则 | 说明 |
|------|------|
| **物料保护** | 铲糖/卸载过程中，翻转电机操作前必须按钮确认 |
| **位置保护** | 只有到达A点后才能举升，到达B点后才能倾倒 |
| **倒退安全** | 倾倒完成后必须倒退回A点（直接倒车，非导航） |
| **归零保护** | 回到A点后才能归零电机，防止空中归零导致物料掉落 |

---

*规范版本: v1.1*
*最后更新: 2026-03-03*

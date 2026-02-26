# 知识库 (Knowledge Base)

> 本文档包含项目的技术知识、配置参考和常见问题解决方案。
> 作为开发参考手册，不定义规则，仅提供知识。

---

## 1. 技术栈详解

### 1.1 后端技术栈

#### FastAPI

- **用途**: 高性能异步 Web 框架
- **特性**: 自动 API 文档、类型验证、依赖注入
- **文档**: <https://fastapi.tiangolo.com/>

#### pyrealsense2

- **用途**: Intel RealSense SDK Python 绑定
- **核心类**:
  - `rs.context`: 设备上下文
  - `rs.device`: 设备实例
  - `rs.pipeline`: 流水线
  - `rs.filter`: 深度滤波器

#### aiortc

- **用途**: WebRTC Python 实现
- **核心类**:
  - `RTCPeerConnection`: WebRTC 连接
  - `VideoStreamTrack`: 视频轨道

### 1.2 前端技术栈

#### React + TypeScript

- **用途**: UI 组件化开发
- **状态管理**: React Hooks + Context

#### Three.js

- **用途**: 3D 点云渲染
- **核心组件**: `Points`, `BufferGeometry`, `PerspectiveCamera`, `OrthographicCamera`
- **实践避坑**:
  - 鸟瞰正交切片建议使用 `OrthographicCamera` 严格锁定视锥无透视形变。
  - 对于放大的密集点云阵列，尽量使用 `NormalBlending` 混合模式；如果滥用 `AdditiveBlending` 会导致粒子重叠区域严重过曝发白。

#### Socket.IO Client

- **用途**: 实时双向通信
- **事件模式**: pub/sub

---

## 2. RealSense 知识

### 2.1 深度滤波器

| 滤波器 | 功能 | 参数 |
|--------|------|------|
| Decimation | 降采样 | `magnitude` (1-8) |
| Spatial | 空间平滑 | `magnitude`, `smooth_alpha` |
| Temporal | 时间平滑 | `smooth_alpha`, `smooth_delta` |
| Hole Filling | 孔填充 | `mode` (0-3) |
| Threshold | 深度阈值 | `min_distance`, `max_distance` |

### 2.2 深度伪彩色映射

```
深度值 → Jet Colormap → RGB 颜色
         ↓
近距离 → 蓝色
中距离 → 绿色 → 黄色
远距离 → 红色
```

### 2.3 点云生成

```python
# 点云计算核心
pc = rs.pointcloud()
points = pc.calculate(depth_frame)
pc.map_to(color_frame)  # 纹理映射
```

### 2.4 IMU 数据处理 (加速度与陀螺仪)

RealSense 的 IMU (D435i/D455) 包含独立的高频流 `rs.stream.accel` 和 `rs.stream.gyro`。

- **重力欧拉角计算**：在低速运动下，可以通过加速度投影直接求出 Roll (翻滚角) 与 Pitch (俯仰角)：

  ```python
  import math
  roll = math.atan2(ay, az) * 180.0 / math.pi
  pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az)) * 180.0 / math.pi
  ```

- **航向角 Yaw 漂移说明**：由于 D455 等系列不带磁力计（Magnetometer），若想获取真实的 Yaw 必须配合陀螺仪积分和复杂滤波（如互补滤波）。强制单积分必定会出现严重常态漂移（Drifting），业务层可用拟合量替代。

---

## 3. WebRTC 知识

### 3.1 连接流程

```
Client                          Server
  │                               │
  │─────── offer SDP ────────────>│
  │<────── answer SDP ───────────│
  │─────── ICE candidates ───────>│
  │<────── ICE candidates ───────│
  │                               │
  │<═══════ Media Stream ════════│
```

### 3.2 视频编码

- **格式**: H.264 (优先) / VP8
- **分辨率**: 640x480 (默认), 最高 1280x720
- **帧率**: 30fps (默认)

---

## 4. 任务系统知识

### 4.1 任务生命周期

```
     ┌──────────┐
     │ pending  │ ◄─── 创建
     └────┬─────┘
          │ start()
          ▼
     ┌──────────┐
     │ running  │◄─────────────┐
     └────┬─────┘              │
          │                    │ resume()
    ┌─────┴─────┐              │
    │           │              │
    ▼           ▼              │
┌────────┐ ┌─────────┐         │
│paused  │─┤         │─────────┘
└────────┘ │         │
           │         │
           ▼         ▼
     ┌──────────┐
     │ stopped  │ ◄─── stop()
     └────┬─────┘
          │
          ▼
     ┌──────────┐
     │completed │ or │ failed │
     └──────────┘
```

### 4.2 内置任务类型

| 类型 | 类名 | 依赖 |
|------|------|------|
| `object_detection` | ObjectDetectionTask | ultralytics (YOLOv8) |
| `point_cloud_analysis` | PointCloudAnalysisTask | numpy, scipy |
| `data_collection` | DataCollectionTask | - |

### 4.3 任务注册机制

```python
# 装饰器注册
@register_task
class MyTask(BaseTask):
    task_type = "my_task"
    ...

# 自动发现
# 在 implementations/__init__.py 中导入即可
```

---

## 5. 配置参考

### 5.1 后端配置 (`config.py`)

| 配置项 | 默认值 | 描述 |
|--------|--------|------|
| `HOST` | `0.0.0.0` | 监听地址 |
| `PORT` | `8000` | 监听端口 |
| `DEBUG` | `False` | 调试模式 |
| `MAX_CONCURRENT_TASKS` | `4` | 最大并发任务 |

### 5.2 前端配置 (`config.ts`)

| 配置项 | 默认值 | 描述 |
|--------|--------|------|
| `API_BASE_URL` | `http://localhost:8000` | API 地址 |
| `SOCKET_URL` | `http://localhost:8000` | Socket.IO 地址 |

### 5.3 环境变量

| 变量 | 用途 |
|------|------|
| `REALSENSE_DEBUG` | 启用 RealSense 调试日志 |
| `WEBRTC_DEBUG` | 启用 WebRTC 调试日志 |

---

## 6. 常见问题解决

### 6.1 设备无法发现

**症状**: `/api/devices` 返回空列表

**排查步骤**:

1. 检查 USB 连接: `lsusb | grep Intel`
2. 检查权限: 确保用户在 `plugdev` 组
3. 检查 librealsense: `realsense-viewer`

### 6.2 WebRTC 连接失败

**症状**: 视频流无法显示

**排查步骤**:

1. 检查 ICE 候选: 浏览器控制台
2. 检查防火墙: 确保 UDP 端口开放
3. 检查编解码器: 浏览器兼容性

### 6.3 深度图噪点多

**解决方案**:

1. 启用空间滤波: `spatial_filter`
2. 启用时间滤波: `temporal_filter`
3. 调整激光功率: 增加 `laser_power`

### 6.4 点云显示异常

**排查步骤**:

1. 检查深度对齐: `align_to_color`
2. 检查点云密度: Decimation 滤波器
3. 检查 Three.js 缓冲区: `BufferGeometry` 更新

---

## 7. 性能优化知识

### 7.1 后端优化

- **帧队列**: 使用 `deque(maxlen=2)` 限制内存
- **异步处理**: 所有 I/O 操作使用 `async`
- **连接池**: Socket.IO 复用连接

### 7.2 前端优化

- **React.memo**: 避免不必要的重渲染
- **useCallback**: 稳定回调函数引用
- **WebGL**: 点云使用 `BufferGeometry`

---

## 8. 依赖关系图

```
┌─────────────────────────────────────────────┐
│                   Frontend                   │
│  ┌─────────┐  ┌─────────┐  ┌─────────────┐ │
│  │ React   │  │ Three.js│  │ Socket.IO   │ │
│  └────┬────┘  └────┬────┘  └──────┬──────┘ │
└───────┼────────────┼──────────────┼────────┘
        │            │              │
        ▼            ▼              ▼
┌─────────────────────────────────────────────┐
│                  Backend                     │
│  ┌─────────┐  ┌─────────┐  ┌─────────────┐ │
│  │ FastAPI │  │ aiortc  │  │ Socket.IO   │ │
│  └────┬────┘  └────┬────┘  └──────┬──────┘ │
│       │            │              │        │
│       └────────────┼──────────────┘        │
│                    ▼                        │
│            ┌─────────────┐                 │
│            │ pyrealsense2│                 │
│            └──────┬──────┘                 │
└───────────────────┼─────────────────────────┘
                    ▼
            ┌─────────────┐
            │ RealSense   │
            │ Hardware    │
            └─────────────┘
```

---

*知识库版本: v1.0*
*最后更新: 2025-02-24*

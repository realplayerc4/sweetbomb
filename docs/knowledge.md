# 知识库 (Knowledge Base)

> 本文档收录 Sweetbomb 项目的技术知识、配置参考、常见问题及最佳实践。
> 发现新知识时，请更新此文档。

---

## 1. 技术栈详解

### 1.1 后端技术

| 技术 | 版本 | 用途 | 关键配置 |
|------|------|------|----------|
| **FastAPI** | 0.115+ | Web 框架 | `lifespan=app_lifespan` |
| **Uvicorn** | 0.30+ | ASGI 服务器 | `workers=1` (RealSense 单例) |
| **Socket.IO** | 5.11+ | 实时通信 | `async_mode='asgi'` |
| **aiortc** | 1.6+ | WebRTC | `RTCPeerConnection` |
| **Pydantic** | 2.0+ | 数据验证 | `model_config` |

### 1.2 前端技术

| 技术 | 版本 | 用途 |
|------|------|------|
| **React** | 18+ | UI 框架 |
| **TypeScript** | 5.0+ | 类型系统 |
| **Vite** | 5.0+ | 构建工具 |
| **Three.js** | 0.160+ | 3D 渲染 |
| **TailwindCSS** | 3.4+ | 样式框架 |
| **shadcn/ui** | - | UI 组件库 |

### 1.3 硬件技术

| 硬件 | 型号 | 规格 |
|------|------|------|
| **深度相机** | Intel RealSense D455 | RGB: 1280x720@30fps, Depth: 1280x720@30fps |
| **计算平台** | PC/工控机 | USB 3.0+ 接口 |
| **控制协议** | WebSocket/Serial | 自定义协议 |

---

## 2. 配置参考

### 2.1 RealSense 流配置

```python
# 推荐配置（低延迟优先）
STREAM_CONFIG = {
    "rgb": {
        "width": 1280,
        "height": 720,
        "fps": 30,
        "format": "RGB8"
    },
    "depth": {
        "width": 1280,
        "height": 720,
        "fps": 30,
        "format": "Z16"
    }
}

# WebRTC 编码配置
WEBRTC_CONFIG = {
    "video_codec": "H264",  # 或 "VP8"
    "bitrate": 2000000,     # 2 Mbps
    "keyframe_interval": 60
}
```

### 2.2 深度滤波器配置

```python
# 深度滤波器链（按推荐顺序）
DEPTH_FILTERS = {
    "decimation": {
        "magnitude": 2,  # 降采样因子
        "enabled": True
    },
    "spatial": {
        "magnitude": 2,
        "alpha": 0.5,
        "delta": 20,
        "enabled": True
    },
    "temporal": {
        "alpha": 0.4,
        "delta": 20,
        "enabled": True
    },
    "hole_filling": {
        "mode": "farest_from_around",
        "enabled": False  # 按需启用
    }
}
```

### 2.3 铲糖任务配置

```json
{
  "task_type": "sugar_harvest",
  "config": {
    "navigation_point": [3.5, 2.0],
    "dump_point_a": [1.0, 1.0],
    "dump_point_b": [1.0, 1.5],
    "bucket_width_m": 0.6,
    "approach_offset_m": 0.3,
    "scoop_position": 90.0,
    "dump_position": 135.0,
    "lift_height": 90.0,
    "dump_angle": 135.0,
    "max_cycles": 5,
    "height_threshold_m": 0.20,
    "home_point": [0.0, 0.0]
  }
}
```

---

## 3. 常见问题 (FAQ)

### Q1: 点云在俯视角下消失

**症状**: 当相机拉远时，点云突然消失。

**原因**: Three.js 的视锥体裁剪 (frustum culling)。

**解决**:
```typescript
// 设置 frustumCulled: false
points = new THREE.Points(geometry, material);
points.frustumCulled = false;
```

### Q2: Socket.IO 连接 403 错误

**症状**: 前端无法连接到后端 WebSocket。

**原因**: Socket.IO 路径未正确挂载到 FastAPI。

**解决**:
```python
# 确保使用 socketio.ASGIApp 包装
app.mount("/", socketio.ASGIApp(sio, other_asgi_app=app))
```

### Q3: RealSense 设备无法启动流

**症状**: `rs.pipeline.start(config)` 报错。

**排查步骤**:
1. 检查 USB 连接: `rs-enumerate-devices`
2. 检查固件版本: `rs-fw-update -l`
3. 降低分辨率/FPS 测试
4. 检查是否有其他程序占用设备

### Q4: WebRTC 视频流延迟高

**优化建议**:
1. 使用 H.264 硬件编码 (如果支持)
2. 降低分辨率和码率
3. 调整关键帧间隔
4. 使用 UDP 模式 (默认)

### Q5: IMU 数据异常波动

**解决**:
```python
# 启用时间戳同步
config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 400)

# 使用 MotionModule 的回调同步
```

---

## 4. 最佳实践

### 4.1 行为树设计

```
✅ 推荐:
- 每个节点职责单一
- 使用黑板共享数据
- 异常处理完备
- 状态广播实时

❌ 避免:
- 节点嵌套过深 (> 5 层)
- 在叶子节点直接操作硬件
- 忽略失败状态
- 阻塞式长时间运行
```

### 4.2 性能优化

| 场景 | 优化策略 |
|------|----------|
| 点云渲染 | 使用 BufferGeometry + Points，frustumCulled: false |
| WebRTC | H.264 硬件编码，降低分辨率 |
| Socket.IO | 批量发送，压缩大消息 |
| RealSense | 降采样滤波器，按需启动流 |

### 4.3 错误处理

```python
# 推荐模式：try-except + 日志 + 状态广播
try:
    result = await operation()
except SpecificError as e:
    logger.error(f"操作失败: {e}", exc_info=True)
    await broadcast_error(context, str(e))
    return NodeStatus.FAILURE
```

---

## 5. 参考资源

### 5.1 官方文档

- [RealSense SDK 2.0](https://dev.intelrealsense.com/docs)
- [FastAPI](https://fastapi.tiangolo.com/)
- [Socket.IO](https://socket.io/docs/v4/)
- [Three.js](https://threejs.org/docs/)
- [aiortc](https://aiortc.readthedocs.io/)

### 5.2 相关论文

- Behavior Trees in Robotics and AI: [Colledanchise & Ögren, 2018]

---

*知识库版本: v1.0*
*最后更新: 2026-03-03*

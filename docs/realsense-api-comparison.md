# Intel RealSense 标准 REST API vs. Sweetbomb 差异对比

> 本文档对比分析 Intel RealSense 标准 REST API（realsenseai/librealsense `wrappers/rest-api/`）与 sweetbomb 项目实现的差异，为后续兼容性修复提供参考。

---

## 执行摘要

**核心结论**：Sweetbomb 是标准 API 的超集。核心端点结构基本对齐，但在路径命名、HTTP 方法和响应字段上存在分歧。

| 兼容性评级 | 功能领域 |
|-----------|----------|
| **高** | 设备列表、传感器列表、选项列表、流启停、点云激活/关闭、WebRTC offer/answer/ICE、Socket.IO metadata_update |
| **中** | DeviceInfo（额外字段）、选项更新（PUT 单个匹配，PATCH 批量缺失）、WebRTC stream_map（增量安全） |
| **低** | 流状态路径、硬件重置路径命名、系统状态字段缺失 |
| **无** | 软件重置、motion_data IMU、纹理映射（Sweetbomb 未实现）；机器人控制、地图管理、点云分析（仅 Sweetbomb） |

---

## 1. 端点路径差异（会导致 404）

### 1.1 流状态端点

| 标准 API | Sweetbomb |
|----------|-----------|
| `GET /api/devices/{id}/stream/` | `GET /api/devices/{device_id}/stream/status` |

**影响**：调用 `GET .../stream/` 的客户端收到 404。

**源文件**：
- Sweetbomb：`app/api/endpoints/streams.py` 第 48 行

---

### 1.2 硬件重置路径命名

| 标准 API | Sweetbomb |
|----------|-----------|
| `POST /api/devices/{id}/hardware-reset` | `POST /api/devices/{device_id}/hw_reset` |

**影响**：期望 `hardware-reset` 的客户端收到 404。

**源文件**：
- Sweetbomb：`app/api/endpoints/devices.py` 第 11-41 行

---

### 1.3 批量选项更新

| 标准 API | Sweetbomb |
|----------|-----------|
| `PATCH /api/devices/{id}/sensors/{sensor_id}/options/`（批量，请求体 `{option_id: value}` 字典） | 只有 `PUT .../options/{option_id}`（单个更新） |

**影响**：发送批量 PATCH 的客户端收到 405 Method Not Allowed。

**源文件**：
- Sweetbomb：`app/api/endpoints/options.py` 第 40-54 行

---

### 1.4 缺失端点

| 端点 | 标准 API | Sweetbomb |
|------|----------|-----------|
| 软件重置 | `POST /api/devices/{id}/software-reset` | **未实现** |
| 纹理映射 | 有 | **未实现** |

---

## 2. 响应字段差异

### 2.1 system_stats（Socket.IO metadata_update）

| 字段 | 标准 API | Sweetbomb | 说明 |
|------|----------|-----------|------|
| cpu_load | 有（真实值 psutil） | 有（真实值 psutil） | 匹配 |
| hostname | 有 | 有 | 匹配 |
| battery | 有（sysfs 读取） | **缺失** | 会返回 undefined |
| temperature | 有（sysfs 读取） | **缺失** | 会返回 undefined |
| signal | 有（/proc/net/wireless） | **缺失** | 会返回 undefined |

**源文件**：`app/services/metadata_socket_server.py` 第 114-123 行

当前 sweetbomb system_stats：
```python
system_stats = {
    "cpu_load": last_cpu_load,
    "hostname": socket.gethostname(),
}
```

---

### 2.2 DeviceInfo 响应

| 字段 | 标准 API | Sweetbomb | 说明 |
|------|----------|-----------|------|
| device_id | 有 | 有 | 匹配 |
| name | 有 | 有 | 匹配 |
| serial_number | 有 | 有 | 匹配 |
| firmware_version | 有 | 有（Optional） | 匹配 |
| physical_port | 有 | 有（Optional） | 匹配 |
| usb_type | 有 | 有（Optional） | 匹配 |
| product_id | 有 | 有（Optional） | 匹配 |
| sensors | 无 | 有（`List[str]`） | **Sweetbomb 新增** — 增量安全 |
| is_streaming | 无 | 有（`bool`） | **Sweetbomb 新增** — 增量安全 |

---

### 2.3 WebRTC Offer 响应

| 字段 | 标准 API | Sweetbomb | 说明 |
|------|----------|-----------|------|
| session_id | 有 | 有 | 匹配 |
| sdp | 有 | 有 | 匹配 |
| type | 有 | 有 | 匹配 |
| stream_map | 无 | 有（`Dict[str, str]`） | **Sweetbomb 新增** — mid 到 stream_type 映射 |

---

## 3. 点云处理对比

### 3.1 数据格式

**完全兼容**：两者都使用 `base64(Float32Array XYZ)` 编码。

- 传输通道：Socket.IO `metadata_update` 事件
- 字段路径：`metadata_streams.depth.point_cloud.vertices`
- 前端解码：`atob(base64) -> Uint8Array -> Float32Array`

**源文件**：`app/services/metadata_socket_server.py` 第 98-101 行

```python
metadata["point_cloud"]["vertices"] = base64.b64encode(
    metadata["point_cloud"]["vertices"].tobytes()
).decode("utf-8")
```

---

### 3.2 Sweetbomb 点云去重机制

基于深度帧时间戳，时间戳不变时跳过重新编码，防止重复发送相同数据。

**源文件**：`app/services/metadata_socket_server.py` 第 92-97 行

---

### 3.3 Sweetbomb 点云分析（标准 API 无）

| 功能 | 实现位置 |
|------|----------|
| 体积计算 | `app/services/point_cloud_analyzer.py` |
| 物料距离检测 | `app/services/point_cloud_analyzer.py` |
| 堆体高度测量 | `app/services/point_cloud_analyzer.py` |
| 前进距离计算 | `app/services/rs_manager.py` 第 164-176 行 |

**API 端点**：
- `GET /api/devices/{device_id}/point_cloud/analysis`
- `GET /api/devices/{device_id}/point_cloud/move_distance`

---

## 4. WebRTC 实现对比

### 4.1 架构

| 方面 | 标准 API | Sweetbomb |
|------|----------|-----------|
| 信令角色 | 后端 offerer | 后端 offerer — 匹配 |
| WebRTC 库 | wrtc (Node.js) | aiortc (Python) |
| 会话管理 | 内存 | 内存 + TTL 自动清理（1小时过期） |

---

### 4.2 ICE 服务器配置

| 方面 | 标准 API | Sweetbomb |
|------|----------|-----------|
| STUN | 硬编码 Google STUN | 可配置，默认 None |
| TURN | 未记录 | 可配置，默认 None |

**部署说明**：Sweetbomb WebRTC 用于局域网访问（192.168.3.101:5173），无需 STUN/TURN。

**源文件**：`config.py` 第 13-16 行

---

### 4.3 Sweetbomb 额外端点

| 端点 | 说明 |
|------|------|
| `GET /api/webrtc/sessions/{session_id}` | 获取会话详情 |
| `GET /api/webrtc/sessions/{session_id}/ice-candidates` | 获取 ICE candidates |
| `DELETE /api/webrtc/sessions/{session_id}` | 关闭会话 |

---

## 5. Socket.IO 事件对比

| 事件 | 标准 API | Sweetbomb | 说明 |
|------|----------|-----------|------|
| `metadata_update` | 有 | 有 | 结构匹配 |
| `motion_data` | 有（IMU 数据） | **缺失** | D435i 等设备 IMU 不传播 |
| `bt_event` | 无 | 有 | **Sweetbomb 自定义** |
| `bt_node_status` | 无 | 有 | **Sweetbomb 自定义** |
| `task_event` | 无 | 有 | **Sweetbomb 自定义** |

---

## 6. Sweetbomb 独有功能（标准 API 完全没有）

### 6.1 机器人控制系统

- TCP 服务器：端口 9090（非 HTTP）
- 自定义协议：UTF-8 文本，`{...}` 定界
- REST 端点：`GET /api/robot/status`, `GET /api/robot/connection`

**源文件**：`app/services/robot_tcp_server.py`

---

### 6.2 地图管理

- txt 格式网格地图 → PNG/SVG 转换
- 缓存机制
- REST 端点：`GET /api/map/`, `GET /api/map/{name}.png`, `GET /api/map/{name}/info`

**源文件**：`app/api/endpoints/map.py`

---

### 6.3 自动流管理

守护线程每 5 秒检查，自动启动所有检测设备的：
- 深度流：640x360@15fps z16
- 彩色流：640x360@30fps rgb8
- 点云处理

**源文件**：`app/services/rs_manager.py` 第 108-162 行

---

### 6.4 健康检查系统

10 分钟循环验证所有活跃流正常工作。

**源文件**：`main.py` 第 70-80 行

---

### 6.5 后处理滤波器

| 滤波器 | 配置 |
|--------|------|
| 降采样 | 幅度 3 |
| 空间滤波 | 默认 |
| 时间滤波 | 默认 |
| 彩色化器 | 色彩方案 0，直方图均衡化，1.0-6.0m |
| 阈值滤波 | 1.0-6.0m |

`StreamStart` 的 `apply_filters` 标志控制启用。

**源文件**：`app/services/rs_manager.py` 第 44-63 行

---

## 7. 兼容性修复建议

按优先级排序：

| 优先级 | 修复项 | 源文件 |
|--------|--------|--------|
| P1 | 流状态别名：`GET .../stream/` → 复用 `/status` | `app/api/endpoints/streams.py` |
| P1 | 硬件重置别名：`POST .../hardware-reset` → 复用 `hw_reset` | `app/api/endpoints/devices.py` |
| P2 | 批量 PATCH 选项更新 | `app/api/endpoints/options.py` |
| P2 | 恢复 battery/temperature/signal 到 system_stats | `app/services/metadata_socket_server.py` |
| P3 | 实现软件重置端点 | `app/api/endpoints/devices.py` |
| P3 | 添加 motion_data IMU 支持 | `app/services/rs_manager.py` + `metadata_socket_server.py` |

---

## 8. 关键文件清单

| 文件路径 | 职责 |
|----------|------|
| `app/api/router.py` | API 路由聚合 |
| `app/api/endpoints/devices.py` | 设备管理、重置 |
| `app/api/endpoints/streams.py` | 流启停、状态 |
| `app/api/endpoints/options.py` | 传感器选项 |
| `app/api/endpoints/webrtc.py` | WebRTC 信令 |
| `app/api/endpoints/point_cloud.py` | 点云激活/分析 |
| `app/api/endpoints/robot.py` | 机器人状态 |
| `app/services/rs_manager.py` | RealSense 流管理、自动流、滤波器 |
| `app/services/webrtc_manager.py` | WebRTC 会话管理 |
| `app/services/metadata_socket_server.py` | Socket.IO 元数据广播 |
| `app/services/point_cloud_analyzer.py` | 点云分析管道 |
| `app/services/robot_tcp_server.py` | 机器人 TCP 服务器 |
| `config.py` | 全局配置 |

---

*生成时间: 2026-04-07*
*参考源: https://github.com/realsenseai/librealsense/tree/master/wrappers/rest-api*
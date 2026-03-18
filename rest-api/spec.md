# REST API Specification

> RealSense Web Monitor - API 规范文档

---

## 基础信息

| 属性 | 值 |
|------|-----|
| 基础 URL | `http://localhost:8000/api/v1` |
| 协议 | HTTP/REST + WebSocket + Socket.IO |
| 内容类型 | `application/json` |

---

## 全局错误响应

```json
{
  "detail": "错误描述信息"
}
```

HTTP 状态码:
- `200` - 成功
- `400` - 请求参数错误
- `404` - 资源不存在
- `500` - 服务器内部错误

---

## API 端点分类

### 1. 设备管理 (Devices)

#### 获取设备列表
```
GET /devices/
```
**响应**: `List[DeviceInfo]`

#### 获取设备详情
```
GET /devices/{device_id}
```

#### 硬件重置
```
POST /devices/{device_id}/hw_reset
```

---

### 2. 传感器管理 (Sensors)

#### 获取传感器列表
```
GET /devices/{device_id}/sensors/
```

#### 获取传感器详情
```
GET /devices/{device_id}/sensors/{sensor_id}
```

---

### 3. 选项控制 (Options)

#### 获取选项列表
```
GET /devices/{device_id}/sensors/{sensor_id}/options/
```

#### 获取选项详情
```
GET /devices/{device_id}/sensors/{sensor_id}/options/{option_name}
```

#### 设置选项值
```
PUT /devices/{device_id}/sensors/{sensor_id}/options/{option_name}
```
**请求体**:
```json
{
  "value": <option_value>
}
```

---

### 4. 流控制 (Streams)

#### 启动流
```
POST /devices/{device_id}/stream/start/
```
**请求体**: `StreamRequest`

#### 停止流
```
POST /devices/{device_id}/stream/stop/
```

---

### 5. 点云管理 (Point Cloud)

#### 激活点云
```
POST /devices/{device_id}/point_cloud/activate
```
**参数**: `device_id` (query)

#### 关闭点云
```
POST /devices/{device_id}/point_cloud/deactivate
```

#### 获取点云状态
```
GET /devices/{device_id}/point_cloud/status
```

#### 获取点云分析结果
```
GET /devices/{device_id}/point_cloud/analysis
```
**响应**: `PointCloudAnalysisResponse`

```json
{
  "device_id": "string",
  "volume": 0.5,
  "target_volume": 1.0,
  "volume_reached": false,
  "pile_height": 0.3,
  "material_distance": 0.8,
  "nearest_point": [0.5, 0.2, 0.3],
  "has_material": true,
  "timestamp": "2024-01-01T00:00:00"
}
```

#### 获取前进距离
```
GET /devices/{device_id}/point_cloud/move_distance
```
**参数**: `approach_offset` (默认 0.05m)

---

### 6. WebRTC 信令 (WebRTC)

#### 创建 Offer
```
POST /webrtc/offer
```
**请求体**:
```json
{
  "device_id": "string",
  "stream_types": ["color", "depth"]
}
```

#### 发送 Answer
```
POST /webrtc/answer/
```

#### 添加 ICE 候选
```
POST /webrtc/ice-candidates
```

#### 获取 ICE 候选
```
GET /webrtc/sessions/{session_id}/ice-candidates/
```

#### 获取会话状态
```
GET /webrtc/sessions/{session_id}
```

#### 关闭会话
```
DELETE /webrtc/sessions/{session_id}
```

---

### 7. 任务管理 (Tasks)

#### 获取任务类型
```
GET /tasks/types
```

#### 获取任务类型详情
```
GET /tasks/types/{task_type}
```

#### 创建任务
```
POST /tasks/
```

#### 列出任务
```
GET /tasks/
```
**查询参数**: `status`, `device_id`, `task_type`

#### 获取任务详情
```
GET /tasks/{task_id}
```

#### 删除任务
```
DELETE /tasks/{task_id}
```

#### 启动任务
```
POST /tasks/{task_id}/start
```

#### 暂停任务
```
POST /tasks/{task_id}/pause
```

#### 恢复任务
```
POST /tasks/{task_id}/resume
```

#### 停止任务
```
POST /tasks/{task_id}/stop
```

---

### 8. 航点管理 (Waypoints)

#### 列出航点
```
GET /waypoints/
```

#### 创建航点
```
POST /waypoints/
```

#### 删除航点
```
DELETE /waypoints/{name}
```

---

### 9. 机器人控制 (Robot) ⚠️ 重大变更

> **警告**: 机器人控制已从 HTTP REST API 变更为 **TCP 协议 (端口 9090)**

#### 通过 TCP 协议通信 (端口 9090)

**连接信息**:
- 地址: `0.0.0.0:9090`
- 编码: UTF-8
- 协议: 文本协议

**报文格式**:
```
{
MessageType=<类型>
Key1=Value1
Key2=Value2
}
```

**支持的消息类型**:
- `status` - 状态查询/上报
- `task` - 任务下发
- `taskFinish` - 任务完成
- `cancelTask` - 取消任务
- `command` - 控制命令
- `remoteControl` - 远程控制
- `cameraCheckDistance` - 相机检测距离

#### 保留的 HTTP API 端点

以下端点仍通过 HTTP 提供:

##### 获取机器人状态 (TCP)
```
GET /robot/tcp/status
```
返回通过 TCP 连接的机器人状态列表。

##### 伺服控制 (HTTP)
```
POST /robot/servo
```
**请求体**:
```json
{
  "servo_id": "lift" | "dump",
  "angle": 90
}
```

##### 铲取/倾倒动作 (HTTP)
```
POST /robot/scoop
POST /robot/dump
POST /robot/dock
```

##### 距离分析 (HTTP)
```
POST /robot/distance_analyze
```

##### 铲糖自主循环 (HTTP)
```
POST /robot/auto_cycle/start
POST /robot/auto_cycle/stop
GET /robot/auto_cycle/status
```

##### WebSocket 状态推送
```
WS /robot/ws
```
实时推送机器人 TCP 连接状态变化。

---

## 数据模型

### DeviceInfo
```json
{
  "device_id": "string",
  "name": "string",
  "serial_number": "string",
  "firmware_version": "string"
}
```

### StreamConfig
```json
{
  "sensor_id": "string",
  "stream_type": "color" | "depth" | "infrared",
  "format": "string",
  "resolution": {
    "width": 640,
    "height": 480
  },
  "framerate": 30,
  "enable": true
}
```

### PointCloudAnalysisResponse
```json
{
  "device_id": "string",
  "volume": 0.5,
  "target_volume": 1.0,
  "volume_reached": false,
  "pile_height": 0.3,
  "material_distance": 0.8,
  "nearest_point": [0.5, 0.2, 0.3],
  "has_material": true,
  "timestamp": "2024-01-01T00:00:00"
}
```

---

## 版本历史

| 版本 | 日期 | 变更内容 |
|-----|------|---------|
| 1.0.0 | 2024-03-17 | 初始版本，完整 API 规范 |

---

*注意: 本文档基于实际代码分析生成，取代了原先缺失的 spec.md 文件。*

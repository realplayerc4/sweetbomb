# restrealsenseMonitor (RealSense Web Wrapper)

`restrealsenseMonitor` 是一个基于 **FastAPI** 和 **React** 构建的现代化 RealSense 监控与控制平台。它支持通过 RESTful API 管理设备，利用 **WebRTC** 进行低延迟的实时视频流传输（RGB 和 深度图），并通过 **Socket.IO** 实时推送元数据和点云数据。

本项目旨在提供一个开箱即用的 Web 界面，用于远程监控和调试 Intel RealSense D400 系列摄像头。

A modern RealSense monitoring and control platform built with **FastAPI** and **React**. It supports device management via RESTful API, low-latency real-time video streaming (RGB and Depth) using **WebRTC**, and real-time metadata/point cloud streaming via **Socket.IO**.

## 🌟 核心功能 (Features)

* **设备管理**: 实时发现与枚举连接的 RealSense 设备。
* **参数配置**: 在线调整传感器参数（曝光、增益、激光功率等）。
* **实时预览**:
  * **低延迟**: 基于 WebRTC 的毫秒级视频传输。
  * **深度伪彩**: 支持 Jet 伪彩色映射与直方图均衡化，深度细节清晰可见。
  * **点云可视化**: 集成 Three.js 的 3D 点云视图。
* **数据流**:
  * RGB / Depth 双流同步。
  * Socket.IO 实时元数据推送。
* **任务系统** (NEW):
  * 可扩展的任务框架，支持多种 AI 模型并行运行。
  * 内置目标检测、点云分析、数据采集任务。
  * 实时进度跟踪和 Socket.IO 事件广播。

## 🏗️ 技术架构 (Architecture)

* **Backend (后端)**: Python FastAPI + pyrealsense2 + aiortc
* **Frontend (前端)**: React + TypeScript + Vite + TailwindCSS + Three.js

## 🚀 快速开始 (Quick Start)

### 1. 环境要求 (Prerequisites)

* **OS**: Linux (Ubuntu 20.04/22.04 推荐)
* **Hardware**: Intel RealSense D400 Series (D415, D435, D455)
* **Runtime**: Python 3.8+, Node.js 16+

### 2. 后端启动 (Backend Setup)

```bash
cd rest-api

# 1. 创建并激活虚拟环境
python3 -m venv venv
source venv/bin/activate

# 2. 安装依赖
pip install -r requirements.txt

# 3. 启动服务
chmod +x start_server.sh
./start_server.sh
```

后端服务将运行在 `http://localhost:8000`。
* API 文档: `http://localhost:8000/docs`

### 3. 前端启动 (Frontend Setup)

```bash
cd rest-api/ui/frontend

# 1. 安装依赖
npm install

# 2. 启动开发服务器
npm run dev
```

前端页面将运行在 `http://localhost:5173`。

## 📖 使用指南 (Usage)

1. **启动服务**: 确保后端 (`./start_server.sh`) 和前端 (`npm run dev`) 均已启动。
2. **访问界面**: 打开浏览器访问 `http://localhost:5173`。
3. **连接设备**: 界面会自动发现连接的 RealSense 设备。
4. **开启视频流**: 点击右上角的 **"启动"** 按钮，即可看到实时的 RGB 和深度视频流。
5. **查看点云**: 页面下方提供了基于 WebGL 的点云视图，支持鼠标拖拽旋转查看。
6. **任务管理**: 在控制面板的 **"任务"** 标签页中创建和管理 AI 任务。

## ⚠️ 注意事项 (Notes)

* **硬件兼容性**: 请确保系统已安装 `librealsense2` 运行库。
* **USB 带宽**: 同时开启高分辨率 RGB 和深度流需要 USB 3.0+ 接口，请确保使用原装或高质量数据线。
* **多设备**: 目前前端界面主要针对单设备优化，后端支持多设备枚举。

## 目录结构 (Directory Structure)

```
rest-api/
├── app/                          # 后端核心代码 (FastAPI)
│   ├── api/
│   │   ├── endpoints/
│   │   │   ├── devices.py        # 设备管理 API
│   │   │   ├── streams.py        # 视频流 API
│   │   │   ├── webrtc.py         # WebRTC API
│   │   │   ├── tasks.py          # 任务管理 API (NEW)
│   │   │   └── ...
│   │   ├── dependencies.py       # 依赖注入
│   │   └── router.py             # 路由配置
│   ├── models/
│   │   ├── task.py               # 任务数据模型 (NEW)
│   │   └── ...
│   ├── services/
│   │   ├── task_manager.py       # 任务管理器 (NEW)
│   │   ├── tasks/                # 任务系统 (NEW)
│   │   │   ├── base_task.py      # 任务基类
│   │   │   ├── registry.py       # 任务注册表
│   │   │   └── implementations/  # 任务实现
│   │   │       └── object_detection_task.py
│   │   ├── rs_manager.py         # RealSense 管理器
│   │   └── webrtc_manager.py     # WebRTC 管理器
│   └── ...
├── ui/frontend/                  # 前端源代码 (React)
│   └── src/app/
│       ├── components/
│       │   ├── TaskPanel.tsx     # 任务面板组件 (NEW)
│       │   └── ...
│       ├── hooks/
│       │   ├── useTaskManager.ts # 任务管理 Hook (NEW)
│       │   └── useRobotConnection.ts
│       ├── services/
│       │   ├── taskApi.ts        # 任务 API 客户端 (NEW)
│       │   └── api.ts
│       └── App.tsx
├── config.py                     # 后端配置文件
└── start_server.sh               # 后端启动脚本
```

---

## 🤖 任务系统 (Task System)

任务系统是一个可扩展的框架，支持在 RealSense 数据流上运行多种 AI 模型和分析任务。

### 核心特性

* **可扩展架构**: 通过继承 `BaseTask` 类轻松添加新任务类型
* **并发控制**: 最多支持 4 个任务并行运行
* **实时更新**: 通过 Socket.IO 实时推送任务进度和状态
* **生命周期管理**: 支持 启动/暂停/恢复/停止 操作

### 内置任务类型

| 任务类型 | 名称 | 描述 |
|---------|------|------|
| `object_detection` | 目标检测 | 使用深度学习模型检测视频帧中的物体 |
| `point_cloud_analysis` | 点云分析 | 对深度点云进行法线估计、平面分割等分析 |
| `data_collection` | 数据采集 | 从传感器采集数据并保存到文件 |

### API 端点

| Method | Endpoint | 描述 |
|--------|----------|------|
| GET | `/api/tasks/types` | 获取可用任务类型 |
| POST | `/api/tasks/` | 创建任务 |
| GET | `/api/tasks/` | 列出所有任务 |
| GET | `/api/tasks/{task_id}` | 获取任务详情 |
| POST | `/api/tasks/{task_id}/start` | 启动任务 |
| POST | `/api/tasks/{task_id}/pause` | 暂停任务 |
| POST | `/api/tasks/{task_id}/resume` | 恢复任务 |
| POST | `/api/tasks/{task_id}/stop` | 停止任务 |
| DELETE | `/api/tasks/{task_id}` | 删除任务 |

### 使用示例

```bash
# 获取可用任务类型
curl http://localhost:8000/api/tasks/types

# 创建目标检测任务
curl -X POST http://localhost:8000/api/tasks/ \
  -H "Content-Type: application/json" \
  -d '{
    "task_type": "object_detection",
    "device_id": "your-device-id",
    "params": {
      "model": "yolov8n",
      "confidence_threshold": 0.5
    }
  }'

# 启动任务
curl -X POST http://localhost:8000/api/tasks/{task_id}/start

# 查看任务状态
curl http://localhost:8000/api/tasks/{task_id}
```

### Socket.IO 事件

任务系统通过 `task_event` 通道广播实时更新：

```javascript
socket.on('task_event', (event) => {
  // event.event_type: created | started | progress | paused | resumed | stopped | completed | failed
  // event.task_id: 任务 ID
  // event.status: 任务状态
  // event.progress: 进度信息
  // event.result: 任务结果（完成时）
});
```

### 添加自定义任务

1. 创建新的任务类，继承 `BaseTask`：

```python
# app/services/tasks/implementations/my_task.py
from app.services.tasks.base_task import BaseTask
from app.services.tasks.registry import register_task
from app.models.task import TaskResult

@register_task
class MyCustomTask(BaseTask):
    task_type = "my_custom_task"
    name = "自定义任务"
    description = "这是一个自定义任务示例"
    category = "custom"
    requires_device = True

    def validate(self) -> bool:
        # 验证参数
        return True

    def setup(self):
        # 初始化资源
        pass

    async def run(self) -> TaskResult:
        # 执行任务逻辑
        for i in range(100):
            await self.async_check_paused()  # 检查暂停/停止
            self.update_progress(current_step=i, total_steps=100, message=f"处理中 {i}%")
            await asyncio.sleep(0.1)

        return TaskResult(success=True, message="任务完成", data={})

    def teardown(self):
        # 清理资源
        pass
```

2. 在 `implementations/__init__.py` 中导入：

```python
from app.services.tasks.implementations.my_task import MyCustomTask
```

---

## 📄 License

MIT License

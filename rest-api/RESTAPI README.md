# LibRS REST API Server (RealSense Web Wrapper)

这是一个基于 **FastAPI** 和 **pyrealsense2** (Intel RealSense SDK) 构建的 REST API 服务。它提供了一套标准化的 HTTP 接口来控制 RealSense 设备，并支持通过 **WebRTC** 进行低延迟的实时视频流传输（RGB 和 深度图），以及通过 **Socket.IO** 推送元数据和点云数据。

本项目旨在简化 RealSense 设备的远程调用与网络集成。

## 🌟 核心功能

* **RESTful API 设备管理**:
  * 枚举已连接的 RealSense 设备。
  * 查询设备详情（序列号、固件版本、传感器信息）。
  * 获取和设置传感器选项（如曝光、增益、激光功率等）。
  * 控制流的开启与停止（支持自定义分辨率、格式、帧率）。
* **WebRTC 实时预览**:
  * 基于 `aiortc` 实现。
  * 支持 RGB 和 Depth 视频流的实时传输到浏览器。
  * 包含完整的信令交换接口 (`/offer`, `/answer`, `/ice-candidates`)。
* **Socket.IO 数据推送**:
  * 实时推送帧元数据（时间戳、帧号）。
  * 支持点云数据（PointCloud）和运动数据（Motion Data）的传输。

## 🏗️ 技术架构

* **Web 框架**: FastAPI (高性能异步框架)
* **硬件交互**: pyrealsense2 (官方 Python 包装器)
* **实时流**: aiortc (Python WebRTC 实现)
* **实时通信**: python-socketio
* **并发模型**: 异步非阻塞 (AsyncIO)，但在调用阻塞的 RealSense SDK 时使用线程/执行器。

## 🚀 快速开始

### 1. 环境要求

* **操作系统**: Linux (推荐 Ubuntu 20.04/22.04), Windows (理论支持)
* **Python**: 3.8+
* **硬件**: Intel RealSense D400 系列摄像头 (如 D415, D435, D455)

### 2. 安装依赖

```bash
# 1. 创建虚拟环境 (推荐)
python3 -m venv venv
source venv/bin/activate

# 2. 安装 Python 依赖
pip install -r requirements.txt
```

> **注意**: 确保系统已安装 `librealsense2` 运行库。通常 `pip install pyrealsense2` 包含的二进制文件已足够，但在某些 ARM 架构（如 Jetson）上可能需要通过 apt 安装或源码编译。

### 3. 配置 (可选)

项目使用 `config.py` 管理配置。建议通过环境变量覆盖默认值（特别是生产环境）：

* `SECRET_KEY`: JWT 签名密钥 (默认: 随机/开发用)
* `CORS_ORIGINS`: 允许跨域的源 (默认: `["*"]`，生产环境建议限制)

```bash
export SECRET_KEY="your-super-secret-key-change-it"
```

### 4. 启动服务

```bash
# 赋予脚本执行权限
chmod +x start_server.sh

# 启动 (使用 uvicorn)
./start_server.sh
```

服务默认运行在 `http://0.0.0.0:8000`。

## 📖 使用指南

### API 文档 (Swagger UI)

启动服务后，访问 **[http://localhost:8000/docs](http://localhost:8000/docs)** 可查看完整的交互式 API 文档。

### Web 预览器 (Viewer)

项目包含一个简单的 HTML 客户端 `viewer.html` 用于测试 WebRTC 功能。

1. **修改 Device ID**: 打开 `viewer.html`，找到 `const DEVICE_ID = '...'`，将其修改为你当前连接的 RealSense 设备 ID（可通过 API `/api/devices` 获取）。
    * *注：当前版本已更新为测试设备 ID `029522250337`。*
2. **打开页面**: 在浏览器中直接打开该文件 (例如 `file:///path/to/viewer.html`)。
3. **连接**: 点击 "连接视频流" 按钮。如果一切正常，你将看到实时的 RGB 和深度图像。

## 🧪 测试

项目包含基于 `pytest` 的测试套件，涵盖 Mock 测试和集成测试。

```bash
# 安装测试依赖
pip install pytest pytest-asyncio httpx lark

# 运行所有测试
pytest tests/test_api_service.py

# 运行针对真实设备的 WebRTC 集成测试 (需要连接设备)
pytest tests/test_api_service.py -k test_webrtc_offer_rs -v
```

## ⚠️ 注意事项与已知限制

1. **单设备流限制**: RealSense SDK 对多进程访问同一设备有限制。本服务作为单例管理设备，但在高并发请求下（如频繁开关流）可能会有资源竞争，建议串行操作流控制。
2. **CORS**: 默认为允许所有来源 (`*`)，这方便了本地开发（如直接打开 `viewer.html`），但在公网部署时请务必修改 `config.py` 或环境变量以限制来源。
3. **安全性**: 目前 API 未强制启用身份验证（虽然代码中有 JWT 框架），在开放网络中使用前请务必完善 `security.py` 中的认证逻辑。

## 目录结构

* `app/`: 核心应用代码
  * `api/`: 路由定义 (endpoints) 和依赖注入
  * `core/`: 全局配置、错误处理、安全工具
  * `models/`: Pydantic 数据模型
  * `services/`: 业务逻辑 (RealSenseManager, WebRTCManager)
* `tests/`: 测试用例
* `main.py`: 程序入口
* `viewer.html`: 测试用客户端

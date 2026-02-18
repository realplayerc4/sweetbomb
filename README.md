# RealSense Web Monitor

基于 **FastAPI** 和 **React** 构建的现代化 RealSense 监控与控制平台。支持通过 RESTful API 管理设备，利用 **WebRTC** 进行低延迟的实时视频流传输（RGB 和深度图），并通过 **Socket.IO** 实时推送元数据和点云数据。

A modern RealSense monitoring and control platform built with **FastAPI** and **React**. It supports device management via RESTful API, low-latency real-time video streaming (RGB and Depth) using **WebRTC**, and real-time metadata/point cloud streaming via **Socket.IO**.

## 🌟 核心功能 (Features)

- **设备管理**: 实时发现与枚举连接的 RealSense 设备
- **参数配置**: 在线调整传感器参数（曝光、增益、激光功率等）
- **实时预览**:
  - **低延迟**: 基于 WebRTC 的毫秒级视频传输
  - **深度伪彩**: 支持 Jet 伪彩色映射与直方图均衡化，深度细节清晰可见
  - **点云可视化**: 集成 Three.js 的 3D 点云视图
- **数据流**:
  - RGB / Depth 双流同步
  - Socket.IO 实时元数据推送

## 🏗️ 技术架构 (Architecture)

- **Backend**: Python FastAPI + pyrealsense2 + aiortc
- **Frontend**: React + TypeScript + Vite + TailwindCSS + Three.js

## 🚀 快速开始 (Quick Start)

### 1. 环境要求 (Prerequisites)

- **OS**: Linux (Ubuntu 20.04/22.04 推荐)
- **Hardware**: Intel RealSense D400 Series (D415, D435, D455)
- **Runtime**: Python 3.8+, Node.js 16+

### 2. 后端启动 (Backend Setup)

```bash
cd rest-api

# 创建并激活虚拟环境
python3 -m venv venv
source venv/bin/activate

# 安装依赖
pip install -r requirements.txt

# 启动服务
chmod +x start_server.sh
./start_server.sh
```

后端服务将运行在 `http://localhost:8000`。
- API 文档: `http://localhost:8000/docs`

### 3. 前端启动 (Frontend Setup)

```bash
cd rest-api/ui/frontend

# 安装依赖
npm install

# 启动开发服务器
npm run dev
```

前端页面将运行在 `http://localhost:5173`。

## 📖 使用指南 (Usage)

1. 启动服务: 确保后端 (`./start_server.sh`) 和前端 (`npm run dev`) 均已启动
2. 访问界面: 打开浏览器访问 `http://localhost:5173`
3. 连接设备: 界面会自动发现连接的 RealSense 设备
4. 开启视频流: 点击右上角的 **"启动"** 按钮，即可看到实时的 RGB 和深度视频流
5. 查看点云: 页面下方提供了基于 WebGL 的点云视图，支持鼠标拖拽旋转查看

## ⚠️ 注意事项 (Notes)

- **硬件兼容性**: 请确保系统已安装 `librealsense2` 运行库
- **USB 带宽**: 同时开启高分辨率 RGB 和深度流需要 USB 3.0+ 接口，请确保使用原装或高质量数据线
- **多设备**: 目前前端界面主要针对单设备优化，后端支持多设备枚举

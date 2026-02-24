# restrealsenseMonitor

> RealSense Web 监控与控制平台 - 快速入门指南

基于 **FastAPI** 和 **React** 构建的现代化 RealSense 监控平台，支持低延迟视频流、点云可视化和 AI 任务管理。

---

## 文档导航

| 文档 | 角色 | 用途 |
|------|------|------|
| [**spec.md**](./spec.md) | 立法 | API 规范、架构规范、组件规范 |
| [**knowledge.md**](./knowledge.md) | 参考 | 技术知识、配置参考、问题解决 |
| [**changelog.md**](./changelog.md) | 记录 | 版本历史、变更记录 |
| **README.md** | 入门 | 快速入门、安装部署 |

---

## 环境要求

| 组件 | 要求 |
|------|------|
| OS | Ubuntu 20.04/22.04 |
| Python | 3.8+ |
| Node.js | 16+ |
| Hardware | RealSense D400 系列 |
| USB | 3.0+ (高分辨率流) |

---

## 快速开始

### 1. 后端启动

```bash
cd rest-api

# 创建虚拟环境
python3 -m venv venv
source venv/bin/activate

# 安装依赖
pip install -r requirements.txt

# 启动服务
./start_server.sh
```

后端运行在 `http://localhost:8000`
- API 文档: http://localhost:8000/docs

### 2. 前端启动

```bash
cd rest-api/ui/frontend

# 安装依赖
npm install

# 启动开发服务器
npm run dev
```

前端运行在 `http://localhost:5173`

---

## 核心功能

| 功能 | 技术 | 说明 |
|------|------|------|
| 设备管理 | REST API | 发现、配置 RealSense 设备 |
| 视频流 | WebRTC | RGB/Depth 低延迟传输 |
| 点云 | Three.js | 3D 实时可视化 |
| 任务系统 | Socket.IO | AI 模型并行运行 |

---

## 目录结构

```
rest-api/
├── app/                    # 后端
│   ├── api/endpoints/      # API 端点
│   ├── models/             # 数据模型
│   └── services/           # 核心服务
├── ui/frontend/            # 前端
│   └── src/app/
│       ├── components/     # UI 组件
│       ├── hooks/          # React Hooks
│       └── services/       # API 封装
├── spec.md                 # 规范文档
├── knowledge.md            # 知识库
└── changelog.md            # 变更日志
```

---

## 常用命令

```bash
# 查看设备
curl http://localhost:8000/api/devices

# 启动视频流
curl -X POST http://localhost:8000/api/devices/{device_id}/stream/start

# 获取任务类型
curl http://localhost:8000/api/tasks/types

# 创建任务
curl -X POST http://localhost:8000/api/tasks/ \
  -H "Content-Type: application/json" \
  -d '{"task_type": "object_detection", "device_id": "..."}'
```

---

## 常见问题

| 问题 | 解决方案 |
|------|----------|
| 设备无法发现 | `lsusb \| grep Intel` 检查连接 |
| 视频流卡顿 | 确认 USB 3.0 接口 |
| 深度图噪点多 | 启用空间/时间滤波器 |

详细问题排查见 [knowledge.md](./knowledge.md)

---

## License

MIT

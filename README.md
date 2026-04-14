# Sweetbomb - 智能铲糖机器人平台

基于 **FastAPI** 和 **React** 构建的现代化 RealSense 监控与机器人控制平台。支持通过 RESTful API 管理设备，利用 **WebRTC** 进行低延迟的实时视频流传输（RGB 和深度图），并通过 **Socket.IO** 实时推送元数据和点云数据。集成行为树引擎实现机器人自主铲糖任务，支持履带运动控制、伺服操作和智能距离分析。

---

## 🚀 Jetson 部署指南

### 环境要求
- **硬件**: Jetson Orin Nano / Orin NX / Xavier NX
- **系统**: JetPack 6.x (Ubuntu 22.04)
- **Python**: 3.10+
- **Node.js**: 20.x (推荐) 或 18.x

### 快速部署步骤

#### 1. 系统准备
```bash
# 更新系统
sudo apt update
sudo apt install -y python3.10 python3.10-venv python3.10-dev git curl

# 安装 Node.js 20
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
sudo apt install -y nodejs
```

#### 2. 克隆项目
```bash
cd ~
git clone <repository-url> sweetbomb
cd sweetbomb
```

#### 3. 后端配置
```bash
# 创建虚拟环境
python3.10 -m venv venv
source venv/bin/activate

# 安装依赖
pip install fastapi uvicorn pydantic python-socketio aiortc psutil

# 链接系统 pyrealsense2
echo "/usr/lib/python3/dist-packages" > venv/lib/python3.10/site-packages/pyrealsense2.pth

# 创建日志目录
mkdir -p logs
```

#### 4. 前端配置
```bash
cd ui/frontend

# 安装依赖
npm install

# 确保 vite.config.ts 配置正确 (已配置 host: '0.0.0.0')

# 编译前端
npm run build

cd ../..
```

#### 5. 启动服务

```bash
# 方式一：使用启动脚本（开发调试）
./start-simple.sh

# 方式二：安装开机自启动（生产环境）
sudo ./install-service.sh
```

**常用服务命令：**
```bash
systemctl start sweetbomb-backend sweetbomb-frontend   # 启动
systemctl stop sweetbomb-backend sweetbomb-frontend    # 停止
systemctl restart sweetbomb-backend sweetbomb-frontend # 重启
systemctl status sweetbomb-backend sweetbomb-frontend  # 状态
journalctl -u sweetbomb-backend -u sweetbomb-frontend -f  # 日志
```

### 常见问题

#### 1. Node.js 版本不兼容
**现象**: `Vite requires Node.js version 20.19+ or 22.12+`
**解决**: 升级到 Node.js 20.x
```bash
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
sudo apt install -y nodejs
```

#### 2. pyrealsense2 导入失败
**现象**: `ModuleNotFoundError: No module named 'pyrealsense2'`
**解决**: 创建 .pth 文件链接系统 pyrealsense2
```bash
echo "/usr/lib/python3/dist-packages" > venv/lib/python3.10/site-packages/pyrealsense2.pth
```

#### 3. 前端无法访问 (跨设备)
**现象**: 其他设备无法访问 `http://<jetson-ip>:5173`
**解决**: 修改 `ui/frontend/vite.config.ts`，添加 `server.host: '0.0.0.0'`

#### 4. RealSense 设备权限错误
**现象**: `RuntimeError: No device connected` 或权限错误
**解决**: 安装 udev 规则并重新插拔设备
```bash
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 路径修改总结 (从 x86 迁移到 Jetson)

主要修改了以下文件中的路径 `/home/yq` → `/home/jetson`:

1. `app/core/logging_config.py` - 日志目录路径
2. `start-simple.sh` - 启动脚本工作目录
3. `deploy/setup_systemd.py` - systemd 服务配置
4. `deploy/clean_remote_deploy.py` - 部署脚本
5. `.claude/settings.json` - Claude 配置

---

## 🌟 核心功能 (Features)

### 设备与视频流
- **设备管理**: 实时发现与枚举连接的 RealSense 设备
- **参数配置**: 在线调整传感器参数（曝光、增益、激光功率等）
- **实时预览**:
  - **低延迟**: 基于 WebRTC 的毫秒级视频传输
  - **深度伪彩**: 支持 Jet 伪彩色映射与直方图均衡化，深度细节清晰可见
  - **点云可视化**: 集成 Three.js 的 3D 点云视图，支持 BEV 俯视图切片
  - **数据流**:
    - RGB / Depth 双流同步
    - Socket.IO 实时元数据推送

### 机器人控制
- **运动控制**: 履带式机器人八方向运动（前进、后退、左转、右转、原地旋转、停止）
- **伺服控制**: 独立伺服电机角度控制，支持铲取和倾倒动作
- **状态监控**: 实时电池电量、位置、姿态、履带速度反馈
- **紧急停止**: 一键紧急停止与状态重置

### 智能任务
- **行为树引擎**: 灵活的 BT（Behavior Tree）系统，支持顺序节点、选择节点、条件节点、动作节点
- **自主铲糖**: 完整的自动铲糖循环任务，包括导航、距离分析、铲取、倾倒、返回
- **距离分析**: 基于点云的智能距离测算，糖堆高度检测，自动判断铲糖策略
- **任务管理**: 可扩展的后台任务系统，支持任务创建、启动、暂停、恢复、停止

### 航点与导航
- **航点管理**: 持久化航点存储，支持添加、删除、查询航点
- **导航接口**: 标准化导航服务接口，支持目标点导航与状态追踪

### 地图管理
- **地图转换**: 自动将 txt 栅格地图转换为 PNG/SVG 图片
- **地图API**: RESTful API 提供地图列表、地图图片、地图信息查询
- **前端组件**: React 组件支持地图显示、缩放、参数调节
- **颜色主题**: 石墨橙色 (#c25e1f) 点图配色，深色背景
- **缓存机制**: 自动缓存转换后的地图图片，支持强制刷新

**API 端点**:
- `GET /api/map/` - 获取地图列表
- `GET /api/map/{name}.png` - 获取地图 PNG 图片（支持 dpi、point_size 参数）
- `GET /api/map/{name}/info` - 获取地图详细信息
- `POST /api/map/cache/clear` - 清理地图缓存

---

## 🏗️ 技术架构 (Architecture)

### 后端技术栈
- **Web 框架**: Python FastAPI + Uvicorn
- **硬件驱动**: pyrealsense2 (RealSense D400 系列)
- **视频流**: (WebRTC)
- **实时通信**: python-socketio
- **数据验证**: Pydantic 2.0+
- **任务调度**: asyncio + 自定义任务管理器

### 前端技术栈
- **UI 框架**: React 18 + TypeScript 5.0+
- **构建工具**: Vite 5.0+
- **样式系统**: TailwindCSS 3.4+
- **3D 渲染**: Three.js 0.160+
- **实时通信**: Socket.IO Client 4.7+

### 硬件支持
- **深度相机**: Intel RealSense D400 系列 (D415/D435/D455)
- **接口**: USB 3.0+
- **机器人**: 履带式移动平台 + 伺服机械臂

---

## 🚀 快速开始 (Quick Start)

### 1. 环境要求 (Prerequisites)
- **OS**: Linux (Ubuntu 20.04/22.04 推荐)
- **Hardware**: Intel RealSense D400 Series (D415, D435, D455)
- **Runtime**: Python 3.10+, Node.js 18+
- **Process Manager**: PM2 (推荐)

### 2. 后端启动 (Backend Setup)
```bash
# 创建并激活虚拟环境
python3 -m venv venv
source venv/bin/activate

# 安装依赖
pip install -r requirements.txt

# 启动服务
uvicorn main:combined_app --host 0.0.0.0 --port 8000
```
后端服务将运行在 `http://localhost:8000`。
API 文档: `http://localhost:8000/docs`

### 3. 前端启动 (Frontend Setup)
```bash
cd ui/frontend

# 安装依赖
npm install

# 启动开发服务器
npm run dev
```
前端页面将运行在 `http://localhost:5173`。

### 4. 生产环境部署 (Production Deployment)

安装 systemd 服务实现开机自启动：
```bash
sudo ./install-service.sh
```

**服务管理命令：**
```bash
systemctl start sweetbomb-backend sweetbomb-frontend   # 启动
systemctl stop sweetbomb-backend sweetbomb-frontend    # 停止
systemctl restart sweetbomb-backend sweetbomb-frontend # 重启
systemctl status sweetbomb-backend sweetbomb-frontend  # 状态
journalctl -u sweetbomb-backend -u sweetbomb-frontend -f  # 日志
```

---

## 📖 使用指南 (Usage)

### 基础监控
1. 启动服务: 确保后端和前端均已启动
2. 访问界面: 打开浏览器访问 `http://localhost:5173`
3. 连接设备: 界面会自动发现连接的 RealSense 设备
4. 开启视频流: 点击右上角的 **"启动"** 按钮，即可看到实时的 RGB 和深度视频流
5. 查看点云: 3D 点云视图支持鼠标拖拽旋转查看，BEV 俯视图展示前向切片

### 机器人控制
1. **手动控制**: 使用方向控制面板或键盘 WASD 控制机器人移动
2. **伺服操作**: 调整伺服角度执行铲取/倾倒动作，或使用预设快捷按钮
3. **紧急停止**: 遇到紧急情况点击红色急停按钮

### 自动铲糖任务
1. **配置航点**: 在地图面板设置导航点（糖堆）和倾倒点
2. **设置参数**: 配置铲斗宽度、接近距离、循环次数等参数
3. **启动任务**: 点击"启动自动铲糖"开始自主作业
4. **监控状态**: 实时查看任务进度、当前循环、糖堆高度等信息

### 任务管理
1. **创建任务**: 从任务类型列表选择任务类型，填写参数创建
2. **控制任务**: 支持启动、暂停、恢复、停止操作
3. **查看进度**: 实时进度条、步骤信息、预估剩余时间

---

## 📁 项目结构 (Project Structure)

```
sweetbomb/
├── app/                          # 后端核心业务服务
│   ├── routers/                  # API 路由层
│   │   ├── devices.py           # 设备管理 API
│   │   ├── streams.py            # WebRTC 信令 API
│   │   ├── robot.py             # 机器人控制 API
│   │   ├── tasks.py             # 任务管理 API
│   │   ├── point_cloud.py          # 点云数据 API
│   │   ├── waypoints.py         # 航点管理 API
│   └── router.py               # 主路由
│   ├── services/                 # 业务逻辑层
│   │   ├── rs_manager.py        # RealSense 设备管理器
│   │   ├── webrtc_manager.py    # WebRTC 流管理器
│   │   ├── task_manager.py      # 任务任务调度管理器
│   │   ├── robot_controller.py  # 机器人控制器
│   │   ├── behavior_tree_engine.py  # 行为树引擎
│   │   ├── distance_analyzer.py # 距离分析服务
│   │   └── waypoint_manager.py  # 航点管理服务
│   └── models/                   # 数据模型
│       ├── robot.py              # 机器人状态模型
│       ├── task.py               # 任务模型
│       ├── waypoint.py           # 航点模型
│       ├── sensor.py              # 传感器数据模型
│       ├── device.py              # 设备模型
│       ├── stream.py              # 视频流模型
│       ├── webrtc.py             # WebRTC 模型
│       └── option.py              # 选项配置模型
├── ui/                           # 前端 React 源码
│   └── frontend/
│       └── src/app/
│           ├── components/       # React 组件
│           │   ├── Dashboard.tsx        # 主仪表盘 (3x2 网格布局)
│           │   ├── RobotControlPanel.tsx # 机器人控制面板
│           │   ├── TaskPanel.tsx         # 任务管理面板
│           │   ├── PointCloud.tsx        # 3D 点云视图
│           │   ├── VideoView.tsx         # RGB/Depth 视频视图
│           │   ├── SliceView.tsx         # BEV 俯视切片
│           │   ├── MapPanel.tsx          # 航点地图面板
│           │   └── SugarHarvestPanel.tsx # 铲糖任务面板
│           ├── services/         # API 客户端
│           ├── hooks/            # 自定义 Hooks
│           └── App.tsx              # 应用入口
├── tests/                        # 单元与集成测试
├── deploy/                       # 运维与部署脚本
├── memory-bank/                  # AI 上下文核心
├── docs/                         # 项目文档
├── ecosystem.config.cjs          # PM2 配置文件
└── README.md                     # 项目说明
```

---

## ⚠️ 注意事项 (Notes)

- **硬件兼容性**: 请确保系统已安装 `librealsense2` 运行库
- **USB 带宽**: 同时开启高分辨率 RGB 和深度流需要 USB 3.0+ 接口，请确保使用原装或高质量数据线
- **坐标系**: 项目采用 Z-up 右手坐标系 (X=右侧, Y=正前, Z=高度)
- **点云范围**: 默认探测范围为 6m，可在空间配置面板调整
- **任务并发**: 后台任务最多支持 4 个并发执行

---

**版本**: 1.3.0
**最后更新**: 2026-04-14

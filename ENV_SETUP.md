# Jetson Orin Nano 环境配置记录

## 系统信息
- **设备**: Jetson Orin Nano
- **JetPack**: 6.2.1
- **Python**: 3.10.12 (系统自带)
- **架构**: ARM64 (aarch64)

## 虚拟环境配置

### 创建虚拟环境
```bash
python3.10 -m venv venv --system-site-packages
```

**关键**: 使用 `--system-site-packages` 来包含系统包，这是为了使用系统编译的 pyrealsense2。

### pyrealsense2 配置
JetPack 6.2.1 系统自带的 pyrealsense2 位于 `/usr/lib/python3/dist-packages/pyrealsense2`

通过 `--system-site-packages` 自动链接，无需额外配置。

## 依赖安装

### 核心依赖
```bash
source venv/bin/activate
pip install fastapi uvicorn pydantic opencv-python numpy psutil python-socketio -i https://pypi.tuna.tsinghua.edu.cn/simple
```

### WebRTC 依赖
```bash
pip install aiortc av websockets -i https://pypi.tuna.tsinghua.edu.cn/simple
```

**注意**: 在 ARM64 上使用清华镜像源，避免编译问题。

## 已安装包版本

| 包 | 版本 |
|------|------|
| fastapi | 0.135.1 |
| uvicorn | 0.42.0 |
| pydantic | 2.12.5 |
| numpy | 2.2.6 |
| opencv-python | 4.13.0.92 |
| aiortc | 1.14.0 |
| av | 16.1.0 |
| pyrealsense2 | 系统版本 |

## 系统 pyrealsense2 验证

```python
import pyrealsense2 as rs
ctx = rs.context()
print(len(ctx.query_devices()))  # 输出: 1
print(ctx.query_devices()[0].get_info(rs.camera_info.name))  # 输出: Intel RealSense D455
```

## 启动服务

```bash
./start-simple.sh
```

服务启动后：
- 前端: http://localhost:5173
- 后端: http://localhost:8000
- API文档: http://localhost:8000/docs

## 常见问题

### numpy 版本冲突
系统有 numpy 1.21.5，虚拟环境安装了 numpy 2.2.6。由于使用 `--system-site-packages`，系统 numpy 也会被加载，但虚拟环境的 numpy 2.2.6 会优先使用。

### 点云不显示
如果场景空旷，点云数据为空是正常的。确保有物体在摄像头前方 1-6 米范围内。

### WebRTC 连接问题
在 Jetson 上使用 aiortc 时，STUN/TURN 服务器可能需要配置。内网使用时可以直接连接。

## 文件结构

```
/home/jetson/sweetbomb/
├── venv/                      # Python 3.10 虚拟环境
│   ├── bin/python -> /usr/bin/python3.10
│   └── lib/python3.10/site-packages/
├── app/                       # 后端代码
│   ├── api/endpoints/
│   ├── services/
│   └── models/
├── ui/frontend/               # 前端代码
├── main.py                    # FastAPI 入口
└── start-simple.sh            # 启动脚本
```

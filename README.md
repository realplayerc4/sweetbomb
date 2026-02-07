# 🗺️ RealSense D455 + OctoMap 实时 Web 3D 可视化系统

高性能、交互式的 ROS 2 实时 3D 观测系统。基于 RealSense D455 点云生成 OctoMap 八叉树地图，并提供流畅的 Web 浏览器 3D 可视化界面。

> **核心技术栈**: ROS 2 Humble + RealSense SDK + OctoMap + Three.js + Rosbridge
> **性能指标**: 5-10 Hz 实时更新，支持体素渲染与黑色边框优化。

---

## 🚀 快速开始

### 1. 构建项目

如果是第一次运行，请先构建工作空间：

```bash
cd /home/yq/ros2realsense
colcon build --symlink-install
```

### 2. 一键启动

通过以下指令同时启动摄像头、地图生成服务器、Web 服务器和数据网关：

```bash
source install/setup.bash
ros2 launch realsense2_camera rs_octomap.launch.py
```

### 3. 打开 Web 可视化

在浏览器中访问：**[http://localhost:8888](http://localhost:8888)**

---

## 🌐 Web 可视化功能介绍

本项目提供了一个定制化的 Web 控制台，专门针对机器人观测场景优化：

- **坐标系对齐 (Z-up)**：与 ROS 标准完全一致，相机默认从后上方俯视前方（X 轴正方向）。
- **实时 3D 渲染**：橙色体素配合黑色边缘线，即使在大量体素下也能清晰辨识深度。
- **地面高度调节**：可在 UI 输入框实时调整“地面高度 (Z)”，方便对齐物理地面或滤除干扰。
- **单键重置视角**：点击“重置视角”按钮可立即将相机恢复至最佳观察位置。
- **本地化离线支持**：所有核心库均已集成在本地，无需联网即可使用。

---

## 🛠️ 参数配置

您可以在启动时通过命令行参数自定义系统行为：

| 参数名 | 默认值 | 描述 |
|--------|--------|------|
| `octomap_resolution` | `0.2` | 体素分辨率（米）。精度越高，计算开销越大。 |
| `enable_rviz` | `false` | 是否同时启动 RViz 2 可视化。 |
| `enable_web_viewer` | `true` | 是否启动 Web 可视化组件 (rosbridge + HTTP)。 |
| `clip_distance` | `3.0` | 深度裁剪距离（米）。超过此距离的点将被忽略。 |

**示例：高精度模式启动**

```bash
ros2 launch realsense2_camera rs_octomap.launch.py octomap_resolution:=0.05 enable_rviz:=true
```

---

## 🔄 地图管理

如果需要清空当前已建立的地图（例如物体已移除但仍有残留位姿）：

```bash
ros2 service call /octomap_server/reset std_srvs/srv/Empty
```

---

## 📊 性能优化点

- ✅ **Decimation 降采样**: 输入点云仅保留 1/16，极大提升实时性。
- ✅ **动态观测模式**: 配置为物体移开后在延迟极低的情况下（<1s）自动消失。
- ✅ **渲染优化**: Web 端采用 `InstancedMesh` 和 `MergedBufferGeometry` 渲染体素，确保流畅运行。
- ✅ **硬件加速**: 充分利用显卡进行 GL 加速（建议安装 NVIDIA 590+ 驱动）。

---

## ⚠️ 环境依赖

- **显卡驱动**: NVIDIA 驱动版本建议 590 以上。
- **浏览器**: 建议使用 Chrome 或 Firefox，并开启“硬件加速”。
- **ROS 2**: Humble Hawksbill (Ubuntu 22.04)。

---
*维护者: Antigravity AI*

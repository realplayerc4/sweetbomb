# RealSense D455 OctoMap Web 可视化

基于 ROS 2 Humble 和 RealSense D455 的实时点云体素化与 Web 3D 可视化系统。

## ⚠️ 核心特性 (v2 优化版)

- **高性能体素化**: 采用 `numpy` 向量化计算，点云处理速度提升 10 倍以上。
- **实时 Web 渲染**: 基于 Three.js + Rosbridge，支持 10Hz+ 实时刷新。
- **标准坐标系**: 强制 Z-up 坐标系，符合 ROS 标准。

## 🚀 快速启动

### 1. 编译

```bash
cd ~/ros2realsense
colcon build --symlink-install
```

### 2. 运行

```bash
source install/setup.bash
ros2 launch octomap_web_viewer rs_voxelizer.launch.py
```

### 3. 访问

浏览器打开: [http://localhost:8888](http://localhost:8888)

## 🛠 配置参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `octomap_resolution` | `0.15` | 体素分辨率 (米) |
| `point_skip` | `4` | 点云下采样倍率 (每 N 个点取 1 个) |
| `publish_rate` | `10.0` | 最大发布频率 (Hz) |

## 🧪 开发与测试

运行单元测试以验证体素化逻辑：

```bash
python3 src/octomap_web_viewer/tests/test_voxelizer.py
```

*维护者: Antigravity AI*

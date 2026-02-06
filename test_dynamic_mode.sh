#!/bin/bash
# 动态观测模式测试脚本
# 用于验证物体移走后能否快速消失（<1秒）

echo "🧪 动态观测模式测试"
echo "================================"
echo ""
echo "📋 当前配置："
echo "  - hit: 0.97 (检测到立即建立)"
echo "  - miss: 0.01 (未检测时每帧概率降低99%)"
echo "  - min: 0.49 (低于0.5后被prune删除)"
echo "  - compress_map: true (启用prune删除体素)"
echo ""
echo "⚡ 预期效果："
echo "  物体移走后 3-5 帧（约 0.3-1 秒）消失"
echo ""
echo "🧪 测试步骤："
echo "  1. 启动系统（自动进行）"
echo "  2. 在相机前放置物体 → 观察 1-2 秒内出现"
echo "  3. 移走物体 → 观察 <1 秒内消失"
echo "  4. 重复 2-3 步确认一致性"
echo ""
echo "📊 监控命令（另开终端）："
echo "  ros2 topic hz /camera/depth/color/points    # 点云频率"
echo "  ros2 topic hz /occupied_cells_vis_array     # OctoMap更新频率"
echo ""
read -p "按 Enter 启动系统..." dummy

# 检查环境
if [ -z "$ROS_DISTRO" ]; then
    echo "⚠️  设置 ROS2 环境..."
    source /opt/ros/humble/setup.bash
    source install/setup.bash
fi

# 启动
echo ""
echo "🚀 启动 RealSense + OctoMap (动态观测模式)..."
ros2 launch realsense2_camera rs_octomap.launch.py clip_distance:=3.0

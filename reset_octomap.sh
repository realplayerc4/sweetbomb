#!/bin/bash
# OctoMap 地图重置脚本
# 用途：清空累积的 3D 地图，从头开始建图

echo "🔄 重置 OctoMap 地图..."

# 检查 ROS2 环境
if [ -z "$ROS_DISTRO" ]; then
    echo "⚠️  请先 source ROS2 环境："
    echo "   source /opt/ros/humble/setup.bash"
    echo "   source install/setup.bash"
    exit 1
fi

# 调用 reset 服务
ros2 service call /octomap_server/reset std_srvs/srv/Empty

if [ $? -eq 0 ]; then
    echo "✅ 地图已重置！"
    echo ""
    echo "💡 提示："
    echo "   - 地图会从当前时刻重新开始累积"
    echo "   - 如果需要定期自动重置，可以添加 cron 任务"
else
    echo "❌ 重置失败！请检查："
    echo "   1. OctoMap Server 是否正在运行"
    echo "   2. 服务名称是否正确（/octomap_server/reset）"
fi

#!/bin/bash
# RealSense D455 到 OctoMap 快速启动脚本

# 设置颜色输出
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}RealSense D455 OctoMap 启动脚本${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# 检查是否在正确的目录
WORKSPACE_DIR="/home/yq/ros2realsense"
if [ "$PWD" != "$WORKSPACE_DIR" ]; then
    echo -e "${YELLOW}切换到工作空间目录: $WORKSPACE_DIR${NC}"
    cd "$WORKSPACE_DIR"
fi

# Source ROS2 环境
echo -e "${YELLOW}加载 ROS2 环境...${NC}"
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo -e "${GREEN}✓ ROS2 Humble 环境已加载${NC}"
else
    echo -e "${RED}✗ 错误: 找不到 ROS2 Humble 安装${NC}"
    exit 1
fi

# Source 工作空间
echo -e "${YELLOW}加载工作空间环境...${NC}"
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo -e "${GREEN}✓ 工作空间环境已加载${NC}"
else
    echo -e "${RED}✗ 错误: 工作空间未构建，请先运行 'colcon build'${NC}"
    exit 1
fi

# 检查 RealSense 相机
echo ""
echo -e "${YELLOW}检查 RealSense 相机连接...${NC}"
if command -v rs-enumerate-devices &> /dev/null; then
    CAMERA_INFO=$(rs-enumerate-devices 2>/dev/null | grep -i "Intel RealSense" | head -n 1)
    if [ -n "$CAMERA_INFO" ]; then
        echo -e "${GREEN}✓ 检测到相机: $CAMERA_INFO${NC}"
    else
        echo -e "${YELLOW}⚠ 警告: 未检测到 RealSense 相机${NC}"
        echo -e "${YELLOW}  请确保相机已连接到 USB 3.0 端口${NC}"
        read -p "是否继续启动? (y/N): " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
else
    echo -e "${YELLOW}⚠ 警告: rs-enumerate-devices 命令不可用${NC}"
    echo -e "${YELLOW}  无法检查相机状态，继续启动...${NC}"
fi

# 询问点云主题(可选)
echo ""
echo -e "${YELLOW}点云主题设置:${NC}"
read -p "点云主题 [默认: /camera/camera/depth/color/points]: " POINTCLOUD_TOPIC
POINTCLOUD_TOPIC=${POINTCLOUD_TOPIC:-/camera/camera/depth/color/points}

# 询问用户是否要自定义参数
echo ""
echo -e "${YELLOW}启动选项:${NC}"
echo "1) 快速启动 (默认参数)"
echo "2) 自定义参数启动"
echo "3) 超高精度模式 (2cm 体素)"
echo "4) 高精度模式 (5cm 体素, 降采样)"
echo "5) 平衡模式 (10cm 体素)"
read -p "请选择 (1-5) [默认: 1]: " CHOICE
CHOICE=${CHOICE:-1}

# 询问是否需要高度过滤
echo ""
echo -e "${YELLOW}点云距离和高度设置:${NC}"
read -p "RealSense 最大距离裁剪 (米) [默认: 3.0, -1=禁用]: " CLIP_DISTANCE
CLIP_DISTANCE=${CLIP_DISTANCE:-3.0}

read -p "点云主题 [默认: /camera/depth/color/points]: " POINTCLOUD_TOPIC
POINTCLOUD_TOPIC=${POINTCLOUD_TOPIC:-/camera/depth/color/points}

read -p "地面高度 (米) [默认: 0.0]: " GROUND_HEIGHT
GROUND_HEIGHT=${GROUND_HEIGHT:-0.0}

read -p "距离地面的最大显示高度 (米) [默认: 2.0]: " MAX_HEIGHT_ABOVE_GROUND
MAX_HEIGHT_ABOVE_GROUND=${MAX_HEIGHT_ABOVE_GROUND:-2.0}

# 计算实际的最大Z值
MAX_Z=$(echo "$GROUND_HEIGHT + $MAX_HEIGHT_ABOVE_GROUND" | bc)

echo -e "${GREEN}RealSense 最大距离: $CLIP_DISTANCE m, 地面高度: $GROUND_HEIGHT m, 最大显示高度: $MAX_HEIGHT_ABOVE_GROUND m${NC}"
echo -e "${GREEN}计算得到的最大Z值: $MAX_Z m${NC}"

# 根据选择设置参数
case $CHOICE in
    1)
        echo -e "${GREEN}使用默认参数启动...${NC}"
        PARAMS="clip_distance:=$CLIP_DISTANCE pointcloud_topic:=$POINTCLOUD_TOPIC pointcloud_max_z:=$MAX_Z occupancy_max_z:=$MAX_Z"
        ;;
    2)
        echo ""
        read -p "OctoMap 分辨率 (米) [默认: 0.05]: " RESOLUTION
        RESOLUTION=${RESOLUTION:-0.05}
        
        read -p "抽取滤波器强度 (1-8) [默认: 6]: " DECIMATION
        DECIMATION=${DECIMATION:-6}
        
        PARAMS="clip_distance:=$CLIP_DISTANCE octomap_resolution:=$RESOLUTION decimation_filter:=$DECIMATION pointcloud_topic:=$POINTCLOUD_TOPIC pointcloud_max_z:=$MAX_Z occupancy_max_z:=$MAX_Z"
        echo -e "${GREEN}自定义参数: $PARAMS${NC}"
        ;;
    3)
        echo -e "${GREEN}高性能模式启动 (2cm 体素, 禁用地面提取)...${NC}"
        PARAMS="clip_distance:=$CLIP_DISTANCE octomap_resolution:=0.02 filter_ground:=false filter_speckles:=false colored_map:=false pointcloud_topic:=$POINTCLOUD_TOPIC pointcloud_max_z:=$MAX_Z occupancy_max_z:=$MAX_Z"
        ;;
    4)
        echo -e "${GREEN}高精度模式启动 (5cm 体素, 降采样, 关闭地面提取)...${NC}"
        PARAMS="clip_distance:=$CLIP_DISTANCE octomap_resolution:=0.05 decimation_filter:=6 filter_ground:=false filter_speckles:=false colored_map:=false pointcloud_topic:=$POINTCLOUD_TOPIC pointcloud_max_z:=$MAX_Z occupancy_max_z:=$MAX_Z"
        ;;
    5)
        echo -e "${GREEN}平衡模式启动 (10cm 体素, 无地面提取)...${NC}"
        PARAMS="clip_distance:=$CLIP_DISTANCE octomap_resolution:=0.1 filter_ground:=false filter_speckles:=false colored_map:=false pointcloud_topic:=$POINTCLOUD_TOPIC pointcloud_max_z:=$MAX_Z occupancy_max_z:=$MAX_Z"
        ;;
    *)
        echo -e "${RED}无效选择，使用默认参数${NC}"
        PARAMS="clip_distance:=$CLIP_DISTANCE pointcloud_topic:=$POINTCLOUD_TOPIC pointcloud_max_z:=$MAX_Z occupancy_max_z:=$MAX_Z"
        ;;
esac

# 启动系统
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}启动 RealSense OctoMap 系统...${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${YELLOW}提示: 按 Ctrl+C 停止系统${NC}"
echo ""
sleep 2

# 执行 launch
echo -e "${YELLOW}应用参数: $PARAMS${NC}"
echo ""
ros2 launch realsense2_camera rs_octomap.launch.py $PARAMS

echo ""
echo -e "${GREEN}系统已关闭${NC}"

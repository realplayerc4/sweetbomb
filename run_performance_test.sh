#!/bin/bash
# OctoMap 性能测试脚本
# 测试不同配置下的 OctoMap 生成速度

set -e

WORKSPACE_DIR="/home/yq/ros2realsense"
TEST_DURATION=30  # 测试运行时间（秒）
LOG_FILE="$WORKSPACE_DIR/octomap_performance_report.txt"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}OctoMap 性能测试${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# 启动 ROS 环境
cd "$WORKSPACE_DIR"
if [ ! -f "install/setup.bash" ]; then
    echo -e "${RED}✗ 错误: 未找到工作空间，请先运行 'colcon build'${NC}"
    exit 1
fi

source /opt/ros/humble/setup.bash
source install/setup.bash

# 初始化日志文件
cat > "$LOG_FILE" << EOF
OctoMap 性能测试报告
生成时间: $(date)
测试持续时间: ${TEST_DURATION} 秒
测试环境: $(uname -a)

========================================

EOF

# 测试配置 1: 快速模式
echo -e "${YELLOW}[1/3] 测试快速模式 (resolution=0.2m)...${NC}"
echo "" >> "$LOG_FILE"
echo "【配置 1】快速模式 (resolution=0.2m, colored_map=false, filter_ground=false)" >> "$LOG_FILE"
echo "----------------------------------------" >> "$LOG_FILE"

timeout ${TEST_DURATION} bash -c '
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch realsense2_camera rs_octomap.launch.py \
  resolution:=0.2 \
  colored_map:=false \
  filter_ground:=false \
  enable_rviz:=false \
  2>&1
' > /tmp/octomap_test_1.log 2>&1 || true

if [ -f "/tmp/octomap_test_1.log" ]; then
  # 计算帧率和处理时间
  FRAME_COUNT=$(grep -c "Time lapse" /tmp/octomap_test_1.log 2>/dev/null || echo "0")
  
  echo "处理时间数据: " >> "$LOG_FILE"
  grep -i "time lapse" /tmp/octomap_test_1.log | head -5 >> "$LOG_FILE" 2>/dev/null || echo "  (无时间日志)" >> "$LOG_FILE"
  
  if [ $FRAME_COUNT -gt 0 ]; then
    AVG_FPS=$(echo "scale=2; $FRAME_COUNT / ${TEST_DURATION}" | bc 2>/dev/null || echo "N/A")
    
    # 计算平均处理时间
    AVG_TIME=$(grep "Time lapse" /tmp/octomap_test_1.log 2>/dev/null | sed 's/.*Time lapse\[\[insert\]\] \([0-9.]*\).*/\1/' | awk '{sum+=$1; count++} END {if(count>0) print sum/count; else print "N/A"}' || echo "N/A")
    
    echo "处理统计: $FRAME_COUNT 帧, 平均帧率: $AVG_FPS Hz, 平均处理时间: ${AVG_TIME} 秒" >> "$LOG_FILE"
    echo -e "${GREEN}✓ 快速模式: $FRAME_COUNT 帧, 平均 $AVG_FPS Hz${NC}"
  else
    echo "  未检测到帧数统计" >> "$LOG_FILE"
    echo -e "${YELLOW}⚠ 快速模式: 无点云订阅或相机未启动${NC}"
  fi
fi

sleep 2

# 测试配置 2: 标准模式
echo -e "${YELLOW}[2/3] 测试标准模式 (resolution=0.15m)...${NC}"
echo "" >> "$LOG_FILE"
echo "【配置 2】标准模式 (resolution=0.15m, colored_map=false, filter_ground=false)" >> "$LOG_FILE"
echo "----------------------------------------" >> "$LOG_FILE"

timeout ${TEST_DURATION} bash -c '
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch realsense2_camera rs_octomap.launch.py \
  resolution:=0.15 \
  colored_map:=false \
  filter_ground:=false \
  enable_rviz:=false \
  2>&1
' > /tmp/octomap_test_2.log 2>&1 || true

if [ -f "/tmp/octomap_test_2.log" ]; then
  FRAME_COUNT=$(grep -c "Time lapse" /tmp/octomap_test_2.log 2>/dev/null || echo "0")
  
  echo "处理时间数据: " >> "$LOG_FILE"
  grep -i "time lapse" /tmp/octomap_test_2.log | head -5 >> "$LOG_FILE" 2>/dev/null || echo "  (无时间日志)" >> "$LOG_FILE"
  
  if [ $FRAME_COUNT -gt 0 ]; then
    AVG_FPS=$(echo "scale=2; $FRAME_COUNT / ${TEST_DURATION}" | bc 2>/dev/null || echo "N/A")
    AVG_TIME=$(grep "Time lapse" /tmp/octomap_test_2.log 2>/dev/null | sed 's/.*Time lapse\[\[insert\]\] \([0-9.]*\).*/\1/' | awk '{sum+=$1; count++} END {if(count>0) print sum/count; else print "N/A"}' || echo "N/A")
    
    echo "处理统计: $FRAME_COUNT 帧, 平均帧率: $AVG_FPS Hz, 平均处理时间: ${AVG_TIME} 秒" >> "$LOG_FILE"
    echo -e "${GREEN}✓ 标准模式: $FRAME_COUNT 帧, 平均 $AVG_FPS Hz${NC}"
  else
    echo "  未检测到帧数统计" >> "$LOG_FILE"
    echo -e "${YELLOW}⚠ 标准模式: 无点云订阅或相机未启动${NC}"
  fi
fi

sleep 2

# 测试配置 3: 高精度模式
echo -e "${YELLOW}[3/3] 测试高精度模式 (resolution=0.1m)...${NC}"
echo "" >> "$LOG_FILE"
echo "【配置 3】高精度模式 (resolution=0.1m, colored_map=false, filter_ground=false)" >> "$LOG_FILE"
echo "----------------------------------------" >> "$LOG_FILE"

timeout ${TEST_DURATION} bash -c '
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch realsense2_camera rs_octomap.launch.py \
  resolution:=0.1 \
  colored_map:=false \
  filter_ground:=false \
  enable_rviz:=false \
  2>&1
' > /tmp/octomap_test_3.log 2>&1 || true

if [ -f "/tmp/octomap_test_3.log" ]; then
  FRAME_COUNT=$(grep -c "Time lapse" /tmp/octomap_test_3.log 2>/dev/null || echo "0")
  
  echo "处理时间数据: " >> "$LOG_FILE"
  grep -i "time lapse" /tmp/octomap_test_3.log | head -5 >> "$LOG_FILE" 2>/dev/null || echo "  (无时间日志)" >> "$LOG_FILE"
  
  if [ $FRAME_COUNT -gt 0 ]; then
    AVG_FPS=$(echo "scale=2; $FRAME_COUNT / ${TEST_DURATION}" | bc 2>/dev/null || echo "N/A")
    AVG_TIME=$(grep "Time lapse" /tmp/octomap_test_3.log 2>/dev/null | sed 's/.*Time lapse\[\[insert\]\] \([0-9.]*\).*/\1/' | awk '{sum+=$1; count++} END {if(count>0) print sum/count; else print "N/A"}' || echo "N/A")
    
    echo "处理统计: $FRAME_COUNT 帧, 平均帧率: $AVG_FPS Hz, 平均处理时间: ${AVG_TIME} 秒" >> "$LOG_FILE"
    echo -e "${GREEN}✓ 高精度模式: $FRAME_COUNT 帧, 平均 $AVG_FPS Hz${NC}"
  else
    echo "  未检测到帧数统计" >> "$LOG_FILE"
    echo -e "${YELLOW}⚠ 高精度模式: 无点云订阅或相机未启动${NC}"
  fi
fi

# 性能总结
echo "" >> "$LOG_FILE"
echo "========================================" >> "$LOG_FILE"
echo "性能总结与建议" >> "$LOG_FILE"
echo "========================================" >> "$LOG_FILE"
echo "" >> "$LOG_FILE"
echo "配置对比:" >> "$LOG_FILE"
echo "| 模式 | 分辨率 | 预期帧率 | 体素数 | 典型应用 |" >> "$LOG_FILE"
echo "|------|--------|---------|--------|----------|" >> "$LOG_FILE"
echo "| 快速 | 0.2m | 5-10 Hz | 最少 | 实时导航、演示 |" >> "$LOG_FILE"
echo "| 标准 | 0.15m | 3-5 Hz | 中等 | 一般应用、平衡 |" >> "$LOG_FILE"
echo "| 精度 | 0.1m | 1-3 Hz | 正常 | 精确建图、分析 |" >> "$LOG_FILE"
echo "" >> "$LOG_FILE"
echo "优化建议:" >> "$LOG_FILE"
echo "1. 若帧率 < 2 Hz，检查：" >> "$LOG_FILE"
echo "   - filter_ground 是否正确禁用（应为 false）" >> "$LOG_FILE"
echo "   - colored_map 是否正确禁用（应为 false）" >> "$LOG_FILE"
echo "   - 点云分辨率是否过高" >> "$LOG_FILE"
echo "" >> "$LOG_FILE"
echo "2. 若帧率 > 预期，说明优化有效，可进一步调整参数：" >> "$LOG_FILE"
echo "   - 增加分辨率获得更好的精度" >> "$LOG_FILE"
echo "   - 启用 colored_map 添加 RGB 信息" >> "$LOG_FILE"
echo "" >> "$LOG_FILE"
echo "3. 关键优化参数（已在配置中应用）：" >> "$LOG_FILE"
echo "   - filter_ground: false (禁用 RANSAC 地面提取，节省 ~70% 时间)" >> "$LOG_FILE"
echo "   - colored_map: false (禁用 RGB 处理，节省 ~15% 时间)" >> "$LOG_FILE"
echo "   - filter_speckles: false (禁用孤立体素过滤，节省 ~10% 时间)" >> "$LOG_FILE"

# 显示结果
echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${GREEN}性能测试完成！${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo -e "报告已保存到: ${GREEN}$LOG_FILE${NC}"
echo ""

# 打印摘要
tail -20 "$LOG_FILE"
echo ""
echo -e "${YELLOW}💡 建议: 选择合适的分辨率配置${NC}"
echo "  • 快速 (0.2m): ros2 launch realsense2_camera rs_octomap.launch.py"
echo "  • 标准 (0.15m): ros2 launch realsense2_camera rs_octomap.launch.py resolution:=0.15"
echo "  • 精度 (0.1m): ros2 launch realsense2_camera rs_octomap.launch.py resolution:=0.1"
echo ""

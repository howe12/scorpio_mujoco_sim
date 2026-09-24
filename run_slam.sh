#!/bin/bash
# Scorpio SLAM 建图 - 完整运行指南
# 
# 使用方法:
#   ./run_slam.sh [terrain]
#
# 示例:
#   ./run_slam.sh plaza        # 室外广场
#   ./run_slam.sh mountain     # 山地
#   ./run_slam.sh forest       # 森林

set -e

TERRAIN=${1:-plaza}

echo "=== Scorpio SLAM 建图启动脚本 ==="
echo "场景: $TERRAIN"
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# 1. 检查环境
echo -e "${GREEN}[1/4] 检查环境...${NC}"
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/humble/setup.bash
fi

# 检查编译产物
if [ ! -d "/develop/scorpio_ros2/install/scorpio_mujoco_sim" ]; then
    echo -e "${YELLOW}  编译产物不存在，先同步并编译...${NC}"
    cd /develop/scorpio_mujoco_sim
    ./sync_to_ros2.sh
    cd /develop/scorpio_ros2
    colcon build --packages-select scorpio_mujoco_sim
fi

# 2. source 工作区
echo -e "${GREEN}[2/4] 加载工作区...${NC}"
cd /develop/scorpio_ros2
source install/setup.bash

# 3. 启动 SLAM
echo -e "${GREEN}[3/4] 启动 SLAM Toolbox...${NC}"
echo -e "${YELLOW}  在新终端运行: python3 scripts/run_seb_naver_sim.py --terrain ${TERRAIN}${NC}"
echo ""

# 4. 启动 launch
echo -e "${GREEN}[4/4] 启动 launch 文件...${NC}"
ros2 launch scorpio_mujoco_sim slam_mapping.launch.py terrain:=${TERRAIN}
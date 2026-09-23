#!/bin/bash
# 测试 SLAM 建图功能
# 
# 使用方法:
#   ./test_slam_mapping.sh [terrain]
#
# 示例:
#   ./test_slam_mapping.sh plaza
#   ./test_slam_mapping.sh mountain

set -e

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

TERRAIN=${1:-plaza}

echo -e "${GREEN}=== Scorpio SLAM 建图测试 ===${NC}"
echo -e "${YELLOW}场景: ${TERRAIN}${NC}"
echo ""

# 1. 检查 ROS2 环境
echo -e "${GREEN}[1/5] 检查 ROS2 环境...${NC}"
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/humble/setup.bash
fi
echo "  ROS2 $ROS_DISTRO: OK"

# 2. 检查依赖包
echo -e "${GREEN}[2/5] 检查依赖包...${NC}"
ros2 pkg list | grep -q "slam_toolbox" && echo "  slam_toolbox: OK" || echo "  slam_toolbox: NOT FOUND"
ros2 pkg list | grep -q "scorpio_mujoco_sim" && echo "  scorpio_mujoco_sim: OK" || echo "  scorpio_mujoco_sim: NOT FOUND"

# 3. 检查仿真模型
echo -e "${GREEN}[3/5] 检查仿真模型...${NC}"
if [ -f "worlds/${TERRAIN}.xml" ]; then
    echo "  场景文件: worlds/${TERRAIN}.xml ✅"
else
    echo -e "${RED}  场景文件不存在: worlds/${TERRAIN}.xml ❌${NC}"
    echo "  可用场景:"
    ls worlds/*.xml 2>/dev/null | sed 's|.*worlds/||' | sed 's|.xml||'
    exit 1
fi

# 4. 测试 LiDAR 仿真
echo -e "${GREEN}[4/5] 测试 LiDAR 仿真...${NC}"
python3 -c "
import sys
sys.path.insert(0, 'scripts')
from lidar_simulator import LiDARSimulator
from run_seb_naver_sim import load_world
import mujoco

m, d = load_world('${TERRAIN}')
mujoco.mj_forward(m, d)
lidar = LiDARSimulator(m, d)
ranges = lidar.scan()
valid = ranges[ranges < 16.0]

print(f'  LiDAR 点数: {len(ranges)}')
print(f'  有效点数: {len(valid)}')
print(f'  距离范围: {valid.min():.2f} ~ {valid.max():.2f} m')
print(f'  平均距离: {valid.mean():.2f} m')
" 2>&1 | grep -v "Loaded heightfield"

echo ""
echo -e "${GREEN}[5/5] 准备启动 SLAM 建图...${NC}"
echo ""
echo "=== 启动步骤 ==="
echo ""
echo "1. 在当前终端运行 MuJoCo 仿真:"
echo "   python3 scripts/run_seb_naver_sim.py --terrain ${TERRAIN}"
echo ""
echo "2. 在新终端运行 SLAM Toolbox:"
echo "   cd /develop/scorpio_ros2"
echo "   source install/setup.bash"
echo "   ros2 launch scorpio_mujoco_sim slam_mapping.launch.py terrain:=${TERRAIN}"
echo ""
echo "3. 在另一个新终端查看地图:"
echo "   rviz2"
echo "   添加显示: /map (Map), /scan (LaserScan), /odom (Odometry)"
echo ""
echo "4. 在 MuJoCo 窗口中用键盘控制小车运动建图"
echo ""
echo -e "${GREEN}=== 测试完成 ===${NC}"
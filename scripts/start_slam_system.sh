#!/bin/bash
# 一键启动 MuJoCo 仿真 + SLAM + RViz2
#
# 使用方法: ./start_slam_system.sh [terrain]
# 示例: ./start_slam_system.sh plaza

TERRAIN=${1:-plaza}

echo "=== 启动 Scorpio SLAM 系统 ==="
echo "场景: $TERRAIN"
echo ""

# 1. 检查并设置显示环境
if [ -z "$DISPLAY" ]; then
    export DISPLAY=:0
fi
if [ -z "$XAUTHORITY" ]; then
    export XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority
fi
echo "显示环境: DISPLAY=$DISPLAY"
echo ""

# 2. 启动 MuJoCo 仿真（后台）
echo "启动 MuJoCo 仿真..."
cd /develop/scorpio_mujoco_sim
python3 scripts/run_seb_naver_sim.py --terrain $TERRAIN &
MUJOCO_PID=$!

# 等待 MuJoCo 窗口启动
sleep 3

# 3. 启动 SLAM + RViz2
echo "启动 SLAM + RViz2..."
cd /develop/scorpio_mujoco_sim_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch scorpio_mujoco_sim slam_mapping.launch.py terrain:=$TERRAIN &

# 4. 等待用户按 Ctrl+C
echo ""
echo "=== 系统已启动 ==="
echo "MuJoCo 窗口: 显示小车和场景"
echo "RViz2 窗口: 显示 LiDAR 数据和地图"
echo ""
echo "操作说明:"
echo "  - 在 MuJoCo 窗口中用键盘控制小车 (W/A/S/D)"
echo "  - 在 RViz2 中查看建图过程"
echo "  - 按 Ctrl+C 停止所有进程"
echo ""

# 捕获 Ctrl+C，停止所有进程
trap "echo '停止所有进程...'; kill $MUJOCO_PID 2>/dev/null; pkill -f 'slam_toolbox|rviz2' 2>/dev/null; exit 0" INT

# 等待
wait
#!/usr/bin/env bash
#
# collect_diagnostics.sh — 一键收集导航问题的诊断信息
#
# 用途:
#   导航表现异常（撞墙、跑偏、卡住、速度慢、到不了目标）时，
#   先运行本脚本，它会把定位问题所需的信息打包到 ~/seb_diag_<时间戳>/
#
# 用法:
#   ./scripts/collect_diagnostics.sh              # 采集 10 秒
#   ./scripts/collect_diagnostics.sh 20           # 采集 20 秒
#
# 前提: 仿真/导航已在另一个终端启动并处于问题状态
#
set -uo pipefail

DUR="${1:-10}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="$HOME/seb_diag_$STAMP"
mkdir -p "$OUT"

echo "=========================================================="
echo " SEB-Naver 诊断收集"
echo " 输出目录: $OUT"
echo " 采集时长: ${DUR}s"
echo "=========================================================="

have() { command -v "$1" >/dev/null 2>&1; }

# ---- 0. 环境 ----
{
  echo "== date =="; date -Is
  echo "== ros nodes =="; timeout 5 ros2 node list 2>&1
  echo "== ros topics =="; timeout 5 ros2 topic list 2>&1
  echo "== ros distro =="; echo "${ROS_DISTRO:-unset}"
  echo "== workspace =="; echo "${AMENT_PREFIX_PATH:-unset}"
} > "$OUT/00_env.txt" 2>&1
echo "[1/7] 环境与节点拓扑 → 00_env.txt"

# ---- 1. 话题频率（判断数据流是否正常）----
{
  for t in /Odometry /sdf_map /fused_map /planner/trajectory /planner/rough_path \
           /planner/status /cmd_vel /livox/lidar /Laser_map; do
    echo "== hz $t =="
    timeout 4 ros2 topic hz "$t" 2>&1 | grep -E "average rate|no new" | head -2
  done
} > "$OUT/01_topic_hz.txt" 2>&1
echo "[2/7] 话题频率 → 01_topic_hz.txt"

# ---- 2. 机器人状态（位置/速度/目标）----
{
  echo "== /odom (SLAM) =="
  timeout 5 ros2 topic echo /odom --once 2>&1 | head -30
  echo "== /cmd_vel =="
  timeout 5 ros2 topic echo /cmd_vel --once 2>&1 | head -12
  echo "== /planner/status =="
  timeout 5 ros2 topic echo /planner/status --once 2>&1 | head -5
  echo "== /goal_pose (最近目标: 用 echo 无法取历史，仅供参考) =="
  timeout 3 ros2 topic info /goal_pose 2>&1
} > "$OUT/02_robot_state.txt" 2>&1
echo "[3/7] 机器人状态 → 02_robot_state.txt"

# ---- 3. TF 树（TF 冲突是常见元凶）----
{
  echo "== base_footprint 的父节点（应为 body）=="
  timeout 10 ros2 run tf2_tools view_frames 2>&1 | grep -oE "base_footprint: .{0,30}"
  echo "== static_transform_publisher 进程数（应为 1）=="
  pgrep -fa static_transform_publisher | wc -l
  echo "== tf2_echo body→base_footprint =="
  timeout 4 ros2 run tf2_ros tf2_echo body base_footprint 2>&1 | head -8
} > "$OUT/03_tf.txt" 2>&1
echo "[4/7] TF 树 → 03_tf.txt"

# ---- 4. 地图内容（判断感知是否正常）----
{
  echo "== /sdf_map 元信息 =="
  timeout 6 ros2 topic echo /sdf_map --once --field info 2>&1 | head -25
  echo "== /sdf_map 层名 =="
  timeout 6 ros2 topic echo /sdf_map --once --field layers 2>&1 | head -3
  echo "== /fused_map 层名 =="
  timeout 6 ros2 topic echo /fused_map --once --field layers 2>&1 | head -3
  echo "== /Laser_map 点数 =="
  timeout 6 ros2 topic echo /Laser_map --once --field width 2>&1 | head -2
  echo "== /planner/trajectory 点数 =="
  timeout 6 ros2 topic echo /planner/trajectory --once --field poses 2>&1 | grep -c "position" 
} > "$OUT/04_maps.txt" 2>&1
echo "[5/7] 地图内容 → 04_maps.txt"

# ---- 5. 参数快照（改了什么一眼可见）----
{
  echo "== planner 参数 =="
  timeout 8 ros2 param dump /seb_naver_planner 2>&1
  echo "== mpc 参数 =="
  timeout 8 ros2 param dump /mpc_controller 2>&1
  echo "== terrain_analyzer 参数 =="
  timeout 8 ros2 param dump /terrain_analyzer 2>&1
} > "$OUT/05_params.txt" 2>&1
echo "[6/7] 参数快照 → 05_params.txt"

# ---- 6. 实时采样 + 进程资源 ----
{
  echo "== CPU 占用（看 terrain_analyzer / fast_lio 是否吃满）=="
  ps -eo pid,pcpu,pmem,comm --sort=-pcpu | head -12
  echo
  echo "== 关键进程 =="
  pgrep -fa "terrain_analyzer|fastlio|laser_mapping|planner_node|mpc_node|scorpio" | head -12
} > "$OUT/06_resources.txt" 2>&1
echo "[7/7] 进程资源 → 06_resources.txt"

# ---- 7. 导航日志（从 launch 重定向文件里抓）----
{
  echo "== 若你把 launch 输出重定向到了文件，请手动附加；这里抓常见位置 =="
  for f in /tmp/nav.log /tmp/p*.log; do
    [ -f "$f" ] || continue
    echo "---- $f (末 120 行中的关键行) ----"
    grep -E "Planning pipeline|traj opt|rejected by|A\* search failed|Goal reached|FSM:|solver error" "$f" 2>/dev/null | tail -60
  done
} > "$OUT/07_nav_log.txt" 2>&1

cat > "$OUT/SUMMARY.txt" <<EOF
SEB-Naver 诊断包  $STAMP
采集时长 ${DUR}s

文件清单:
  00_env.txt       节点/话题拓扑
  01_topic_hz.txt  各话题频率
  02_robot_state.txt  位姿/速度/FSM 状态
  03_tf.txt        TF 树与冲突检查
  04_maps.txt      SDF/高程/点云/轨迹规模
  05_params.txt    三个节点的全部参数
  06_resources.txt CPU 与进程
  07_nav_log.txt   导航关键日志

自查要点（先自己看一眼，很多问题这里就能定位）:
  1. 01_topic_hz.txt: /sdf_map 与 /fused_map 是否 ~5-10Hz？为 0 说明 terrain_analyzer 挂了
  2. 03_tf.txt: "base_footprint: parent: 'body'" 才对；若为 odom 或 static_transform
     进程数 >1，则是 TF 冲突（典型症状: RViz 模型错乱、第二次导航失效）
  3. 04_maps.txt: /Laser_map 点数应随行驶增长；/planner/trajectory 点数应为 15-40
  4. 06_resources.txt: terrain_analyzer 长期占满 CPU 说明降采样参数需要调
  5. 07_nav_log.txt: 大量 "rejected by risk/attitude" 说明约束太严；
     大量 "A* fallback" 说明轨迹优化被频繁拒绝；
     "A* search failed" 说明膨胀/坡度 mask 挡死了通路

压缩后可直接发给维护者:
  tar -czf ~/seb_diag_$STAMP.tar.gz -C ~ "seb_diag_$STAMP"
EOF

echo
echo "=========================================================="
echo " 完成: $OUT"
echo " 先看 SUMMARY.txt 里的自查要点"
echo " 打包: tar -czf ~/seb_diag_$STAMP.tar.gz -C ~ seb_diag_$STAMP"
echo "=========================================================="

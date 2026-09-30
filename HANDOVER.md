# Scorpio MuJoCo 仿真 + SEB-Naver 自主导航系统 — 交接文档

> 本文档面向新接手本工程的开发者或 AI Agent，涵盖环境准备、编译、运行、测试、调试和已知问题。

---

## 1. 项目概述

本项目是 **NXROBO Scorpio 机器人**的 MuJoCo 物理仿真 + ROS2 自主导航系统，复现论文 *"SEB-Naver: A SE(2)-based Local Navigation Framework for Car-like Robots on Uneven Terrain"* 的核心算法。

**核心能力**：在 MuJoCo 仿真环境中，机器人通过 3D LiDAR + IMU 进行 SLAM 定位，构建局部高程图并检测障碍物，使用 Kinodynamic A* 规划路径，MPC 跟踪控制，实现 RViz SetGoal 点击式自主导航。

### 1.1 技术栈

| 组件 | 技术 | 版本 |
|------|------|------|
| OS | Ubuntu 22.04 LTS | — |
| ROS2 | Humble Hawksbill | — |
| 物理仿真 | MuJoCo (Python bindings) | 3.10.0 |
| SLAM | FAST-LIO2 (修改版) | C++ / ROS2 |
| 轨迹优化 | CasADi | 3.8.1 (Python, pip) |
| GPU | NVIDIA RTX 2070 SUPER | Driver 580.x |
| Python | CPython | 3.10 |

### 1.2 仓库结构

```
/develop/scorpio_mujoco_sim_ws/          # colcon 工作空间根目录
├── src/
│   ├── scorpio_mujoco_sim/              # Git repo → github.com/howe12/scorpio_mujoco_sim
│   │   ├── scripts/                     # Python 仿真脚本
│   │   ├── models/                      # MuJoCo XML + URDF + meshes
│   │   ├── worlds/                      # 5 个场景 XML
│   │   └── config/                      # RViz 配置
│   ├── FAST_LIO/                        # 本地 fork (无远程), 含 ikd-Tree 子模块
│   ├── seb_naver_ros2/                  # Git repo (本地, 无远程)
│   │   ├── se2_grid_msgs/               # ROS2 消息定义
│   │   ├── se2_grid_core/               # SE2Grid C++ 库 (header-only)
│   │   ├── terrain_analyzer/            # 高程图 + 障碍物检测 (C++)
│   │   └── seb_naver_planner/           # A* + MPC + FSM (Python)
│   └── livox_ros_driver2/               # Livox 消息定义
├── install/                             # colcon build 输出
├── build/                               # colcon build 中间文件
├── clean_ros.sh                         # 残留进程清理脚本
└── HANDOVER.md                          # 本文档
```

---

## 2. 环境准备

### 2.1 系统依赖

```bash
# Ubuntu 22.04
sudo apt update
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-rviz2 \
    ros-humble-tf2-ros \
    ros-humble-robot-state-publisher \
    python3-pip \
    libeigen3-dev \
    libpcl-dev \
    cmake build-essential

# MuJoCo (pip, 用户级安装)
pip3 install mujoco==3.10.0

# CasADi (pip, 必须用系统 Python 3.10, 不能用 conda)
/usr/bin/python3 -m pip install casadi==3.8.1

# 验证
python3 -c "import mujoco; print(f'mujoco {mujoco.__version__}')"
python3 -c "import casadi; print(f'casadi {casadi.__version__}')"
python3 -c "import rclpy; print('rclpy OK')"
```

> ⚠️ **关键约束**：所有涉及 `rclpy` 的操作必须使用 `/usr/bin/python3`（系统 Python），不能用 conda 环境的 Python（libstdc++ 版本不兼容）。

### 2.2 ROS2 环境

```bash
# 每次开新终端都要 source
source /opt/ros/humble/setup.bash

# 工作空间编译后还要 source install
source /develop/scorpio_mujoco_sim_ws/install/setup.bash
```

### 2.3 显示环境

```bash
# GUI 模式 (MuJoCo 窗口 / RViz) 需要 X11
export DISPLAY=:0
export XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority

# 如果是 SSH 远程, 需要 X11 forwarding 或 VNC
```

### 2.4 首次编译

```bash
cd /develop/scorpio_mujoco_sim_ws
source /opt/ros/humble/setup.bash

# 按依赖顺序编译
colcon build --packages-select se2_grid_msgs
colcon build --packages-select se2_grid_core
colcon build --packages-select livox_ros_driver2
colcon build --packages-select fast_lio
colcon build --packages-select scorpio_mujoco_sim
colcon build --packages-select terrain_analyzer
colcon build --packages-select seb_naver_planner

# 或一次性全部编译 (较慢)
colcon build

source install/setup.bash
```

> ⚠️ FAST_LIO 编译约 2-3 分钟（C++ 模板重）。如果报 `ikd_Tree.h not found`，确认 `src/FAST_LIO/include/ikd-Tree/` 目录存在且包含 `ikd_Tree.cpp` 和 `ikd_Tree.h`。

---

## 3. 运行方式

### 3.1 完整自主导航 (推荐)

```bash
source /opt/ros/humble/setup.bash
source /develop/scorpio_mujoco_sim_ws/install/setup.bash
export DISPLAY=:0
export XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority

# 纯 RViz (无 MuJoCo 窗口, 后台仿真)
ros2 launch seb_naver_planner seb_naver_full.launch.py terrain:=plaza

# 带 MuJoCo 3D 窗口
ros2 launch seb_naver_planner seb_naver_full.launch.py terrain:=plaza sim_gui:=true

# 可选参数
#   terrain:= plaza | forest | mountain | pump_track | snowy_mountain
#   sim_gui:= true | false  (默认 false)
#   rviz:=    true | false  (默认 true)
#   sim:=     true | false  (默认 true, 设为 false 可接真实机器人)
```

启动后在 RViz 中使用 **SetGoal** 工具（绿色箭头图标）点击目标位置，机器人自主导航。

### 3.2 仅建图 (不导航)

```bash
ros2 launch terrain_analyzer terrain_mapping.launch.py terrain:=forest
```

### 3.3 手动键盘控制 (不启动 ROS2)

```bash
cd /develop/scorpio_mujoco_sim_ws/src/scorpio_mujoco_sim
python3 scripts/run_seb_naver_sim.py --terrain plaza
# W/S 前进后退, A/D 转向, Tab 切换自动导航, ESC 退出
```

### 3.4 Headless 基准测试

```bash
python3 scripts/run_seb_naver_sim.py --headless
# 在所有地形上跑性能基准
```

---

## 4. 数据流与话题

### 4.1 话题一览

```
MuJoCo Sim (scorpio_ros2_publisher)
  ├── /livox/lidar      PointCloud2    10Hz   360×32 线, time=0.001
  ├── /imu/data          Imu            200Hz  加速度+角速度+姿态
  ├── /scan              LaserScan      10Hz   2D 水平扫描
  ├── /odom              Odometry       50Hz   ground truth (sim 真值)
  └── /wheel_odom        TwistStamped   50Hz   轮速里程计 + 打滑置信度 (linear.y)

FAST-LIO2 (laser_mapping)
  ├── /Odometry          Odometry       ~10Hz  SLAM 位姿 (camera_init→body)
  ├── /cloud_registered  PointCloud2    ~10Hz  配准后点云
  ├── /Laser_map         PointCloud2    ~1Hz   累积地图
  └── /path              Path           ~10Hz  历史轨迹

TerrainAnalyzer (terrain_analyzer)
  ├── /fused_map         SE2Grid        5Hz    融合高程图
  └── /sdf_map           SE2Grid        5Hz    SDF 占用图 (-1=occupied, 1=free, 0.5=unknown)

Planner (seb_naver_planner)
  ├── /planner/trajectory  Path         2Hz    MPC 跟踪路径
  ├── /planner/rough_path  Path         2Hz    A* 原始路径
  └── /planner/status      String       2Hz    FSM 状态名

MPC (mpc_controller)
  └── /cmd_vel           Twist          20Hz   速度指令 (linear.x, angular.z)

RViz 交互
  └── /goal_pose         PoseStamped    按需   SetGoal 点击发布
```

### 4.2 TF 树

```
camera_init  ← FAST-LIO2 世界系 (SLAM 原点)
  └── body   ← FAST-LIO2 机体 (动态, ~10Hz)
       └── base_footprint  ← static_transform_publisher (固定)
            └── base_link  ← robot_state_publisher / URDF (固定)
                 ├── lidar_link      (z=+0.282)
                 ├── IMU_link        (x=+0.16, y=+0.04, z=+0.175)
                 ├── d435_link
                 ├── left_rear_wheel
                 ├── right_rear_wheel
                 ├── left_front_wheel
                 └── right_front_wheel
```

> ⚠️ **TF 冲突陷阱**：sim 的 `--no-tf-odom` 参数阻止发布 `odom→base_footprint`。如果漏传此参数（如 GUI 模式旧 bug），`base_footprint` 会有两个父节点（`odom` 和 `body`），导致 RViz 小车模型显示失败。**已修复**，但修改 sim 启动参数时务必保留 `--no-tf-odom`。

---

## 5. 关键参数说明

### 5.1 机器人物理参数 (`scorpio.xml`)

| 参数 | 值 | 说明 |
|------|-----|------|
| timestep | 0.001s | MuJoCo 积分步长 (**不能改大**, >0.002 转弯数值爆炸) |
| integrator | RK4 | 四阶 Runge-Kutta |
| wheel_base | 0.315m | 前后轴距离 |
| wheel_radius | 0.0525m | 轮子半径 |
| max_steer | 0.785 rad (45°) | 最大转向角 |
| velocity kv | 1.0 | 轮速闭环增益 (太小打滑, 太大 IMU 噪声) |
| forcerange | ±1.5 Nm | 轮子力矩限制 |
| steering kp/kv | 60/8 | 转向位置控制 |
| 整车质量 | ~5.33 kg | — |

### 5.2 SLAM 参数 (launch 内联)

| 参数 | 值 | 说明 |
|------|-----|------|
| lidar_type | 2 (VELO16) | velodyne_handler, 需要 ring 字段 |
| scan_line | 32 | LiDAR 线数 |
| timestamp_unit | 0 (SEC) | 时间戳单位 |
| blind | 0.5m | 盲区过滤 |
| det_range | 40.0m | 地图管理范围 (**默认 300 会清空地图**) |
| fov_degree | 360 | 视野角 |
| acc_cov | 0.1 | IMU 加速度噪声协方差 |
| gyr_cov | 0.1 | IMU 陀螺仪噪声协方差 |
| wheel_odom.enable | true | 轮速校正开关 (抑制崎岖地形 IMU 冲击漂移) |
| point_filter_num | 1 | 点云下采样 (1=全保留) |
| filter_size_surf/map | 0.3m | 体素下采样尺寸 |

### 5.3 导航参数 (`planning.yaml`)

| 参数 | 值 | 说明 |
|------|-----|------|
| max_speed | 0.5 m/s | 最大速度 |
| step_arc | 1.0m | A* 弧长步长 |
| steer_res | 0.3 rad | A* 转向分辨率 |
| max_search_time | 5.0s | A* 超时 |
| replan_interval | 1.0s | 重规划间隔 |
| goal_tolerance_dist | 0.8m | 到达距离容差 |
| N_horizon | 20 | MPC 预测步数 |
| dt | 0.05s | MPC 步长 (1s 视界) |
| Q | [10, 10, 2] | 状态权重 [x, y, yaw] |
| R | [10, 10] | 控制权重 [v, δ] |
| Rd | [5, 3] | 控制变化率权重 |

### 5.4 地形分析参数 (`terrain_analyzer.yaml`)

| 参数 | 值 | 说明 |
|------|-----|------|
| length_pos_x/y | 20.0m | 地图尺寸 (滚动窗口) |
| resolution_pos | 0.1m | 栅格分辨率 |
| ignore_z_min/max | [-3.0, 2.0] | body 系高度过滤 |
| obstacle_z_threshold | 0.15m | 障碍物高度阈值 (代码内硬编码) |
| obstacle_point_threshold | 3 | 最少高点数标记 occupied |
| mahalanobis_threshold | 2.5 | Kalman 异常值拒绝 |

---

## 6. 测试方法

### 6.1 基本功能测试

```bash
# 1. 编译检查
cd /develop/scorpio_mujoco_sim_ws
source /opt/ros/humble/setup.bash
colcon build 2>&1 | tail -5
# 期望: "Summary: N packages finished" 无 failed

# 2. 启动完整系统
source install/setup.bash
export DISPLAY=:0 && export XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority
ros2 launch seb_naver_planner seb_naver_full.launch.py terrain:=plaza sim_gui:=false rviz:=false &
sleep 40

# 3. 检查各节点是否就绪
grep -E "SQP solver|Planner initialized|Initialize the map|Frame 30" /tmp/launch_*.log | tail -5
# 期望: 4 行日志 (MPC/Planner/FAST-LIO2/TerrainAnalyzer 各一条)

# 4. 检查话题
ros2 topic list | grep -cE "sdf_map|Odometry|cmd_vel|goal_pose"
# 期望: ≥ 4

# 5. 发目标测试导航
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'camera_init'}, pose: {position: {x: 3.0, y: 0.0}, orientation: {w: 1.0}}}"
sleep 15

# 6. 检查机器人是否移动
ros2 topic echo /odom --once | grep -A2 "position:"
# 期望: x ≈ 3.0 (到达目标附近)

# 7. 检查 SLAM 稳定性
ros2 topic echo /Odometry --once | grep -A2 "position:"
# 期望: 与 /odom 接近 (偏差 < 1m)

# 8. 清理
/develop/scorpio_mujoco_sim_ws/clean_ros.sh
```

### 6.2 避障测试

```bash
# plaza 场景有墙壁 wall_low_1 在 x=4.0
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'camera_init'}, pose: {position: {x: 6.0, y: 0.0}, orientation: {w: 1.0}}}"
sleep 20
ros2 topic echo /odom --once | grep -A2 "position:"
# 期望: x < 4.0 (停在墙前, 不撞墙)
```

### 6.3 多场景测试

```bash
for terrain in plaza forest mountain pump_track snowy_mountain; do
    echo "=== Testing $terrain ==="
    /develop/scorpio_mujoco_sim_ws/clean_ros.sh
    ros2 launch seb_naver_planner seb_naver_full.launch.py terrain:=$terrain rviz:=false &
    sleep 40
    ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
      "{header: {frame_id: 'camera_init'}, pose: {position: {x: 3.0, y: 0.0}, orientation: {w: 1.0}}}"
    sleep 15
    ros2 topic echo /odom --once | grep -A2 "position:"
    /develop/scorpio_mujoco_sim_ws/clean_ros.sh
done
```

### 6.4 单元测试 (无需 ROS2)

```bash
# MuJoCo 物理稳定性测试
cd /develop/scorpio_mujoco_sim_ws/src/scorpio_mujoco_sim
python3 -c "
import mujoco, numpy as np
m = mujoco.MjModel.from_xml_path('worlds/plaza.xml')
d = mujoco.MjData(m)
mujoco.mj_forward(m, d)
# 静止 1s
for i in range(1000): mujoco.mj_step(m, d)
v = np.linalg.norm(d.qvel[:2])
assert v < 0.01, f'静止不稳定: v={v}'
# 直行 0.5 m/s
d.ctrl[0] = d.ctrl[1] = 0.5 / 0.0525
for i in range(1000): mujoco.mj_step(m, d)
v = np.linalg.norm(d.qvel[:2])
assert 0.4 < v < 0.6, f'直行速度异常: v={v}'
# 转弯
d.ctrl[2] = np.radians(15)
for i in range(1000): mujoco.mj_step(m, d)
v = np.linalg.norm(d.qvel[:2])
assert v < 2.0, f'转弯失稳: v={v}'
print('✅ 物理稳定性测试通过')
"

# Python 语法检查
for f in src/seb_naver_ros2/seb_naver_planner/seb_naver_planner/*.py; do
    python3 -c "import py_compile; py_compile.compile('$f', doraise=True)" && echo "✅ $(basename $f)"
done
```

---

## 7. 调试指南

### 7.1 常见问题速查

| 症状 | 原因 | 修复 |
|------|------|------|
| RViz 小车模型不显示 / 乱跳 | TF 冲突 (base_footprint 多父) | 确认 sim 启动带 `--no-tf-odom`; 运行 `clean_ros.sh` |
| SLAM 定位爆炸 (坐标千万级) | 残留进程发 cmd_vel 导致转弯失稳 | `clean_ros.sh` 后重启 |
| SLAM z 轴漂移 (悬空几米) | 不平地形初始悬空 → IMU 初始化污染 | 已修复 (snap_to_ground + 稳定期) |
| 导航方向反 / 偏转 | 初始朝向非 0° → 坐标系偏差 | 已修复 (reset_robot_facing_goal 设 yaw=0) |
| 机器人不动 | terrain_analyzer 未启动 → 无 SDF → planner 等待 | 检查 `ros2 topic list \| grep sdf_map` |
| MPC solver failed | 短程转弯 SQP 迭代超限 | 已有 fallback (沿用上次控制衰减) |
| 撞墙 | SDF 无障碍物信息 | 已修复 (高点统计标记 occupied) |
| `No Effective Points` 持续 | det_range 太大 (默认 300) 清空地图 | 已修复 (det_range=40) |
| 转弯时速度爆炸 70+ m/s | timestep > 0.002 数值发散 | 已修复 (timestep=0.001) |

### 7.2 诊断命令

```bash
# 检查 TF 树
ros2 run tf2_tools view_frames  # 生成 frames.pdf

# 检查话题频率
ros2 topic hz /cloud_registered --window 5
ros2 topic hz /Odometry --window 5
ros2 topic hz /cmd_vel --window 5

# 检查 IMU 数据质量 (静止时应为 ~9.81 m/s²)
ros2 topic echo /imu/data --once | grep -A3 "linear_acceleration"

# 检查 LiDAR 点数 (应 > 2000)
ros2 topic echo /livox/lidar --once | grep "width"

# 检查 SDF 地图 (应有 occupied 区域)
ros2 topic echo /sdf_map --once | head -20

# 检查 planner 状态
ros2 topic echo /planner/status --once

# 查看各节点日志
ls -t ~/.ros/log/latest/*.log | head -10
```

### 7.3 清理残留进程

```bash
# 一键清理
/develop/scorpio_mujoco_sim_ws/clean_ros.sh

# 手动清理
pkill -9 -f "ros2 launch"
pkill -9 -f run_seb_naver_sim
pkill -9 -f fastlio_mapping
pkill -9 -f terrain_analyzer_node
pkill -9 -f planner_node
pkill -9 -f mpc_node
pkill -9 -f static_transform_publisher
pkill -9 -f robot_state_publisher
pkill -9 -f rviz2
```

> ⚠️ **重要**：退出 launch 时用 **Ctrl-C**（SIGINT），不要用 `kill -9`（SIGKILL）。SIGKILL 不给 launch 清理子进程的机会，会留下大量僵尸 `static_transform_publisher` 进程，导致 TF 冲突。

---

## 8. 已知局限与待办

### 8.1 已知局限

1. **绕路导航**：目标在障碍物后方时，A* 在 20m 地图 + 5s 超时内可能找不到绕路路径。需要更大地图或分层规划（全局 + 局部）。
2. **长时间 SLAM 漂移**：FAST-LIO2 无回环检测，长时间运行后 z 轴有缓慢累积漂移（~0.1m/min）。论文本身就是局部导航不回环。
3. **PHR-ALM 未实现**：当前直接用 A* 粗略路径，未经过 PHR-ALM 轨迹优化。MPC 直接跟踪 A* 路径。
4. **CasADi 仅 Python**：无 C++ CasADi，MPC 求解在 Python 中运行（~5ms/次，20Hz 够用）。
5. **seb_naver_ros2 无远程仓库**：仅在本地，未推送 GitHub。
6. **RViz SE2Grid 可视化**：高程图/风险图无法在 RViz 中直接显示（需要自定义 display 插件）。

### 8.2 待办 (TODO)

- [ ] 实现 PHR-ALM 轨迹优化 (Python CasADi)
- [ ] 增大 SDF 地图或实现分层规划以支持绕路
- [ ] forest/mountain 场景的完整导航验证
- [ ] seb_naver_ros2 推送到 GitHub
- [ ] RViz SE2Grid 可视化插件
- [ ] 真实机器人部署 (替换 MuJoCo sim 为真实 LiDAR/IMU)

---

## 9. 关键设计决策记录

| 决策 | 原因 |
|------|------|
| LiDAR time=0.001 (统一正值) | time=0 让 velodyne_handler 从 yaw 算假时间戳 → 运动畸变 → 漂移 |
| timestep=0.001 (非默认 0.002) | 0.002 转弯时接触力数值发散 → 速度爆到 70+ m/s |
| det_range=40 (非默认 300) | 300 导致 lasermap_fov_segment 每次删 150m 条带 → 地图清空 |
| 初始朝向 yaw=0 (非 face_goal) | 非 0 朝向导致 FAST-LIO2 camera_init 与 MuJoCo 世界系旋转偏差 → 导航方向错 |
| snap_to_ground + 2s 稳定期 | 不平地形初始悬空 → 自由落体 → IMU 初始化被零加速度污染 |
| wheel_odom.enable=true | 抑制崎岖地形 IMU 冲击 (z 峰值 22 m/s²) 导致的 SLAM 漂移 |
| MPC 控制平滑 (Δδ≤0.15/步) | 消除重规划导致的 ±δmax 极限转向横跳 → 物理甩动 → IMU 178 m/s² |
| SDF occupied=-1.0 (高点≥3) | 原 SDF 只有 free/unknown, A* 碰撞检测永远 True → 撞墙 |
| sim output='log' | headless 状态行每 2s 刷屏终端 |

---

## 10. 文件修改注意事项

### 10.1 修改后必须重新编译

```bash
# Python 包 (seb_naver_planner)
colcon build --packages-select seb_naver_planner

# C++ 包 (terrain_analyzer, fast_lio)
colcon build --packages-select terrain_analyzer  # ~20s
colcon build --packages-select fast_lio           # ~2-3min

# Python 脚本 (scorpio_mujoco_sim)
colcon build --packages-select scorpio_mujoco_sim  # ~1s

# 修改 launch/yaml 后也要重编译对应包
```

### 10.2 同步到上游工作空间

```bash
# scorpio_mujoco_sim 有 sync 脚本
cd /develop/scorpio_mujoco_sim_ws/src/scorpio_mujoco_sim
bash sync_to_ros2.sh  # rsync 到 /develop/scorpio_ros2/src/
```

### 10.3 Git 推送

```bash
# scorpio_mujoco_sim → GitHub
cd /develop/scorpio_mujoco_sim_ws/src/scorpio_mujoco_sim
no_proxy=github.com git push origin main  # 需要 no_proxy 绕过代理

# seb_naver_ros2 → 无远程 (本地 only)
```

---

## 11. 参考论文与算法对应

### 11.1 SEB-Naver 论文

**论文**: *SEB-Naver: A SE(2)-based Local Navigation Framework for Car-like Robots on Uneven Terrain*

- **作者**: Xiaoying Li, Long Xu, Xiaolin Huang 等 (浙江大学 ZJU-FAST-Lab)
- **发表**: arXiv:2503.02412v2, March 2025
- **PDF 位置**: `/develop/scorpio_ros2/doc/SEB-Naver A SE(2)-based Local Navigation Framework for Car-like Robots on Uneven Terrain.pdf`
- **原始源码**: https://github.com/ZJU-FAST-Lab/seb_naver (ROS1 + CUDA + CasADi C++)
- **本项目**: ROS2 + CPU + Python CasADi 移植

#### 论文核心贡献

1. **SE(2) 可通行性评估 (Algorithm 1)**: 在椭圆足迹内取高程点 → 3×3 协方差矩阵 → 最小特征向量 = 地形法线 → 曲率 κ = 3λ_min/Σλ → Risk = w·[κ/κ_max, φ_x/φ_max, φ_y/φ_max]。Risk=1 判定为显式障碍物。GPU 并行计算实现实时性。

2. **基于微分平坦的轨迹优化 (PHR-ALM)**: 引入中间变量 s(t) 消除非完整约束 ẋsinθ - ẏcosθ = 0 的奇异性。用五次分段多项式参数化轨迹，将运动学约束转化为不等式约束，通过 PHR (Penalty-Homotopy-Reformulation) + ALM (Augmented Lagrangian Method) 求解。

3. **统一框架 SEB-Naver**: LIO 定位 → SE(2) 局部高程图 (GPU) → 可通行性评估 → Kinodynamic A* 路径搜索 → PHR-ALM 轨迹优化 → MPC 跟踪控制。

#### 论文算法 → 本项目代码对应

| 论文模块 | 论文章节 | 本项目实现 | 文件 | 状态 |
|----------|----------|-----------|------|------|
| LiDAR-Inertial Odometry | Sec IV | FAST-LIO2 (修改版) | `FAST_LIO/src/laserMapping.cpp` | ✅ 已移植 |
| SE(2) 高程图构建 | Sec V-A, Eq.(4)-(9) | SE2Grid + Kalman 融合 | `se2_grid_core/SE2Grid.hpp`, `terrain_analyzer.cpp` | ✅ CPU 版 |
| 可通行性评估 (Algorithm 1) | Sec V-B, Alg.1 | 椭圆足迹 + 协方差 + 曲率 + Risk | `terrain_analyzer.cpp::computeTraversability()` | ⚠️ 已实现但 CPU 太慢, 当前禁用 |
| SDF 构建 | Sec V 末 | 高点统计障碍物检测 | `terrain_analyzer.cpp::timerCallback()` | ✅ 简化版 |
| Kinodynamic A* 路径搜索 | Sec VI-C | 自行车运动学 + Reeds-Shepp | `kino_astar.py` | ✅ 已移植 |
| PHR-ALM 轨迹优化 | Sec VI-B, Eq.(14)-(24) | — | — | ❌ 未实现 (直接用 A* 粗路径) |
| MPC 跟踪控制器 | Sec IV, Fig.3 | CasADi SQP, N=20, dt=0.05s | `mpc_node.py` | ✅ Python CasADi |
| SE(2) 栅格消息 | — | SE2Grid.msg, SE2GridInfo.msg | `se2_grid_msgs/msg/` | ✅ 自定义 |

#### 论文关键公式速查

```
地形姿态映射 F: SE(2) → R × S²₊
  p_B = [x, y, f₁(x,y,θ)]                    Eq.(1)
  y_b = f₂(x,y,θ) × x_yaw / ‖...‖             Eq.(2)
  x_b = y_b × f₂(x,y,θ)                       Eq.(3)

Kalman 高程融合:
  z_new = (σ²_p · z_m + σ²_m · z_p) / (σ²_p + σ²_m)
  σ²_new = (σ²_p · σ²_m) / (σ²_p + σ²_m)

Algorithm 1 可通行性:
  P ← FindEllipticalPoints(M, sr, ex, ey)
  Cov ← Σ(pj - p_mean)(pj - p_mean)ᵀ / |P|
  z_b, κ_ter ← GetMinEigenVecWithCurv(Cov)
  Risk = w · [κ/κ_max, φ_x/φ_max, φ_y/φ_max]ᵀ
  Risk = 1 → 显式障碍物

运动学 (不平地形):
  ṗ_B = vx · x_b                               Eq.(10)
  B Ṙ = B R⌊vx tanδ / Lw · z_b⌋               Eq.(11)

轨迹参数化 (五次分段多项式):
  xi(s) = cᵀ_xi · β(s),  β(s) = [1,s,s²,...,s⁵]ᵀ   Eq.(14)-(16)
  约束: ẋ²(s) + ẏ²(s) ≥ δ₊                        Eq.(24)
```

### 11.2 FAST-LIO2 论文

**论文**: *FAST-LIO2: Fast Direct LiDAR-Inertial Odometry*

- **作者**: Wei Xu, Yixi Cai, Dongqing He, Jiarong Lin, Fu Zhang (HKU-MARS)
- **PDF 位置**: `/develop/scorpio_mujoco_sim_ws/src/FAST_LIO/doc/Fast_LIO_2.pdf`
- **原始源码**: https://github.com/hku-mars/FAST_LIO
- **本项目修改**:
  - 添加 `wheel_odom_cbk`: 轮速里程计回调 + IMU 加速度校正 (置信度门控)
  - `det_range` 默认 300→40: 防止 `lasermap_fov_segment` 清空地图
  - `velodyne_handler` 适配: time=0.001 统一时间戳避免假去畸变
  - `wheel_odom.enable` 参数: launch 可控开关

### 11.3 其他参考

- **MuJoCo 文档**: https://mujoco.readthedocs.io/
- **CasADi 文档**: https://web.casadi.org/docs/
- **ROS2 Humble 文档**: https://docs.ros.org/en/humble/
- **Ackermann 运动学**: 自行车模型, 后轮驱动前轮转向, v = ω·R, δ = atan(ω·L/v)

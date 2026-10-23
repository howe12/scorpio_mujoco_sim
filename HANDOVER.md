# Scorpio MuJoCo 仿真 + SEB-Naver 自主导航系统 — 交接文档

> 本文档面向新接手本工程的开发者或 AI Agent，涵盖环境准备、三种运行模式、测试、调试和已知问题。
>
> **快速导航**
> - 只想跑起来 → [§2 环境准备](#2-环境准备) + [§3.1 模式总览](#31-模式总览)
> - 手动开车 → [§3.2 模式 A](#32-模式-a手动控制纯仿真无-ros2)
> - 键盘开车 + 建图 → [§3.3 模式 B](#33-模式-b手动建图键盘驾驶--fast-lio2-建图)
> - 点击导航 → [§3.4 模式 C](#34-模式-c自动导航rviz-点击目标全自动)
> - 遇到问题 → [§7 调试指南](#7-调试指南)
> - 想改参数 → [§5 关键参数](#5-关键参数说明)
> - 工程完成度 → [§1.3 完整性状态](#13-工程完整性状态)

---

## 1. 项目概述

本项目是 **NXROBO Scorpio 机器人**的 MuJoCo 物理仿真 + ROS2 自主导航系统，复现论文 *"SEB-Naver: A SE(2)-based Local Navigation Framework for Car-like Robots on Uneven Terrain"* 的核心算法。

**核心能力**：在 MuJoCo 仿真环境中，机器人通过 3D LiDAR + IMU 进行 SLAM 定位，构建局部高程图并检测障碍物，使用 Kinodynamic A* 规划路径，MPC 跟踪控制，实现 RViz SetGoal 点击式自主导航。

**三种使用模式**：手动控制（纯仿真验证）、手动建图（键盘驾驶 + SLAM）、自动导航（点击目标全自动）。详见 §3。

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

**单一 Git 仓库**: https://github.com/howe12/scorpio_mujoco_sim

```
scorpio_mujoco_sim/                      # Git 仓库根目录 (= colcon workspace src/scorpio_mujoco_sim)
├── scripts/                             # Python 仿真脚本
│   ├── run_seb_naver_sim.py             #   主入口: MuJoCo 仿真循环
│   ├── scorpio_ros2_publisher.py        #   ROS2 话题发布器
│   ├── generate_terrain.py              #   地形高度场生成
│   └── lidar_simulator.py               #   LiDAR 射线投射
├── models/                              # MuJoCo 模型
│   ├── scorpio.xml                      #   机器人物理模型 (Ackermann)
│   ├── scorpio_rviz.urdf                #   RViz 显示用 URDF
│   └── meshes/                          #   3D 网格文件
├── worlds/                              # 5 个仿真场景
│   ├── plaza.xml                        #   平地 + 墙壁 + 木箱
│   ├── forest.xml                       #   森林 (hfield + 树)
│   ├── mountain.xml                     #   山地
│   ├── pump_track.xml                   #   泵道
│   └── snowy_mountain.xml               #   雪山
├── config/
│   └── fastlio.rviz                     # RViz 配置
├── se2_grid_msgs/                       # ROS2 子包: SE(2) 栅格消息定义 (CMake)
├── se2_grid_core/                       # ROS2 子包: SE2Grid C++ 核心库 (header-only)
├── terrain_analyzer/                    # ROS2 子包: 高程图 + 障碍物检测 (C++)
├── seb_naver_planner/                   # ROS2 子包: A* + MPC + FSM (Python)
├── sync_to_ros2.sh                      # 同步脚本 (本仓库 → 上游 workspace)
├── HANDOVER.md                          # 本文档
└── README.md
```

**colcon 工作空间布局** (`/develop/scorpio_mujoco_sim_ws/`):

```
/develop/scorpio_mujoco_sim_ws/
├── src/
│   ├── scorpio_mujoco_sim/              # Git 仓库 (clone 在此)
│   ├── se2_grid_msgs/                   # rsync 副本 (由 sync_to_ros2.sh 生成)
│   ├── se2_grid_core/                   # rsync 副本
│   ├── terrain_analyzer/                # rsync 副本
│   ├── seb_naver_planner/               # rsync 副本
│   ├── FAST_LIO/                        # 本地 fork (无远程)
│   └── livox_ros_driver2/               # Livox 消息定义
├── install/                             # colcon build 输出
├── build/                               # colcon build 中间文件
└── clean_ros.sh                         # 残留进程清理
```

> **为什么需要 rsync 副本？** colcon 只扫描 `src/` 的**直接子目录**，不会递归进入 `scorpio_mujoco_sim/seb_naver_planner/`。所以 4 个 ROS2 子包必须在 `src/` 下有独立目录才能被 colcon 发现。`sync_to_ros2.sh` 会自动完成这个复制。
>
> **开发流程**：在 `src/scorpio_mujoco_sim/` 里改代码 → 运行 `bash sync_to_ros2.sh` → `colcon build`。

### 1.3 工程完整性状态

下表是本工程各模块的**实际状态**。状态含义：
- ✅ **可用** — 已实测通过
- ⚠️ **可用但有局限** — 能跑，已知边界
- ❌ **未实现** — 论文有、本工程没有

| 模块 | 文件 | 规模 | 状态 | 说明 |
|------|------|------|------|------|
| MuJoCo 物理仿真 | `models/scorpio.xml`, `worlds/*.xml` | 1.9k 行 XML | ✅ | Ackermann 车 + 5 场景，timestep=0.001 |
| 仿真主循环 | `scripts/run_seb_naver_sim.py` | ~600 行 | ✅ | 3 种模式（GUI/headless/benchmark） |
| 传感器发布 | `scripts/scorpio_ros2_publisher.py` | ~600 行 | ✅ | LiDAR/IMU/odom/wheel_odom（含打滑置信度） |
| LiDAR 射线仿真 | `scripts/lidar_simulator.py` + `run_seb_naver_sim.py` | — | ✅ | `mj_ray` 瞬时投射，360×32 线 |
| 地形生成 | `scripts/generate_terrain.py` | ~300 行 | ✅ | hfield + 树木布置 |
| FAST-LIO2 SLAM | `FAST_LIO/src/*` | ~2.7k 行 C++ | ✅ | 修改版：wheel_odom 校正、det_range=40 |
| SE(2) 栅格核心 | `se2_grid_core/` | ~2.2k 行 C++ | ✅ | 从 SEB-Naver 原版移植 |
| 高程图构建 | `terrain_analyzer/src/terrain_analyzer.cpp` | ~640 行 | ✅ | Kalman 融合 + 补洞 + 障碍物检测 |
| 可通行性评估 (Alg.1) | 同上 `computeTraversability()` | — | ⚠️ | 已实现但 CPU 320k 格太慢，**默认注释** |
| SDF 占用图 | 同上 | — | ⚠️ | 简化版：高点计数（非真实 signed distance） |
| Kinodynamic A* | `seb_naver_planner/kino_astar.py` | ~670 行 | ✅ | 自行车运动学 + Reeds-Shepp + SDF 碰撞 |
| ReplanFSM | `seb_naver_planner/planner_node.py` | ~530 行 | ✅ | 5 状态机 + 到达判定 + 定时重规划 |
| MPC 控制器 | `seb_naver_planner/mpc_node.py` | ~380 行 | ⚠️ | CasADi SQP，偶发迭代超限（有 fallback） |
| 轨迹优化 (P0+P1) | `seb_naver_planner/trajectory_optimizer.py` | ~800 行 | ✅ | 五次多项式最小 jerk + 运动学约束 + **SDF 避障**（距离场 + outside-in 排斥）；P0 5/5、P1 6/6 |
| PHR-ALM 完整版 (P2-P4) | 同上 | — | ⚠️ | 未并入 planner 主流程；地形姿态/Risk/换挡未做；弯道曲率有局限 |
| RViz 可视化 | `config/fastlio.rviz` | — | ✅ | 点云 + 机器人 + 路径 + SetGoal |
| SE2Grid 可视化 | — | — | ❌ | RViz 无自定义 display 插件，看不出高程图/风险图 |

**实测验证结论**（2026-10，plaza 场景）：

| 项目 | 结果 |
|------|------|
| 编译 | 7 个包全部通过 |
| 模式 A 手动控制 | ✅ GNU 窗口 + 键盘，MANUAL 状态正常，初始 yaw=0° |
| 模式 B 手动建图 | ✅ 静止地图 10.4 万点；移动 8s 后 15.6 万点；SLAM 位姿跟随 |
| 模式 C 自动导航 | ✅ 目标 (3,0) → 到达 (2.99,-0.04)，SLAM 偏差 0.1m，`Goal reached!` |
| TF 树 | ✅ `camera_init→body→base_footprint→base_link→…` 单一父节点 |
| 障碍物避让 | ⚠️ 能识别墙壁并停下（不撞），但目标在墙后时不绕路 |

**主要缺口**（详见 §8）：
1. 轨迹优化完成 P0（平滑+运动学）与 P1（SDF 避障），但**未并入 planner 主流程** → MPC 仍跟踪 A* 粗路径；地形姿态/Risk/换挡未做
2. 可通行性评估默认关闭 → SDF 只有障碍物二值，没有 slope/curvature 风险
3. 无全局地图/回环 → 长时间运行 SLAM 会缓慢漂移
4. 目标在障碍物后方时 A* 不绕路（20m 局部地图限制）

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

## 3. 三种运行模式

本工程有 **三种主要使用模式**，用途不同、启动的节点不同。先看对比表，再按需选择。

### 3.1 模式总览

| | **A. 手动控制** | **B. 手动建图** | **C. 自动导航** |
|---|---|---|---|
| **用途** | 验证物理/运动学、学键盘操作 | 人工驾驶同时建 3D 地图 | 点击目标，机器人自主导航 |
| **谁控制车** | 你（键盘） | 你（键盘） | MPC 控制器 |
| **启动命令** | `python3 run_seb_naver_sim.py` | `ros2 launch ... fastlio_mapping.launch.py` | `ros2 launch ... seb_naver_full.launch.py` |
| **需要 ROS2** | ❌ 不需要 | ✅ 需要 | ✅ 需要 |
| **MuJoCo 窗口** | ✅ 必须 | ✅ 必须（键盘驾驶） | 可选（`sim_gui:=true`） |
| **RViz** | ❌ | ✅ 建图可视化 | ✅ 地图+路径+机器人 |
| **启动节点数** | 1 | 4 | 7 |
| **SLAM** | ❌ 无 | ✅ FAST-LIO2 | ✅ FAST-LIO2 |
| **规划/控制** | ❌ 无 | ❌ 无 | ✅ A* + MPC |
| **典型用途** | 调物理参数 | 采集地图、验证 SLAM | 验证完整导航链路 |

> **共同前置**（每次新终端都要执行）：
> ```bash
> source /opt/ros/humble/setup.bash
> source /develop/scorpio_mujoco_sim_ws/install/setup.bash
> export DISPLAY=:0
> export XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority
> ```

---

### 3.2 模式 A：手动控制（纯仿真，无 ROS2）

**用途**：最快验证仿真环境是否正常、调物理参数、学习机器人操作手感。不涉及 SLAM 和导航。

```bash
cd /develop/scorpio_mujoco_sim_ws/src/scorpio_mujoco_sim
source /opt/ros/humble/setup.bash          # 仅为导入 rclpy 依赖
export DISPLAY=:0
export XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority

/usr/bin/python3 scripts/run_seb_naver_sim.py --terrain plaza
```

**会弹出**：一个 MuJoCo 3D 窗口（可直接拖拽旋转、滚轮缩放）。

**键盘操作**：

| 按键 | 功能 |
|------|------|
| `W` / `↑` | 加速前进（按住） |
| `S` / `↓` | 倒车（按住） |
| `A` / `←` | 左转（按住，松开自动回正） |
| `D` / `→` | 右转（按住，松开自动回正） |
| `Q` / `E` | 前进+左转 / 前进+右转 |
| `Z` / `C` | 倒车+左转 / 倒车+右转 |
| `Space` | 刹车（速度归零） |
| `R` | 复位（速度、转向归零） |
| `Tab` | 切换 手动 ↔ 内置自动巡航（`simple_controller`） |
| `ESC` | 退出 |

**屏幕上方的状态行**（每 0.5s 刷新）：
```
[MANUAL] t=  5.0s | v=0.49m/s(令+0.50) str= +5.2° | pos=(1.20,0.30) yaw=+14° | goal=13.2m
```
- `v` = 实际线速度，`令` = 指令值
- `str` = 前轮转角
- `yaw` = 车头朝向（应初始为 `-0°`，见 §9 坐标系决策）
- `goal` = 到场景内置 `goal_site` 的距离（仅显示，导航不用它）

**注意事项**：
- 这是**纯本地仿真**，不发布任何 ROS2 话题（除非加 `--ros2`）
- 退出时进程会 `os._exit(0)` 规避 NVIDIA GLFW 清理段错误，看到 `Killed` 属正常
- 若窗口不出现：检查 `DISPLAY` / `XAUTHORITY`，或改用 headless 模式验证

---

### 3.3 模式 B：手动建图（键盘驾驶 + FAST-LIO2 建图）

**用途**：人工开车，同时用 FAST-LIO2 建立 3D 点云地图，在 RViz 中实时观察建图质量。这是验证 SLAM 是否正常的主要方式。

```bash
source /opt/ros/humble/setup.bash
source /develop/scorpio_mujoco_sim_ws/install/setup.bash
export DISPLAY=:0
export XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority

# GUI 模式：MuJoCo 窗口（键盘驾驶） + RViz（看地图）
ros2 launch scorpio_mujoco_sim fastlio_mapping.launch.py terrain:=plaza sim_gui:=true

# headless 模式：无 MuJoCo 窗口，只发数据 + RViz（用 /cmd_vel 驱动）
ros2 launch scorpio_mujoco_sim fastlio_mapping.launch.py terrain:=plaza
```

**启动的 4 个节点**：

| 节点 | 作用 |
|------|------|
| `scorpio_mujoco_sim` | MuJoCo 物理仿真 + 发布 LiDAR/IMU/odom |
| `robot_state_publisher` | 从 URDF 发布 `base_footprint→base_link→...` 静态 TF |
| `fastlio_mapping` | FAST-LIO2，发布 `/Odometry` + `/cloud_registered` + `/Laser_map` |
| `static_transform_publisher` | 补 `body→base_footprint`（连接 SLAM 系与 URDF） |

**怎么操作**：
1. 弹出 MuJoCo 窗口 → 用键盘 `W/A/S/D` 开车
2. RViz 中观察：
   - **Laser Map**（`/Laser_map`）：累积的 3D 点云地图，应随行驶扩展
   - **Registered Cloud**（`/cloud_registered`）：当前帧配准点云
   - **Odometry**（`/Odometry`）：机器人位姿箭头
3. 地图应**闭合、无重影**。若出现重影/分层 → SLAM 漂移，见 §7 调试

**建图质量检查命令**：

```bash
# 地图点数（行驶后应持续增长）
ros2 topic echo /Laser_map --once | grep width

# SLAM 位姿（应与 /odom 真值接近）
ros2 topic echo /Odometry --once | grep -A2 "position:"
ros2 topic echo /odom --once | grep -A2 "position:"
```

**headless 模式下如何开车**（无键盘）：
```bash
# 用 cmd_vel 驱动
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4}, angular: {z: 0.1}}" --rate 20
```

**注意**：本模式**没有** planner/MPC，`Tab` 键的自动巡航只在模式 A（纯本地）生效。此模式下 `--no-tf-odom` 已由 launch 传入，不要手动去掉，否则 TF 冲突（见 §4.2）。

---

### 3.4 模式 C：自动导航（RViz 点击目标，全自动）

**用途**：验证完整导航链路（SLAM → 高程图/障碍物 → A* → MPC）。这是本工程的核心功能。

```bash
source /opt/ros/humble/setup.bash
source /develop/scorpio_mujoco_sim_ws/install/setup.bash
export DISPLAY=:0
export XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority

# 推荐：纯 RViz 控制（仿真在后台跑，无 MuJoCo 窗口）
ros2 launch seb_naver_planner seb_naver_full.launch.py terrain:=plaza

# 或：同时弹出 MuJoCo 窗口（看物理真相 + RViz 看算法视图）
ros2 launch seb_naver_planner seb_naver_full.launch.py terrain:=plaza sim_gui:=true
```

**启动的 7 个节点**：

| 节点 | 作用 | 关键输出 |
|------|------|----------|
| `scorpio_mujoco_sim` | 物理仿真 + 传感器 + 执行 cmd_vel | `/livox/lidar`, `/imu/data`, `/odom` |
| `robot_state_publisher` | URDF → TF | `base_footprint→base_link→lidar_link` |
| `fastlio_mapping` | 3D SLAM 定位 | `/Odometry`, `/cloud_registered` |
| `terrain_analyzer` | 高程图 + 障碍物 SDF | `/sdf_map`, `/fused_map` |
| `seb_naver_planner` | Kinodynamic A* + ReplanFSM | `/planner/trajectory` |
| `mpc_controller` | MPC 轨迹跟踪 | `/cmd_vel` |
| `static_transform_publisher` | `body→base_footprint` | — |

**完整操作流程**：

1. **等待就绪**（约 30-40 秒）。确认日志出现：
   ```bash
   # 挂到终端观察, 或看日志
   grep -E "SQP solver|Planner initialized|Initialize the map|Frame 30" ~/.ros/log/latest/*.log
   ```

2. **在 RViz 中选 SetGoal 工具**（工具栏绿色箭头图标，即 "2D Nav Goal"）

3. **在 RViz 3D 视图的地面上点击目标点**（可拖拽确定朝向）。RViz 会把点击位置转换成 `camera_init` 系坐标，发布到 `/goal_pose`。

4. **观察机器人**：
   - RViz 中机器人模型（URDF）沿 **Planner Path**（红色）行驶
   - **A* Rough Path**（蓝色）是搜索的原始路径
   - `/planner/status` 显示 FSM 状态：`SEQUENTIAL_START → EXEC_TRAJ ⇄ REPLAN_TRAJ → WAIT_TARGET`

5. **到达目标后**：planner 打印 `Goal reached!`，FSM 回到 `WAIT_TARGET`，可再次点击新目标。

**不用 RViz 点击也可以发目标**（适合脚本化测试）：
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'camera_init'}, pose: {position: {x: 3.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

**可选 launch 参数**：

| 参数 | 默认 | 说明 |
|------|------|------|
| `terrain` | `plaza` | `plaza` / `forest` / `mountain` / `pump_track` / `snowy_mountain` |
| `sim_gui` | `false` | `true` 弹出 MuJoCo 窗口 |
| `rviz` | `true` | `false` 不启动 RViz（纯 headless 跑算法） |
| `sim` | `true` | `false` 不启动仿真（接真实机器人时用） |

**导航成功判据**：
```bash
ros2 topic echo /odom --once | grep -A2 "position:"      # 接近目标点
ros2 topic echo /Odometry --once | grep -A2 "position:"  # SLAM 与真值偏差 < 1m
ros2 topic echo /planner/status --once                   # WAIT_TARGET (已到达)
```

---

### 3.5 辅助模式

**Headless 性能基准**（跑所有地形，不需要 ROS2/GUI）：
```bash
cd /develop/scorpio_mujoco_sim_ws/src/scorpio_mujoco_sim
python3 scripts/run_seb_naver_sim.py --headless
```

**仅地形成图**（不启动 planner/MPC，只看高程图 + SDF）：
```bash
ros2 launch terrain_analyzer terrain_mapping.launch.py terrain:=forest
```

**2D SLAM 建图**（历史方案，slam_toolbox + `/scan`，非当前导航链路）：
```bash
ros2 launch scorpio_mujoco_sim slam_mapping.launch.py terrain:=plaza
```
> 该方案用 2D 激光 + `odom→base_footprint` TF，**不用于** SEB-Naver 导航（导航用 FAST-LIO2 3D）。

---

### 3.6 launch 文件对照表

| launch 文件 | 包 | 节点 | 用途 |
|-------------|-----|------|------|
| `fastlio_mapping.launch.py` | `scorpio_mujoco_sim` | sim + rsp + fastlio + static_tf + rviz | **模式 B** 手动建图 |
| `seb_naver_full.launch.py` | `seb_naver_planner` | 上述 + terrain_analyzer + planner + mpc | **模式 C** 自动导航 |
| `terrain_mapping.launch.py` | `terrain_analyzer` | sim + rsp + fastlio + terrain_analyzer + static_tf + rviz | 仅地形成图 |
| `slam_mapping.launch.py` | `scorpio_mujoco_sim` | sim + slam_toolbox + rviz | 2D SLAM（历史） |
| `scorpio_mujoco.launch.py` | `scorpio_mujoco_sim` | sim bridge | 仅启动仿真桥 |

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

> ⚠️ **TF 冲突陷阱**：sim 的 `--no-tf-odom` 参数阻止发布 `odom→base_footprint`。若漏传，`base_footprint` 会有两个父节点（`odom` 和 `body`），RViz 小车模型显示失败、画面混乱，且**第一次导航可能正常、第二次出错**。
>
> **现状**：`fastlio_mapping.launch.py`、`terrain_mapping.launch.py`、`seb_naver_full.launch.py` 三个 launch 都已传 `--no-tf-odom` 并支持 `sim_gui` 参数。
>
> **例外**：`slam_mapping.launch.py`（2D slam_toolbox 方案）**故意不传** `--no-tf-odom`，因为它依赖 sim 的 `odom→base_footprint` 做 2D SLAM，且该 launch 没有 `body→base_footprint` 的 static TF，所以不冲突。

验证 TF 是否健康：
```bash
ros2 run tf2_tools view_frames 2>&1 | grep -oE "base_footprint: .{0,25}"
# 期望: base_footprint: \n  parent: 'body'      ← 正确
# 若为 'odom' → TF 冲突，检查是否漏传 --no-tf-odom
```

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

### 5.5 轨迹优化参数 (`TrajectoryOptimizer` 构造参数)

| 参数 | 默认 | 说明 |
|------|------|------|
| `wheel_base` | 0.315 | 轴距，用于 κ 上限 |
| `max_speed` | 0.5 | 速度上限 (m/s) |
| `max_accel` | 1.0 | 加速度上限 (m/s²) |
| `max_steer` | 0.785 | 转向上限；`κmax = tan(δmax)/L ≈ 3.17` |
| `kappa_margin` | 0.85 | 优化用 `0.85·κmax`，给 MPC 跟踪留余量 |
| `seg_len` | 1.2 | 每段目标弧长 (m)，决定分段数 N（3~10） |
| `rho_t` | 1.5 | 时间权重 ρt（论文 Eq.22），越大越快 |
| `follow_mode` | auto | auto: 无 SDF→硬约束; 有 SDF→软引导（让几何为避障让路） |
| `follow_path` | auto 结果 | 兼容旧参数，显式指定硬/软约束 |
| `path_guidance` | 4.0 | **有 SDF 时**的软引导权重（防切角） |
| `path_guidance_no_sdf` | 0.5 | **无 SDF 时**的软引导权重（小值让轨迹自由圆角，κ 才不超限） |
| `dmin` | 0.28 | 避障间距阈值 (m) = 车半宽 0.211 + 余量 |
| `sdf_iters` | 5 | outside-in 迭代轮数（找违反点→加排斥→重解） |
| `sdf_repulsion_weight` | 60.0 | 排斥代价权重 |
| `sdf_samples` | 10 | 每段 SDF 检查点数 |
| `sdf_tolerance` | 0.05 | 验收容差：min_clearance 允许比 dmin 小这么多 |
| `sdf_dmax` | 1.0 | 距离场截断 (m) |
| `constraint_samples` | 6 | 每段施加动力学约束的采样点数 |
| `quad_points` | 4 | jerk 积分的 Simpson 求积点数 |
| `tol` / `max_iter` | 1e-2 / 200 | IPOPT 容差与迭代上限 |
| `sample_dt` | 0.1 | 输出采样间隔 (s) |

> 求解用 IPOPT + `hessian_approximation=limited-memory`。实测：用精确 Hessian 需 1300ms，
> 改 L-BFGS 后降到 200~400ms，精度仍满足约束。

---

## 6. 测试方法

### 6.1 分模式冒烟测试（最快验证）

**模式 A 冒烟**（10 秒）：
```bash
cd /develop/scorpio_mujoco_sim_ws/src/scorpio_mujoco_sim
export DISPLAY=:0 XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority
timeout 10 /usr/bin/python3 scripts/run_seb_naver_sim.py --terrain plaza 2>&1 | grep -E "Keyboard|MANUAL" | head -3
# 期望: 打印键盘说明 + [MANUAL] 状态行, 弹出 MuJoCo 窗口
```

**模式 B 冒烟**（40 秒）：
```bash
source /opt/ros/humble/setup.bash && source /develop/scorpio_mujoco_sim_ws/install/setup.bash
ros2 launch scorpio_mujoco_sim fastlio_mapping.launch.py terrain:=plaza rviz:=false > /tmp/smoke_b.log 2>&1 &
sleep 40
grep -c "process started" /tmp/smoke_b.log          # 期望: 4
ros2 topic echo /Laser_map --once | grep width      # 期望: > 10000
ros2 run tf2_tools view_frames 2>&1 | grep -oE "base_footprint: .{0,20}"  # 期望 parent: 'body'
```

**模式 C 冒烟**（60 秒）：
```bash
ros2 launch seb_naver_planner seb_naver_full.launch.py terrain:=plaza rviz:=false > /tmp/smoke_c.log 2>&1 &
sleep 45
grep -c "process started" /tmp/smoke_c.log          # 期望: 7
ros2 topic echo /sdf_map --once | grep length_x     # 期望: 20.0
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'camera_init'}, pose: {position: {x: 3.0, y: 0.0}, orientation: {w: 1.0}}}"
sleep 18
ros2 topic echo /odom --once | grep -A2 "position:" # 期望 x≈3.0
grep "Goal reached" /tmp/smoke_c.log                # 期望有一行
```

> 每次测试前先 `/develop/scorpio_mujoco_sim_ws/clean_ros.sh`，避免残留进程干扰。

### 6.2 完整链路测试

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

### 6.3 避障测试

```bash
# plaza 场景有墙壁 wall_low_1 在 x=4.0
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'camera_init'}, pose: {position: {x: 6.0, y: 0.0}, orientation: {w: 1.0}}}"
sleep 20
ros2 topic echo /odom --once | grep -A2 "position:"
# 期望: x < 4.0 (停在墙前, 不撞墙)
```

### 6.4 多场景测试

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

### 6.5 单元测试 (无需 ROS2)

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
for f in seb_naver_planner/seb_naver_planner/*.py; do
    python3 -c "import py_compile; py_compile.compile('$f', doraise=True)" && echo "✅ $(basename $f)"
done
```

### 6.6 轨迹优化器测试（离线，无需 ROS2）

`TrajectoryOptimizer` 有 5 个场景的离线单测（直线 / L 弯 / S 弯 / U 形掉头 / 真实 A*），
校验收敛性、起终点精度、速度/曲率/加速度不超限、时间单调、航向连续。

```bash
cd /develop/scorpio_mujoco_sim_ws/src/scorpio_mujoco_sim/seb_naver_planner
source /opt/ros/humble/setup.bash
/usr/bin/python3 test/test_trajectory_optimizer.py

# 期望输出末段:
#   结果: 5/5 通过
```

P1 的 SDF 避障测试（6 个场景：贴墙/近墙/穿墙拒绝/窄通道/真实A*绕墙/无SDF回归）:

```bash
cd /develop/scorpio_mujoco_sim_ws/src/scorpio_mujoco_sim/seb_naver_planner
/usr/bin/python3 test/test_trajectory_optimizer_sdf.py
# 期望: 结果: 6/6 通过
```

单独调用:

```python
from seb_naver_planner.trajectory_optimizer import TrajectoryOptimizer

opt = TrajectoryOptimizer({'wheel_base': 0.315, 'max_speed': 0.5, 'max_steer': 0.785})
res = opt.optimize(astar_path, start_vel=0.3)     # astar_path: [(x, y, theta), ...]
if res.success:
    poses = [(x, y, th) for x, y, th, t in res.samples]   # 带时间的平滑轨迹
    print(f"Tf={res.duration:.2f}s  κ_max={res.max_kappa:.2f}  {res.solve_time_ms:.0f}ms")
else:
    poses = astar_path[:-1]                        # 回退到 A* 粗路径
```

---

## 7. 调试指南

### 7.1 常见问题速查

| 症状 | 原因 | 修复 |
|------|------|------|
| RViz 小车模型不显示 / 乱跳 | TF 冲突 (`base_footprint` 多父) | 三个 launch 都已传 `--no-tf-odom`；若手动启动 sim 必须加该参数；再运行 `clean_ros.sh` |
| SLAM 定位爆炸 (坐标千万级) | 残留进程发 cmd_vel 导致转弯失稳 | `clean_ros.sh` 后重启 |
| SLAM z 轴漂移 (悬空几米) | 不平地形初始悬空 → IMU 初始化污染 | 已修复 (snap_to_ground + 2s 稳定期) |
| 导航方向反 / 偏转 | 初始朝向非 0° → 坐标系偏差 | 已修复 (`reset_robot_facing_goal` 设 yaw=0) |
| 机器人不动 | `terrain_analyzer` 未启动 → 无 SDF → planner 等待 | `ros2 topic list \| grep sdf_map` |
| MPC solver failed | 短程转弯 SQP 迭代超限 | 已有 fallback (沿用上次控制衰减) |
| 撞墙 | **A* 碰撞检测采样间隔 0.33m 跨过薄墙**（wall_low_1 仅 0.2m 厚）；SDF 无障碍物是次级原因 | 已修复：① terrain_analyzer 高点统计标记 occupied ② **A* 采样改为 1 个栅格 (0.1m)**，见 §9 |
| 目标在墙后不动 | A* 20m 局部地图内找不到绕路 | 已知局限 (§8.1)，改发可绕行目标 |
| `No Effective Points` 持续 | `det_range` 太大 (默认 300) 清空地图 | 已修复 (det_range=40) |
| 转弯时速度爆炸 70+ m/s | timestep > 0.002 数值发散 | 已修复 (timestep=0.001) |
| MuJoCo 窗口不弹 | `sim_gui:=false` 或 DISPLAY 未设 | 加 `sim_gui:=true` 并检查 `DISPLAY`/`XAUTHORITY` |
| launch 报 "xacro/urdf not found" | 未 source install | `source install/setup.bash` |

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

1. **绕路导航**：目标在障碍物后方时，A* 在 20m 局部地图 + 5s 超时内找不到绕路路径，机器人会停在墙前。需要全球地图或分层规划（全局 + 局部）。实测：plaza 中发 (6,0)（墙在 x=4）→ 停在 x=3.69，不撞墙但也不绕路。
2. **长时间 SLAM 漂移**：FAST-LIO2 无回环检测，长时间运行后 z 轴有缓慢累积漂移（~0.1m/min）。论文本身就是局部导航不回环。
3. **轨迹优化未接入主流程**：`trajectory_optimizer.py` 已完成 P0（离线可用，5/5 场景通过），但 `planner_node.py` 仍把 A* 粗路径直接发给 MPC。接入需处理 200~400ms 求解对 50Hz FSM 定时器的阻塞（降频 / 异步 / 热启动），见 §8.2。
4. **可通行性评估默认关闭**：`computeTraversability()` 已实现但 CPU 版处理 320k SE(2) 格太慢，`terrain_analyzer.cpp` 中默认注释掉。因此 SDF 只有障碍物二值，**没有坡度/曲率风险**。
5. **SDF 是简化版**：不是真正的 signed distance field，而是「高点计数 → occupied」的占用图。
6. **CasADi 仅 Python**：无 C++ CasADi，MPC 求解在 Python 中运行（~5ms/次，20Hz 够用）。
7. **MPC 数值鲁棒性**：短程急转弯时偶发 `Maximum_Iterations_Exceeded`（已有 fallback：沿用上次控制 50% 衰减）。
8. **RViz SE2Grid 可视化**：高程图/风险图无法在 RViz 中直接显示（需要自定义 display 插件），只能通过 `/sdf_map` 原始消息查看。

### 8.2 待办 (TODO)

- [x] **轨迹优化 P0**：五次多项式最小 jerk + δ/v/a/κ 约束（5/5 场景，200~400ms）
- [x] **轨迹优化 P1**：SDF 避障（EDT 距离场 + outside-in 迭代排斥 + 端点过滤 + 输穿墙拒绝），6/6 场景
- [ ] **轨迹优化 P2**：并入 planner（降频/异步，`Path.header.stamp` 带时间）
- [ ] **轨迹优化 P2.5**：分段点改为优化变量（论文 P），根治弯道曲率局限
- [ ] **轨迹优化 P3**：订阅 `/fused_map` 加地形姿态 φx/φy 约束
- [ ] **轨迹优化 P4**：启用可通行性后加 Risk 项；可选 s(t)/换挡（论文完整版）
- [ ] 用 OpenMP/向量化加速 `computeTraversability()` 并重新启用（恢复坡度/曲率风险）
- [ ] 真实 SDF 构建（欧氏距离变换），替代高点计数
- [ ] 增大局部地图或加全局规划器以支持绕路
- [ ] forest / mountain / pump_track / snowy_mountain 场景的完整导航验证（目前主要验证 plaza）
- [ ] MPC 换 IPOPT 或加软约束以提升数值鲁棒性
- [ ] RViz SE2Grid 可视化插件
- [ ] 真实机器人部署（`sim:=false`，替换 MuJoCo 为真实 LiDAR/IMU）

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
| **所有 launch 统一 `sim_gui` + `--no-tf-odom`** | 早期 `fastlio_mapping.launch.py` 硬编码 GUI 且漏传 `--no-tf-odom` → TF 冲突。三个 launch 现参数一致 |
| **4 个 ROS2 包平铺仓库根目录** | colcon 只扫 `src/` 直接子目录，不递归；`sync_to_ros2.sh` 负责 rsync |
| **A* 碰撞采样按栅格密度** | 原 `check_num=3` 在 1.0m 弧长上间隔 0.33m，会跨过 0.2m 薄墙 → 规划穿墙 → 撞墙。改为 ≤1 个栅格 (0.1m) |
| **SDF 用 EDT 距离场而非原始占用值** | terrain_analyzer 发的 `sdf` 层是 -1/1/0.5 离散值，无梯度；IPOPT 拿不到下降方向。用 `distance_transform_edt` 重建欧氏距离 |
| **避障用 outside-in 迭代而非 NLP 内插值** | CasADi 3.8 的 `interpolant` 多维不可用（`n_in=1`/返回 `nan`）。改为「解→查违反点→加排斥代价→重解」 |
| **排斥跳过起终点** | 起终点位置是边界等式硬约束，推不动；对其施加排斥会导致 `Restoration_Failed` |
| **输入路径穿墙直接拒绝** | A* 负责避障、优化器负责平滑。强行「拉出来」不可行（`Maximum_Iterations_Exceeded`），应让上游重规划 |
| **分段策略维持等弧长** | 试过 4 种替代（DP 特征点 / 曲率加权 / 按航向加段 / 增加段数）全部让 κ 恶化到 4.7~553。弯道根治需把分段点变成优化变量（P2.5） |

---

## 10. 文件修改注意事项

### 10.1 修改后必须重新编译

```bash
# 先同步 (改的是仓库里的文件, workspace src/ 是副本)
cd /develop/scorpio_mujoco_sim_ws/src/scorpio_mujoco_sim
bash sync_to_ros2.sh

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
cd /develop/scorpio_mujoco_sim_ws/src/scorpio_mujoco_sim

# 同步到上游工作区 /develop/scorpio_ros2 (默认)
bash sync_to_ros2.sh

# 同步到本仓库所在工作区 (/develop/scorpio_mujoco_sim_ws/src/) ← 日常开发用这个
bash sync_to_ros2.sh --here

# 预览
bash sync_to_ros2.sh --here --dry-run
```

> ⚠️ **常见坑**：只跑 `sync_to_ros2.sh`（不带 `--here`）会把改动同步到
> `/develop/scorpio_ros2`，而**当前工作区 `/develop/scorpio_mujoco_sim_ws` 不更新**，
> 导致 `colcon build` 后新文件不存在。日常开发务必用 `--here`。

### 10.3 Git 推送

```bash
# 单仓库: 全部代码 (仿真 + 4 个 ROS2 包) 都在这里
cd /develop/scorpio_mujoco_sim_ws/src/scorpio_mujoco_sim
no_proxy=github.com git push origin main  # 需要 no_proxy 绕过代理

# 注意: FAST_LIO/ 和 livox_ros_driver2/ 在 workspace 的 src/ 下, 但不在本仓库内
#      它们是本地 fork, 不随本仓库推送
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
| 轨迹优化 | Sec VI-B, Eq.(14)-(29) | P0+P1: 五次多项式 + SDF + IPOPT | `seb_naver_planner/trajectory_optimizer.py` | ⚠️ P0/P1 完成，P2-P4 待做 |
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

# Scorpio MuJoCo 仿真 - ROS2 工作空间版

NXROBO Scorpio 机器人的 MuJoCo 仿真环境，支持**阿克曼转向**驱动、**YDLidar G6 激光雷达仿真**、**SLAM 建图**与键盘遥控。

---

## 目录

- [1. 工作空间结构](#1-工作空间结构)
- [2. 快速开始](#2-快速开始)
- [3. SLAM 建图](#3-slam-建图)
- [4. 键盘控制](#4-键盘控制)
- [5. 可用场景](#5-可用场景)
- [6. 传感器数据](#6-传感器数据)
- [7. 常见问题](#7-常见问题)

---

## 1. 工作空间结构

```
/develop/scorpio_mujoco_sim_ws/          # ROS2 工作空间
├── src/
│   └── scorpio_mujoco_sim/              # 功能包 (符号链接到独立仓库)
│       ├── models/                      # 机器人模型 + 网格
│       ├── worlds/                      # 场景文件
│       ├── scripts/                     # Python 脚本
│       ├── launch/                      # ROS2 launch 文件
│       ├── config/                      # 配置文件 (SLAM 等)
│       ├── package.xml                  # ROS2 包定义
│       └── CMakeLists.txt               # 构建配置
├── build/                               # 编译产物
├── install/                             # 安装产物
└── log/                                 # 编译日志
```

**特点**：
- 独立仓库 (`/develop/scorpio_mujoco_sim`) 用于开发和 Git 推送
- 工作空间 (`/develop/scorpio_mujoco_sim_ws`) 用于 ROS2 编译
- 通过符号链接保持同步

---

## 2. 快速开始

### 2.1 编译工作空间

```bash
cd /develop/scorpio_mujoco_sim_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 2.2 启动仿真

```bash
# 终端 1: 启动 MuJoCo 仿真
cd /develop/scorpio_mujoco_sim
python3 scripts/run_seb_naver_sim.py --terrain plaza

# 终端 2: 启动 SLAM
ros2 launch scorpio_mujoco_sim slam_mapping.launch.py terrain:=plaza
```

---

## 3. SLAM 建图

### 3.1 支持的 SLAM 方案

| 方案 | 特点 | 适用场景 |
|------|------|---------|
| **SLAM Toolbox** | 2D 占用栅格地图 | 平坦/轻微起伏场景 |
| **Cartographer** | 2D/3D SLAM，支持回环检测 | 复杂环境 |
| **FAST-LIO2** | 3D 点云地图，LiDAR-Inertial | 不平地形（论文方案） |

### 3.2 使用 SLAM Toolbox（默认）

```bash
# 启动仿真 + SLAM
ros2 launch scorpio_mujoco_sim slam_mapping.launch.py terrain:=plaza

# 在 MuJoCo 窗口中用键盘控制小车建图
# 保存地图
ros2 run nav2_map_server map_saver_cli -f /tmp/scorpio_map
```

### 3.3 查看地图

```bash
ros2 run rviz2 rviz2
# 添加显示:
#   /map (Map) - 占用栅格地图
#   /scan (LaserScan) - LiDAR 数据
#   /odom (Odometry) - 里程计
```

---

## 4. 键盘控制

| 按键 | 功能 | 说明 |
|------|------|------|
| **W** | 前进 | 按住加速 |
| **S** | 倒车 | 按住加速 |
| **A** | 左转 | 按住转向，松开自动回正 |
| **D** | 右转 | 按住转向，松开自动回正 |
| **Q** | 左前斜行 | 前进 + 左转 |
| **E** | 右前斜行 | 前进 + 右转 |
| **Z** | 左后斜行 | 倒车 + 左转 |
| **C** | 右后斜行 | 倒车 + 右转 |
| **Space** | 刹车 | 停止 |
| **R** | 回正 | 方向盘回正 + 停车 |
| **Tab** | 切换模式 | 手动 ↔ 自动导航 |
| **ESC** | 退出 | 关闭窗口 |

---

## 5. 可用场景

| 场景 | 命令 | 特点 |
|------|------|------|
| **室外广场** | `--terrain plaza` | 平坦 + 障碍物（木箱/柱子/锥桶） |
| **山地** | `--terrain mountain` | 崎岖起伏，最大坡度 45° |
| **森林** | `--terrain forest` | 缓和丘陵 + 树木障碍 |
| **雪山** | `--terrain snowy_mountain` | 陡峭斜坡，高落差 |
| **泵道** | `--terrain pump_track` | 连续波浪起伏 |

**示例**：
```bash
python3 scripts/run_seb_naver_sim.py --terrain mountain
ros2 launch scorpio_mujoco_sim slam_mapping.launch.py terrain:=mountain
```

---

## 6. 传感器数据

### 6.1 发布的 ROS2 话题

| 话题 | 类型 | 频率 | 说明 |
|------|------|------|------|
| `/scan` | LaserScan | 10 Hz | LiDAR 数据 (720 点, 0.12~16m) |
| `/odom` | Odometry | 50 Hz | 里程计 |
| `/imu/data` | Imu | 50 Hz | IMU (加速度 + 陀螺仪) |
| `/camera/rgb/image_raw` | Image | 30 Hz | RGB 图像 (640x480) |
| `/camera/depth/image_raw` | Image | 30 Hz | 深度图像 (640x480) |
| `/tf` | TF | 50 Hz | 坐标变换 |

### 6.2 坐标系

```
base_link (机器人中心)
    ├── laser_link (LiDAR, z=0.082m)
    └── camera_link (D435 相机, x=0.084m, z=0.651m)
```

---

## 7. 常见问题

### Q1: SLAM Toolbox 启动崩溃

**原因**：参数文件路径错误或 TF 变换缺失

**解决**：
```bash
# 检查 TF
ros2 run tf2_tools view_frames

# 检查话题
ros2 topic list | grep -E "scan|odom|tf"
```

### Q2: 建图不更新

**原因**：小车没有移动或 LiDAR 数据为空

**解决**：
```bash
# 检查 LiDAR 数据
ros2 topic echo /scan --once

# 检查 TF
ros2 run tf2_ros tf2_echo base_link laser_link
```

### Q3: 机器人无法爬坡

**原因**：力矩不足

**解决**：当前使用速度闭环控制 (±1.5 Nm)，可爬约 35° 坡。更陡的坡需要更高力矩或不同控制策略。

### Q4: 键盘控制不响应

**原因**：焦点窗口不对

**解决**：
- 使用 `pynput` 全局键盘监听，任何窗口都能控制
- 确保 MuJoCo 窗口没有最小化

---

## 附录

### A. 工作空间管理

```bash
# 重新编译
cd /develop/scorpio_mujoco_sim_ws
colcon build

# 清理编译产物
rm -rf build/ install/ log/

# 只编译本包
colcon build --packages-select scorpio_mujoco_sim
```

### B. 同步独立仓库

```bash
cd /develop/scorpio_mujoco_sim
git add -A
git commit -m "描述"
git push
```

### C. 重新同步到工作空间

```bash
# 符号链接已存在，无需手动同步
# 如果需要刷新：
rm /develop/scorpio_mujoco_sim_ws/src/scorpio_mujoco_sim
ln -s /develop/scorpio_mujoco_sim /develop/scorpio_mujoco_sim_ws/src/scorpio_mujoco_sim
```

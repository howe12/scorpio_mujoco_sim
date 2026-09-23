# Scorpio MuJoCo 仿真

> **首次使用请先阅读 [DEPLOY.md](DEPLOY.md)** — 完整的环境部署与排查指南。
>
> **需要与 scorpio_ros2 工作区协作时看 [WORKFLOW.md](WORKFLOW.md)** — 两份副本的同步与合并流程。
>
> 快速安装: `pip install -r requirements.txt`，然后配置 X11 (`export XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority`)

NXROBO Scorpio 机器人的 MuJoCo 仿真环境，支持**阿克曼转向**驱动、**YDLidar G6 激光雷达仿真**、键盘遥控与多场景切换。

---

## 目录

- [1. 版本状态](#1-版本状态)
- [2. 环境准备](#2-环境准备)
- [3. 快速开始](#3-快速开始)
- [4. 键盘控制](#4-键盘控制)
- [5. 内置场景](#5-内置场景)
- [6. 传感器仿真](#6-传感器仿真)
- [7. 如何设计环境](#7-如何设计环境)
- [8. 文件结构](#8-文件结构)
- [9. 常见问题](#9-常见问题)

---

## 1. 版本状态

**当前版本**: Scorpio MuJoCo 仿真 v2.0  
**最后更新**: 2026-09-23

### 已完成
- ✅ 阿克曼转向驱动 (后轮驱动 + 前轮转向)
- ✅ 所有部件 URDF 精确坐标 (误差 0.0mm)
- ✅ D435 深度相机 (RGB + Depth 渲染)
- ✅ YDLidar G6 激光雷达仿真 (360°, 720点)
- ✅ 转向执行器 (kp=20, kv=4, 45°可达 43°)
- ✅ 碰撞掩码 (机器人内部互不碰撞)
- ✅ 5 个内置场景 (plaza + 4个地形)
- ✅ 键盘控制 (W/A/S/D + Space + Tab)
- ✅ 自动导航 (Tab 切换)
- ✅ 浏览器可视化 (EGL + HTTP)

### 关键参数
| 参数 | 值 | 说明 |
|------|-----|------|
| 驱动方式 | 阿克曼 | 后轮驱动 + 前轮转向 |
| 后轮控制 | 速度闭环 | ctrl 单位 rad/s (v = ω×0.0525) |
| 力矩上限 | ±1.5 Nm | 可爬约 35° 坡 |
| 轮子位置 | x=±0.1575, y=±0.0928, z=0.0525 | URDF 坐标 |
| D435 安装 | (0.1706, 0.0175, 0.242) | camera_link |
| LiDAR 安装 | (0.10, 0, 0.35) | 车顶上方 |
| 转向增益 | kp=20, kv=4 | 45° 指令可达 43° |
| 最大车速 | 1.575 m/s | ctrlrange ±30 rad/s |
| 碰撞掩码 | robot: ct=1/ca=4, ground: ct=1/ca=7 | 位掩码排除内部碰撞 |

---

## 2. 环境准备

### 2.1 依赖
```bash
pip install mujoco numpy Pillow
```

### 2.2 显示环境配置（重要！）
```bash
# 每次打开新终端时执行，或写入 ~/.bashrc
export XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority
export DISPLAY=:0
```

### 2.3 GLX/段错误说明
**现象**: MuJoCo 窗口能弹出、正常运行，但进程退出时段错误 (exit 139)。

**原因**: MuJoCo 3.10 + NVIDIA 580 驱动下，Python 退出时 GLX 上下文清理顺序错误。

**修复**: 已在所有运行脚本末尾加 `os._exit(0)` 跳过清理阶段。无需手动处理。

---

## 3. 快速开始

### 3.1 进入仿真目录
```bash
cd /develop/scorpio_ros2/src/scorpio/scorpio_mujoco_sim
```

### 3.2 启动仿真 (以 plaza 广场为例)
```bash
python3 scripts/run_seb_naver_sim.py --terrain plaza
```

### 3.3 指定场景
```bash
python3 scripts/run_seb_naver_sim.py --terrain mountain
python3 scripts/run_seb_naver_sim.py --terrain forest
python3 scripts/run_seb_naver_sim.py --terrain snowy_mountain
python3 scripts/run_seb_naver_sim.py --terrain pump_track
```

### 3.4 无头基准测试
```bash
python3 scripts/run_seb_naver_sim.py --headless
```

---

## 4. 键盘控制

启动后默认进入**手动键盘模式**：

| 按键 | 功能 | 说明 |
|------|------|------|
| **W / ↑** | 加速前进 | 每按一次增加油门，可连续按 |
| **S / ↓** | 倒车 / 减速 | 每按一次减少油门，可连续按 |
| **A / ←** | 左转 | 转向角 +8.6° |
| **D / →** | 右转 | 转向角 -8.6° |
| **Space** | 刹车 | 油门归零 |
| **R** | 方向盘回正 | 转向角归零 + 刹车 |
| **Tab** | 切换模式 | 手动 ↔ 自动导航 |
| **ESC** | 退出 | 关闭窗口 |

**自动导航模式**（按 Tab 切换）：机器人自动驶向场景中的**绿色目标点**，带简单避障。

**状态栏显示**（窗口底部每 0.5s 更新）：
```
[MANUAL] t=5.2s | spd=3.0m/s str=+8.6° | pos=(2.1,1.5) yaw=15° | goal=18.3m
│          │            │                   │                     │
模式       时间      油门/转向角          当前位置              离目标距离
```

---

## 5. 内置场景

### 5.1 室外广场 (plaza)
平坦地面 + 障碍物 (木箱/柱子/锥桶)，适合基础避障测试。

### 5.2 山地 (mountain)
崎岖起伏地形，最大坡度 45°，论文复现场景。

### 5.3 森林 (forest)
缓和丘陵 + 30 棵树木障碍。

### 5.4 雪山 (snowy_mountain)
陡峭斜坡，高落差，需要谨慎驾驶。

### 5.5 泵道 (pump_track)
连续波浪起伏，姿态控制测试。

---

## 6. 传感器仿真

### 6.1 YDLidar G6 激光雷达
**型号规格** (NXROBO 官方 + ydlidar_g6.yaml):
- 型号: YDLidar G6 (EAI)
- 外形: 82.8 × 75.4 × 42.9 mm (实际网格)
- 量程: 0.12 ~ 16.0 m
- 扫描角: 360°
- 角度分辨率: 0.5° (720 点)
- 扫描频率: 12 Hz
- 安装: 车体顶部平台, 底座 z=0.250, 扫描中心 z=0.282

**使用**:
```python
from lidar_simulator import LiDARSimulator

lidar = LiDARSimulator(model, data)
ranges = lidar.scan()  # 返回 (720,) 距离数组, 单位米, 无命中为 inf

# ASCII 可视化
print(lidar.visualize_ascii())

# ROS LaserScan 格式
ros_msg = lidar.to_ros_laserscan()
```

**过滤机器人自身遮挡** (车体后部结构会遮挡 ±147° 方向):
```python
# 方式一: 自动检测遮挡角度
ignore = lidar.auto_ignore_array(threshold=0.35)
# 返回: "-150.5,-145.5,145.5,150.5"

# 方式二: 手动指定 (格式与真实 ROS 驱动 ignore_array 一致)
lidar = LiDARSimulator(model, data, ignore_array="-150,-145,145,150")
clean = lidar.scan()   # 700 点, 无自遮挡

# 方式三: 完全排除机器人本体
lidar = LiDARSimulator(model, data, exclude_robot=True)
clean = lidar.scan()   # 纯净环境扫描
```

### 6.2 D435 深度相机
**特性**:
- RGB: 640×480, 42° FOV
- Depth: 640×480, 64° FOV
- 无机器人自身遮挡

**使用**:
```python
import mujoco
renderer = mujoco.Renderer(model, 480, 640)

# RGB
renderer.update_scene(data, camera=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'd435_rgb'))
rgb = renderer.render()  # (480, 640, 3) uint8

# Depth
renderer.enable_depth_rendering()
renderer.update_scene(data, camera=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'd435_depth'))
depth = renderer.render()  # (480, 640) float32, 单位米
renderer.disable_depth_rendering()
```

### 6.3 其他传感器
- **IMU**: 6 轴 (加速度 + 陀螺仪)
- **轮编码器**: 4 个轮子的位置和速度
- **转向角**: 前轮转向角度
- **里程计**: 基座位姿 (ground truth)
- **目标距离**: 到目标点的距离

---

## 7. 如何设计环境

### 7.1 平坦场景 (手写 MJCF)
复制 `worlds/plaza.xml` 为模板，修改障碍物：

```xml
<!-- 固定墙壁 -->
<body name="wall1" pos="8 0 0.5">
  <geom type="box" size="0.1 5 0.5" rgba="0.5 0.5 0.5 1"/>
</body>

<!-- 可推动的箱子 -->
<body name="box1" pos="5 3 0.25">
  <freejoint/>
  <geom type="box" size="0.3 0.3 0.25" rgba="0.6 0.4 0.2 1" mass="2"/>
</body>

<!-- 固定柱子 -->
<body name="pillar" pos="7 -2 0.5">
  <geom type="cylinder" size="0.15 0.5" rgba="0.8 0.2 0.1 1"/>
</body>
```

### 7.2 不平地形场景 (程序化生成)
编辑 `scripts/generate_terrain.py` 中的 `generate_heightmap()` 函数：

```python
elif terrain_type == "my_terrain":
    # 自定义高度公式
    h = 0.8 * np.sin(2*np.pi*X/6.0) * np.cos(2*np.pi*Y/8.0)
    h += perlin_like_noise(X, Y, octaves=3, persistence=0.5, scale=0.2) * 0.3
    h = np.clip(h, 0, 2.0)
```

### 7.3 爬坡动力与控制方式

**后轮使用速度闭环控制**（MuJoCo `velocity` actuator），而非力矩开环。

**为什么**：力矩开环时，坡道上的重力分力会直接抵消电机力矩，车速骤降甚至爬不动。
速度闭环会自动增大力矩维持目标速度，与真实机器人驱动器行为一致。

**实测对比**（命令 0.5 m/s）：

| 场景 | 力矩开环 0.5 Nm | 速度闭环 ±1.5 Nm |
|------|----------------|-----------------|
| mountain | 0.809 m/s | 0.460 m/s（= 命令值） |
| forest | 0.449 m/s | 0.470 m/s |
| snowy_mountain | **0.062 m/s（爬不动）** | 0.628 m/s |
| pump_track | **0.114 m/s（爬不动）** | 0.468 m/s |

**力矩需求参考**（整车 5.33 kg，轮半径 0.0525 m，后轮 2 驱）：

| 坡度 | 每轮所需力矩 |
|------|-------------|
| 10° | 0.238 Nm |
| 20° | 0.470 Nm |
| 30° | 0.687 Nm |
| 35° | 0.788 Nm |
| 40° | 0.883 Nm |

当前 ±1.5 Nm 上限对应约 35° 静态爬坡能力（含 2 倍余量）。
更陡的坡度会受**牵引力**和**重心**限制（轴距 0.315 m，重心高约 0.19 m，静态倾覆角约 40°）。

### 7.4 调整机器人参数
编辑 `models/scorpio.xml`：

| 参数 | 当前值 | 说明 |
|------|--------|------|
| 转向增益 | kp=20, kv=4 | 改大转向更灵敏 |
| 后轮力矩 | ±0.5 Nm | 改大加速更快 |
| 轮胎摩擦 | 1.2 | 改大抓地力强 |
| 转向范围 | ±45° | 改大转弯半径更小 |

---

## 8. 文件结构

```
scorpio_mujoco_sim/
├── models/
│   ├── scorpio.xml              # 机器人主模型 (阿克曼转向)
│   ├── meshes/                  # 外观网格 (body, wheels, d435)
│   └── terrains/                # 地形网格 (.obj)
├── worlds/                      # 场景文件
│   ├── plaza.xml                # 室外广场 (平坦)
│   ├── mountain.xml             # 山地 (生成)
│   ├── forest.xml               # 森林 (生成)
│   ├── snowy_mountain.xml       # 雪山 (生成)
│   └── pump_track.xml           # 泵道 (生成)
├── scripts/
│   ├── run_seb_naver_sim.py     # 主运行脚本 (键盘控制+自动导航)
│   ├── generate_terrain.py      # 地形生成器
│   ├── lidar_simulator.py       # LiDAR 仿真模块 (新增)
│   ├── scorpio_mujoco_bridge.py # ROS2 桥接
│   ├── scorpio_rl_env.py        # RL 训练环境
│   └── view_sim_web.py          # 浏览器可视化
├── MODEL_STATUS.md              # 模型状态记录
└── README.md                    # 本文档
```

---

## 9. 常见问题

### Q1: 窗口能弹出但退出时段错误
正常现象，见 [2.3 GLX/段错误说明](#23-glx段错误说明)。脚本已内置 `os._exit(0)` 修复。

### Q2: 报错 `GLXBadContext`
```bash
export XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority
export DISPLAY=:0
```

### Q3: 机器人掉穿地面 / 不爬坡
旧版已修复（法线方向 bug）。重新生成地形：
```bash
python3 scripts/generate_terrain.py
```

### Q4: 障碍物几何体中有一个透明方块
正常。`body_collision` 是 `rgba="1 1 1 0"`（alpha=0）的碰撞体，物理上存在但不可见，是机器人底盘的碰撞壳。

### Q5: 怎么改目标点位置
编辑对应 world XML 中 `goal_marker` 的 `pos`：
```xml
<body name="goal_marker" pos="X Y Z">
  <site name="goal_site" pos="0 0 0"/>
</body>
```

### Q6: LiDAR 扫描结果不正确
确保 LiDAR 位置在车顶以上 (z=0.35)。检查 `lidar_site` 的 `euler` 设置为 `0 0 0`（水平扫描）。

### Q7: D435 相机画面被机器人遮挡
已修复。D435 相机光心置于前端玻璃处 (pos="0.005 -0.0175 0")，视野无遮挡。

### Q8: 转向只有 11° 左右
旧版已修复。转向执行器增益已调为 kp=20, kv=4，45° 指令可达 43°。

---

## 10. 运行命令速查

```bash
# 设置显示环境
export XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority
export DISPLAY=:0

cd /develop/scorpio_ros2/src/scorpio/scorpio_mujoco_sim

# 1. 室外广场 (键盘控制)
python3 scripts/run_seb_naver_sim.py --terrain plaza

# 2. 其他场景
python3 scripts/run_seb_naver_sim.py --terrain mountain
python3 scripts/run_seb_naver_sim.py --terrain forest
python3 scripts/run_seb_naver_sim.py --terrain snowy_mountain
python3 scripts/run_seb_naver_sim.py --terrain pump_track

# 3. 无头基准测试
python3 scripts/run_seb_naver_sim.py --headless

# 4. 浏览器可视化 (远程/无显示器)
MUJOCO_GL=egl python3 scripts/view_sim_web.py --terrain plaza --port 8080

# 5. LiDAR 仿真测试
python3 scripts/lidar_simulator.py

# 6. 重新生成地形
python3 scripts/generate_terrain.py
```

---

## 11. 开发记录

### v2.0 更新 (2026-09-23)
- 修复雷达位置: z=0.08 (车体内部) → z=0.35 (车顶上方)
- 添加完整 LiDAR 仿真模块: `lidar_simulator.py`
- 360° 水平扫描, 720 点, 0.12-12.0m 范围
- ASCII 可视化 + ROS LaserScan 格式
- 更新所有场景的雷达位置

### v1.5 更新 (2026-09-23)
- 修复部件分散问题 (MuJoCo mesh 重定向)
- 修正 D435 相机位置 (0.1706, 0.0175, 0.242)
- 修正相机朝向 (quat 0.5 0.5 0.5 0.5)
- 修复转向卡在 11° 问题 (kp=20, kv=4)
- 碰撞掩码排除内部碰撞 (contype=1, conaffinity=4)

### v1.0 初始版本
- 阿克曼转向驱动
- 5 个内置场景
- 键盘控制 + 自动导航
- D435 相机仿真

---

## 资源来源与许可

### 本项目包含
- MuJoCo 模型 (`models/`)、场景 (`worlds/`)、仿真脚本 (`scripts/`)
- 网格文件 (`models/meshes/*.obj`, `models/terrains/*.obj`)
- 地形高度数据 (`models/terrains/*_hfield.bin`)

### 派生资源说明
`models/meshes/` 中的网格由以下**上游开源工程**转换而来（STL/DAE → OBJ + 简化）：

| 文件 | 来源 | 原格式 |
|------|------|--------|
| `scorpio.obj` | NXROBO scorpio_ros2 · `scorpio_description/meshes/scorpio/base/scorpio.stl` | STL (19MB) |
| `left_wheel.obj` / `right_wheel.obj` | 同上 · `left_wheel.stl` / `right_wheel.stl` | STL |
| `d435.obj` | Intel RealSense ROS · `realsense2_description/meshes/d435.dae` | DAE (16MB) |
| `ydlidar_g6.obj` | YDLidar ROS2 Driver · `ydlidar_ros2_driver/meshes/ydlidar.dae` | DAE (3MB) |

转换时仅做了网格简化（减少面数），**未改变外形尺寸**。

### 机器人参数来源
- 部件坐标: `NXROBO/scorpio_ros2` 的 URDF (`scorpio_description`)
- 雷达规格: NXROBO 官网 Scorpio 产品页 + `ydlidar_g6.yaml`
- 相机挂载: `realsense2_description/urdf/scorpio_d435_camera.urdf.xacro`

### 许可
本仿真工程: Apache-2.0

上游资源版权归各自所有者：
- NXROBO Scorpio — © NXROBO INTERNATIONAL (HONG KONG) LIMITED
- Intel RealSense — © Intel Corporation (Apache-2.0)
- YDLidar — © EAI / 深圳乐动机器人

使用前请确认符合各上游项目的许可条款。

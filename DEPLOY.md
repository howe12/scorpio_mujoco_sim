# 环境部署说明

Scorpio MuJoCo 仿真环境的完整部署指南。

---

## 目录

- [1. 系统要求](#1-系统要求)
- [2. 快速部署](#2-快速部署)
- [3. 显示环境配置](#3-显示环境配置)
- [4. 安装验证](#4-安装验证)
- [5. 首次运行](#5-首次运行)
- [6. 可选: ROS2 集成](#6-可选-ros2-集成)
- [7. 无显示器环境](#7-无显示器环境)
- [8. 常见问题排查](#8-常见问题排查)
- [9. 卸载](#9-卸载)

---

## 1. 系统要求

### 必需
| 项目 | 要求 | 说明 |
|------|------|------|
| 操作系统 | Ubuntu 20.04 / 22.04 (或兼容 Linux) | 已在 Ubuntu 22.04 验证 |
| Python | 3.8 ~ 3.11 | 已在 3.10 验证 |
| 显示环境 | X11 (物理显示器或 Xvfb) | MuJoCo 窗口需要 |
| GPU | 支持 OpenGL 3.1+ | NVIDIA/AMD/Intel 均可 |

### 可选
| 项目 | 用途 |
|------|------|
| ROS2 Humble | 与导航栈集成 (`scripts/scorpio_mujoco_bridge.py`) |
| NVIDIA GPU + CUDA | 更快的渲染 |

---

## 2. 快速部署

### 2.1 获取代码

```bash
git clone <你的仓库地址> scorpio_mujoco_sim
cd scorpio_mujoco_sim
```

### 2.2 创建 Python 虚拟环境（推荐）

```bash
# 使用 venv
python3 -m venv venv
source venv/bin/activate

# 或使用 conda
conda create -n scorpio_sim python=3.10 -y
conda activate scorpio_sim
```

### 2.3 安装依赖

```bash
pip install -r requirements.txt
```

或手动安装：

```bash
pip install mujoco>=3.0 numpy Pillow pynput
```

**各依赖用途**：
| 包 | 版本 | 用途 |
|-----|------|------|
| `mujoco` | >= 3.0 | 物理仿真引擎（必需） |
| `numpy` | >= 1.20 | 数值计算（必需） |
| `Pillow` | 任意 | 图像编码（浏览器可视化用） |
| `pynput` | 任意 | 全局键盘监听（键盘控制用） |
| `trimesh` | 任意 | 网格转换（仅开发时需要） |

### 2.4 一键验证安装

```bash
python3 scripts/lidar_simulator.py
```

若输出 LiDAR 扫描结果和 ASCII 可视化图，说明安装成功。

---

## 3. 显示环境配置

> **这是最容易出问题的一步，请务必完成。**

MuJoCo 的窗口需要正确的 X11 认证。很多情况下 `DISPLAY=:0` 已设置，但 `XAUTHORITY` 指向错误的文件，导致报错或闪退。

### 3.1 找到正确的 Xauthority

```bash
# 查看当前 Xorg 进程使用的认证文件
ps aux | grep [X]org | grep -o '\-auth [^ ]*'
```

典型输出：`-auth /run/user/1000/gdm/Xauthority`

### 3.2 设置环境变量

```bash
export XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority
export DISPLAY=:0
```

**写入 `~/.bashrc` 避免每次设置**：

```bash
cat >> ~/.bashrc << 'EOF'

# Scorpio MuJoCo 仿真显示配置
export XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority
export DISPLAY=:0
EOF

source ~/.bashrc
```

### 3.3 验证显示可用

```bash
# 应能看到显示器信息
xdpyinfo -display :0 | head -3
```

### 3.4 GLX 段错误说明

**现象**：MuJoCo 窗口能弹出、正常运行，但**进程退出时**段错误（exit code 139）。

**原因**：MuJoCo 3.10 + NVIDIA 580 驱动下，Python 退出时 GLX 上下文清理顺序问题。

**处理**：本项目的所有运行脚本末尾已加 `os._exit(0)` 跳过该阶段，**无需额外处理**。

若你自己写脚本调用 MuJoCo viewer，末尾加上：

```python
import os
viewer.close()
os._exit(0)   # 避免 GLX 清理崩溃
```

---

## 4. 安装验证

逐项验证，确保环境正常。

### 4.1 验证 MuJoCo 基础

```bash
python3 -c "
import mujoco
print('MuJoCo:', mujoco.__version__)
m = mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom size=\"0.1\"/></worldbody></mujoco>')
print('模型加载: OK')
"
```

### 4.2 验证模型文件完整

```bash
python3 -c "
import mujoco, os
files = ['models/scorpio.xml'] + ['worlds/'+f for f in sorted(os.listdir('worlds'))]
for f in files:
    m = mujoco.MjModel.from_xml_path(f)
    print(f'  OK  {f:34s} {m.nbody:3d} bodies, {m.nsensor} sensors')
" 2>&1 | grep -v "Loaded heightfield"
```

预期输出 6 行，全部 `OK`。

### 4.3 验证传感器

```bash
python3 -c "
import sys, mujoco, numpy as np
sys.path.insert(0,'scripts')
from lidar_simulator import LiDARSimulator
m = mujoco.MjModel.from_xml_path('models/scorpio.xml')
d = mujoco.MjData(m); mujoco.mj_forward(m,d)
lidar = LiDARSimulator(m,d)
r = lidar.scan()
print(f'LiDAR: {len(r)} 点, 扫描中心 {d.site_xpos[lidar.site_id].round(3)}')
print(f'相机: {m.ncam} 个 (d435_rgb, d435_depth)')
print(f'传感器: {m.nsensor} 个')
"
```

### 4.4 验证窗口可弹出

```bash
python3 -c "
import os, mujoco, mujoco.viewer, time
m = mujoco.MjModel.from_xml_path('models/scorpio.xml')
d = mujoco.MjData(m)
v = mujoco.viewer.launch_passive(m, d)
print('窗口已弹出, 5秒后关闭...')
t0 = time.time()
while v.is_running() and time.time()-t0 < 5:
    mujoco.mj_step(m, d); v.sync()
v.close(); os._exit(0)
"
```

应能看到一个简单的仿真窗口。

---

## 5. 首次运行

```bash
cd scorpio_mujoco_sim

# 运行广场场景 (键盘控制)
python3 scripts/run_seb_naver_sim.py --terrain plaza
```

**键盘操作**：

| 按键 | 功能 |
|------|------|
| W / ↑ | 加速前进 |
| S / ↓ | 倒车 / 减速 |
| A / ← | 左转 |
| D / → | 右转 |
| Space | 刹车 |
| R | 方向盘回正 |
| Tab | 切换 手动 ↔ 自动导航 |
| ESC | 退出 |

**其他场景**：

```bash
python3 scripts/run_seb_naver_sim.py --terrain mountain        # 山地
python3 scripts/run_seb_naver_sim.py --terrain forest          # 森林
python3 scripts/run_seb_naver_sim.py --terrain snowy_mountain  # 雪山
python3 scripts/run_seb_naver_sim.py --terrain pump_track      # 泵道
```

**无头测试**（不需要显示器）：

```bash
python3 scripts/run_seb_naver_sim.py --headless
```

---

## 6. 可选: ROS2 集成

仅在需要与 ROS2 导航栈集成时安装。

### 6.1 安装 ROS2 Humble

```bash
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash
```

### 6.2 将本包放入 ROS2 工作区

```bash
mkdir -p ~/scorpio_ws/src
cp -r scorpio_mujoco_sim ~/scorpio_ws/src/
cd ~/scorpio_ws
colcon build --packages-select scorpio_mujoco_sim
source install/setup.bash
```

### 6.3 启动 ROS2-MuJoCo 桥接

```bash
ros2 launch scorpio_mujoco_sim scorpio_mujoco.launch.py
```

**发布的话题**：

| 话题 | 类型 | 说明 |
|------|------|------|
| `/cmd_vel` | Twist | 订阅：速度指令 |
| `/odom` | Odometry | 发布：里程计 |
| `/scan` | LaserScan | 发布：LiDAR 扫描 |
| `/imu/data` | Imu | 发布：IMU 数据 |
| `/camera/rgb/image_raw` | Image | 发布：RGB 图像 |
| `/camera/depth/image_raw` | Image | 发布：深度图像 |

---

## 7. 无显示器环境

服务器（无显示器）可用以下方式之一。

### 7.1 离屏渲染（EGL）

读取传感器数据不需要窗口：

```bash
MUJOCO_GL=egl python3 -c "
import os
os.environ['MUJOCO_GL']='egl'
import mujoco, numpy as np
m = mujoco.MjModel.from_xml_path('models/scorpio.xml')
d = mujoco.MjData(m); mujoco.mj_forward(m,d)
r = mujoco.Renderer(m, 480, 640)
r.update_scene(d, camera=mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA,'d435_rgb'))
print('RGB 帧:', r.render().shape)
"
```

### 7.2 浏览器可视化（HTTP 推流）

在远程机器上启动，本地浏览器查看：

```bash
MUJOCO_GL=egl python3 scripts/view_sim_web.py --terrain plaza --port 8080
# 浏览器打开 http://<服务器IP>:8080
```

### 7.3 虚拟显示器（Xvfb）

若必须使用窗口模式：

```bash
sudo apt install xvfb
Xvfb :99 -screen 0 1280x1024x24 &
export DISPLAY=:99
python3 scripts/run_seb_naver_sim.py --terrain plaza
```

---

## 8. 常见问题排查

### Q1: `ModuleNotFoundError: No module named 'mujoco'`

```bash
pip install mujoco
# 若用 conda，确认环境已激活
which python3 && python3 -c "import mujoco; print(mujoco.__file__)"
```

### Q2: 报错 `GLXBadContext` / 窗口不弹出

X11 认证问题，见 [第 3 节](#3-显示环境配置)：

```bash
export XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority
export DISPLAY=:0
```

### Q3: `Segmentation fault (core dumped)` 退出时崩溃

正常现象（NVIDIA 驱动 + MuJoCo 3.10 清理问题），不影响仿真结果。见 [3.4 节](#34-glx-段错误说明)。

### Q4: `Error opening file 'xxx.obj'`

网格路径问题。确认在**包根目录**下运行命令：

```bash
cd scorpio_mujoco_sim     # 必须在包根目录
python3 scripts/run_seb_naver_sim.py --terrain plaza
```

### Q5: 键盘无响应

需要 `pynput`：

```bash
pip install pynput
```

Linux 上可能需要 X11 权限，确认 `DISPLAY` 已设置。

### Q6: 机器人掉穿地面

模型文件损坏或版本不匹配，重新生成地形：

```bash
python3 scripts/generate_terrain.py
```

### Q7: 渲染很慢

MuJoCo 默认使用 CPU 渲染。降低分辨率或使用离屏渲染：

```bash
# 降低渲染分辨率
python3 scripts/run_seb_naver_sim.py --terrain plaza   # 修改脚本中 Renderer 尺寸
```

### Q8: 中文路径/文件名问题

MuJoCo 对非 ASCII 路径支持有限，确保项目路径为纯英文。

---

## 9. 卸载

```bash
# 删除项目
rm -rf scorpio_mujoco_sim

# 删除 Python 依赖 (若在独立虚拟环境中)
deactivate
rm -rf venv

# 清理 ~/.bashrc 中的显示配置 (手动编辑删除相关行)
```

---

## 附: 依赖版本参考

已在以下环境验证通过：

```
OS      : Ubuntu 22.04
Python  : 3.10.13
MuJoCo  : 3.10.0
NumPy   : 1.24+
GPU     : NVIDIA RTX 2070 (驱动 580.126.09)
```

若遇版本兼容问题，可尝试固定版本：

```bash
pip install "mujoco==3.10.0" "numpy<2.0"
```

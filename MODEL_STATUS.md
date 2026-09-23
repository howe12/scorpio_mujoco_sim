# Scorpio MuJoCo 仿真模型 - 当前版本记录

**记录时间**: 2026-09-23
**状态**: 模型主体 + YDLidar G6 激光雷达 已完成

## 已验证通过

### 机器人模型 (models/scorpio.xml)
- 驱动方式: 阿克曼转向 (后轮驱动 + 前轮转向)
- 所有部件坐标严格对照原始 URDF (scorpio_description)
- 部件位置误差: 0.0mm (全部场景)
- D435 相机: 安装位置正确，朝向正确，视野无遮挡
- 转向执行器: kp=20, kv=4, 45° 指令可达 43°
- 碰撞掩码: 机器人内部互不碰撞 (contype=1, conaffinity=4)

### 场景 (worlds/)
- plaza: 平坦地面 + 障碍物
- mountain/forest/snowy_mountain/pump_track: 不平地形 (heightfield)

### YDLidar G6 激光雷达 (已完成)
- 型号: YDLidar G6 (EAI), 实际网格 82.8 × 75.4 × 42.9 mm
- 安装: 车体顶部平台, 底座 z=0.250, 扫描中心 z=0.282 (相对 base_link)
- 量程: 0.12 ~ 16.0 m (NXROBO 官方规格)
- 扫描: 360°, 720 点 (0.5°), 12 Hz
- 仿真模块: scripts/lidar_simulator.py
- 支持 ignore_array (与真实 ROS 驱动参数一致)

## 关键参数

| 参数 | 值 | 来源 |
|------|-----|------|
| 车体位置 | (0,0,0) | URDF base_link |
| 车体 mesh 范围 | z=[0.0325, 0.3101] | scorpio.stl |
| 轮子位置 | x=±0.1575, y=±0.0928, z=0.0525 | URDF |
| D435 安装 | (0.1706, 0.0175, 0.242) | realsense2_description |
| LiDAR 底座 | (0, 0, 0.250) | 车体顶部平台 (mesh 实测) |
| LiDAR 扫描中心 | (0, 0, 0.282) | 底座 + 0.03172 |
| D435 相机 quat | (0.5, 0.5, 0.5, 0.5) | MuJoCo 约定 |
| 转向 kp/kv | 20 / 4 | 扫描测试 |
| 碰撞掩码 | robot: ct=1/ca=4, ground: ct=1/ca=7 | 位掩码排除内部碰撞 |

## 文件清单

- models/scorpio.xml: 机器人主模型
- models/meshes/*.obj: 外观网格 (body, wheels, d435)
- worlds/*.xml: 场景文件 (plaza + 4个地形)
- scripts/generate_terrain.py: 地形生成器
- scripts/run_seb_naver_sim.py: 运行脚本 (键盘控制+自动导航)
- scripts/scorpio_rl_env.py: RL 训练环境
- scripts/view_sim_web.py: 浏览器可视化

## 运行命令

```bash
export XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority
export DISPLAY=:0
cd /develop/scorpio_ros2/src/scorpio/scorpio_mujoco_sim

# 主场景 (键盘控制)
python3 scripts/run_seb_naver_sim.py --terrain plaza

# 其他场景
python3 scripts/run_seb_naver_sim.py --terrain mountain
python3 scripts/run_seb_naver_sim.py --terrain forest
python3 scripts/run_seb_naver_sim.py --terrain snowy_mountain
python3 scripts/run_seb_naver_sim.py --terrain pump_track

# 无头基准测试
python3 scripts/run_seb_naver_sim.py --headless
```

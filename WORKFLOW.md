# 开发工作流

本仿真包有两份副本，本文件说明如何在它们之间协作。

---

## 1. 两份副本

| 位置 | 角色 | 版本管理 | 推送目标 |
|------|------|---------|---------|
| `/develop/scorpio_mujoco_sim/` | **开发主副本** | 独立 git 仓库 | `howe12/scorpio_mujoco_sim` |
| `/develop/scorpio_ros2/src/scorpio/scorpio_mujoco_sim/` | **工作区副本** | scorpio_ros2 的未跟踪目录 | 暂不推送 |

**原则：以开发主副本为准。** 所有修改先在这里做，需要时同步到工作区副本。

---

## 2. 为什么这样分

- **开发主副本**是自包含的 MuJoCo 工程，克隆即可用，与上游 NXROBO 仓库完全解耦。
- **工作区副本**存在的意义是：`scorpio_ros2` 是一个完整 ROS2 工作区，只有把包放回 `src/` 下才能 `colcon build`，与其他功能包（导航、SLAM、驱动）联合编译测试。

---

## 3. 日常开发流程

### 3.1 在主副本开发

```bash
cd /develop/scorpio_mujoco_sim

# 修改模型/场景/脚本 ...

# 本地验证
python3 scripts/lidar_simulator.py
python3 scripts/run_seb_naver_sim.py --terrain plaza

# 提交并推送
git add -A
git commit -m "描述你的改动"
git push
```

### 3.2 需要时同步到工作区副本

```bash
cd /develop/scorpio_mujoco_sim

# 先预览将要同步的内容
./sync_to_ros2.sh --dry-run

# 实际同步 (自动验证 + 模型自检)
./sync_to_ros2.sh
```

脚本会：
1. 用 `rsync --delete` 让工作区副本与主副本**完全一致**（包括删除主副本中已移除的文件）
2. 校验两副本无差异
3. 逐个加载 6 个模型做自检

**指定其他工作区路径**：

```bash
SCORPIO_ROS2_WS=~/scorpio_ros2 ./sync_to_ros2.sh
```

### 3.3 在工作区内联合编译

```bash
cd /develop/scorpio_ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select scorpio_mujoco_sim
source install/setup.bash

ros2 launch scorpio_mujoco_sim scorpio_mujoco.launch.py
```

---

## 4. 反向同步（不推荐）

如果在工作区副本里临时改了东西需要拿回主副本：

```bash
# 手动复制，或
rsync -a --delete \
  --exclude='.git/' --exclude='__pycache__/' --exclude='*.pyc' \
  /develop/scorpio_ros2/src/scorpio/scorpio_mujoco_sim/ \
  /develop/scorpio_mujoco_sim/
```

> ⚠️ 反向同步会覆盖主副本，操作前先 `git status` 确认主副本没有未提交的改动。

---

## 5. 之后要合并进 scorpio_ros2 仓库

如果将来想把仿真包提交到 `scorpio_ros2` 的上游（或你自己的 fork）：

### 5.1 前提确认

```bash
cd /develop/scorpio_ros2
git remote -v      # 确认推送到哪里
git status --short # 确认改动范围
```

当前 remote 是 `https://github.com/NXROBO/scorpio_ros2.git`（**上游官方仓库，无推送权限**）。

### 5.2 方案 A：推到你自己的 fork

```bash
cd /develop/scorpio_ros2

# 改名为 upstream，避免误推
git remote rename origin upstream

# 添加你自己的 fork
git remote add origin https://github.com/howe12/scorpio_ros2.git

# 同步后提交
./src/scorpio/scorpio_mujoco_sim/sync_to_ros2.sh
git add src/scorpio/scorpio_mujoco_sim/
git commit -m "添加 Scorpio MuJoCo 仿真包"
git push -u origin main
```

### 5.3 方案 B：用 subtree 保留历史（进阶）

若希望仿真包的 commit 历史也带过去：

```bash
cd /develop/scorpio_ros2
git subtree add --prefix=src/scorpio/scorpio_mujoco_sim \
  https://github.com/howe12/scorpio_mujoco_sim.git main --squash

# 后续更新
git subtree pull --prefix=src/scorpio/scorpio_mujoco_sim \
  https://github.com/howe12/scorpio_mujoco_sim.git main --squash
```

### 5.4 注意

- **不要** `git add -A` 后直接提交 —— 工作区里还有 `build/`、`install/`、`log/` 等编译产物，以及 `MUJOCO_LOG.TXT`
- 仿真包内的 `.gitignore` 只对其自身目录生效；在 `scorpio_ros2` 根目录提交时会按根目录的 `.gitignore` 规则判断
- 提交前用 `git status --short` 逐项确认

---

## 6. 文件对照

```
/develop/scorpio_mujoco_sim/                    ← 开发主副本 (git 仓库)
├── README.md               使用说明
├── DEPLOY.md               环境部署指南
├── MODEL_STATUS.md         模型参数与验证记录
├── requirements.txt        Python 依赖
├── sync_to_ros2.sh         同步脚本 (本文件描述的工具)
├── WORKFLOW.md             本文件
├── .gitignore
├── package.xml / CMakeLists.txt   ROS2 包定义
├── models/                 机器人模型 + 网格 + 地形数据
├── worlds/                 5 个场景
├── scripts/                7 个脚本
└── launch/                 ROS2 启动文件

/develop/scorpio_ros2/src/scorpio/scorpio_mujoco_sim/   ← 工作区副本
   (内容与主副本一致，供 colcon build 使用)
```

---

## 7. 速查

| 操作 | 命令 |
|------|------|
| 开发 + 推送 | `cd /develop/scorpio_mujoco_sim && git add -A && git commit -m "..." && git push` |
| 同步到工作区 | `cd /develop/scorpio_mujoco_sim && ./sync_to_ros2.sh` |
| 预览同步内容 | `./sync_to_ros2.sh --dry-run` |
| 指定工作区 | `SCORPIO_ROS2_WS=<路径> ./sync_to_ros2.sh` |
| 联合编译 | `cd /develop/scorpio_ros2 && colcon build --packages-select scorpio_mujoco_sim` |
| 远程仓库 | https://github.com/howe12/scorpio_mujoco_sim |

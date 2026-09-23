#!/usr/bin/env bash
#
# sync_to_ros2.sh — 将本仿真包同步到 scorpio_ros2 工作区
#
# 用途:
#   本目录 (独立 git 仓库) 是开发主副本。
#   当需要回到完整 ROS2 工作区执行 colcon build，或在原工程内测试时，
#   用本脚本把改动同步过去。
#
# 用法:
#   ./sync_to_ros2.sh                 # 同步到默认路径
#   ./sync_to_ros2.sh --dry-run       # 预览将要同步的内容，不实际写入
#   SCORPIO_ROS2_WS=~/scorpio_ros2 ./sync_to_ros2.sh   # 指定工作区路径
#
# 默认工作区: /develop/scorpio_ros2
# 目标目录:   <工作区>/src/scorpio/scorpio_mujoco_sim
#
set -euo pipefail

SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS="${SCORPIO_ROS2_WS:-/develop/scorpio_ros2}"
DEST="$ROS2_WS/src/scorpio/scorpio_mujoco_sim"

DRY_RUN=""
if [[ "${1:-}" == "--dry-run" ]]; then
    DRY_RUN="--dry-run"
    echo "[预览模式] 不会写入任何文件"
    echo
fi

# ---- 前置检查 ----
if [[ ! -d "$ROS2_WS" ]]; then
    echo "错误: 工作区不存在: $ROS2_WS" >&2
    echo "      用 SCORPIO_ROS2_WS=<路径> 指定" >&2
    exit 1
fi

if ! command -v rsync >/dev/null 2>&1; then
    echo "错误: 需要 rsync (sudo apt install rsync)" >&2
    exit 1
fi

# 安全检查: 确认目标在预期位置，避免误删其它目录
case "$DEST" in
    */src/scorpio/scorpio_mujoco_sim) ;;
    *)
        echo "错误: 目标路径异常，拒绝执行: $DEST" >&2
        exit 1
        ;;
esac

echo "源目录:   $SRC"
echo "目标目录: $DEST"
echo

# ---- 同步 ----
# --delete: 让目标与源完全一致 (删除源中已移除的文件)
rsync -a --delete $DRY_RUN \
    --exclude='.git/' \
    --exclude='__pycache__/' \
    --exclude='*.pyc' \
    --exclude='MUJOCO_LOG.TXT' \
    --exclude='mujoco_log*.txt' \
    --exclude='venv/' \
    --exclude='.venv/' \
    "$SRC/" "$DEST/"

if [[ -n "$DRY_RUN" ]]; then
    echo
    echo "[预览模式结束] 去掉 --dry-run 实际执行"
    exit 0
fi

# ---- 验证 ----
echo
echo "=== 同步后验证 ==="
DIFF=$(diff -rq --exclude=.git --exclude=__pycache__ --exclude='*.pyc' \
        "$SRC" "$DEST" 2>&1 || true)
if [[ -z "$DIFF" ]]; then
    echo "✓ 两副本完全一致 ($(find "$DEST" -type f -not -name '*.pyc' -not -path '*__pycache__*' | wc -l) 个文件)"
else
    echo "⚠ 仍有差异:"
    echo "$DIFF"
    exit 1
fi

# ---- 模型自检 ----
echo
echo "=== 模型加载自检 ==="
( cd "$DEST" && python3 -c "
import mujoco, os, sys
sys.path.insert(0, 'scripts')
files = ['models/scorpio.xml'] + ['worlds/' + f for f in sorted(os.listdir('worlds'))]
ok = True
for f in files:
    try:
        m = mujoco.MjModel.from_xml_path(f)
        print(f'  OK  {f:34s} {m.nbody:3d} bodies')
    except Exception as e:
        ok = False
        print(f'  FAIL {f}: {e}')
sys.exit(0 if ok else 1)
" 2>&1 | grep -v "Loaded heightfield" )

echo
echo "✓ 同步完成"

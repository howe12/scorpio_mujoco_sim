#!/usr/bin/env bash
#
# eval_isolated.sh — 权威口径评估：每个目标独立 launch 会话
#
# 用法:
#   bash scripts/eval_isolated.sh                          # 默认 5 个目标
#   bash scripts/eval_isolated.sh "1.5,-1.5 3,-0.5"        # 自定义
#   TERRAIN=forest bash scripts/eval_isolated.sh "5,3"
#
# 产出: /tmp/eval_results/*.json  + 终端汇总表
#
set -o pipefail
export DISPLAY=:0 XAUTHORITY=/run/user/$(id -u)/gdm/Xauthority
source /opt/ros/humble/setup.bash
source /develop/scorpio_mujoco_sim_ws/install/setup.bash

PKG=/develop/scorpio_mujoco_sim_ws/src/scorpio_mujoco_sim
TERRAIN="${TERRAIN:-plaza}"
GOALS="${1:-1.5,-1.5 3,-0.5 3,-2 2.5,1.5 2,2}"
BOOT="${BOOT:-46}"
REPEAT="${REPEAT:-1}"   # 每个目标重复次数（独立会话），用于方差评估
OUTDIR=/tmp/eval_results
mkdir -p "$OUTDIR"
# 清掉本次 terrain 的旧结果，否则汇总会把上一次（可能被中断的）运行也算进去
rm -f "$OUTDIR"/eval_${TERRAIN}_*.json

# 复用仓库已验证的 clean_ros.sh，避免自制 kill 逻辑误杀自身
# （早期版本用宽泛 grep 模式 + ps 解析，曾把脚本自己 SIGKILL 掉）
clean_all() {
  bash /develop/scorpio_mujoco_sim_ws/clean_ros.sh >/dev/null 2>&1 || true
  sleep 2
}

echo "############################################################"
echo "# 评估 terrain=$TERRAIN  目标: $GOALS"
echo "############################################################"

i=0
TOTAL=$(( $(echo "$GOALS" | wc -w) * REPEAT ))
for r in $(seq 1 "$REPEAT"); do
for g in $GOALS; do
  i=$((i+1))
  safe=$(echo "$g" | tr ',.-' '___')
  LOG="$OUTDIR/launch_${TERRAIN}_r${r}_${safe}.log"
  JSON="$OUTDIR/eval_${TERRAIN}_r${r}_${safe}.json"
  echo ""
  echo "---- [$i/$TOTAL r$r] 目标 $g ----"
  clean_all
  nohup ros2 launch seb_naver_planner seb_naver_full.launch.py \
      terrain:="$TERRAIN" sim_gui:=false rviz:=false > "$LOG" 2>&1 &
  sleep "$BOOT"
  timeout 150 /usr/bin/python3 "$PKG/scripts/eval_navigation.py" \
      --terrain "$TERRAIN" --goals "$g" --timeout 80 --stall 20 \
      --json "$JSON" 2>&1 | grep -E "^\[|✅|❌|成功率|到达时间|平均速度|碰撞|路径效率"
done
done
clean_all

echo ""
echo "############################################################"
echo "# 全部结果汇总"
/usr/bin/python3 "$PKG/scripts/summarize_eval.py" --dir "$OUTDIR" --terrain "$TERRAIN" \
    ${SUMMARY_JSON:+--json "$SUMMARY_JSON"}

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
for g in $GOALS; do
  i=$((i+1))
  safe=$(echo "$g" | tr ',.-' '___')
  LOG="$OUTDIR/launch_${TERRAIN}_${safe}.log"
  JSON="$OUTDIR/eval_${TERRAIN}_${safe}.json"
  echo ""
  echo "---- [$i] 目标 $g ----"
  clean_all
  nohup ros2 launch seb_naver_planner seb_naver_full.launch.py \
      terrain:="$TERRAIN" sim_gui:=false rviz:=false > "$LOG" 2>&1 &
  sleep "$BOOT"
  timeout 150 /usr/bin/python3 "$PKG/scripts/eval_navigation.py" \
      --terrain "$TERRAIN" --goals "$g" --timeout 80 --stall 20 \
      --json "$JSON" 2>&1 | grep -E "^\[|✅|❌|成功率|到达时间|平均速度|碰撞|路径效率"
done
clean_all

echo ""
echo "############################################################"
echo "# 全部结果汇总"
/usr/bin/python3 - "$OUTDIR" "$TERRAIN" <<'PY'
import glob, json, sys, statistics as st
d, terrain = sys.argv[1], sys.argv[2]
runs = []
for f in sorted(glob.glob(f"{d}/eval_{terrain}_*.json")):
    runs += json.load(open(f))["runs"]
if not runs:
    print("无结果")
    sys.exit()
ok = [r for r in runs if r["ok"]]
print(f"{'目标':>14} {'结果':>6} {'耗时':>7} {'终点':>7} {'yaw':>6} {'均速':>8} {'间距':>6}")
for r in runs:
    g = f"({r['goal'][0]:.1f},{r['goal'][1]:.1f})"
    print(f"{g:>14} {'成功' if r['ok'] else '失败':>6} {r['time_s']:6.1f}s "
          f"{r['final_dist']:6.2f}m {r['final_yaw_err_deg']:5.0f}° "
          f"{(r['avg_speed'] or 0):7.3f} {r['min_clearance']:5.2f}")
print("-" * 60)
print(f"成功率 {len(ok)}/{len(runs)} = {100*len(ok)/len(runs):.0f}%")
if ok:
    t = [r["time_s"] for r in ok]
    s = [r["avg_speed"] for r in ok if r["avg_speed"]]
    print(f"到达时间 平均 {st.mean(t):.1f}s  范围 {min(t):.1f}~{max(t):.1f}s")
    if s:
        print(f"平均速度 {st.mean(s):.3f} m/s (上限 0.5, 达成 {100*st.mean(s)/0.5:.0f}%)")
print(f"碰撞 {sum(1 for r in runs if r['collided'])} 次")
PY

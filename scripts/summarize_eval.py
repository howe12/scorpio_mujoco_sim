#!/usr/bin/env python3
"""汇总一次评估套件的全部 JSON，输出方差统计与基线快照。

按 `docs/EVALUATION.md` §6/§8 的口径：
  - 每个目标聚合多次重复（独立会话）
  - 报告 mean / std / 变异系数 CV
  - 判定方差是否达标（CV < 20%，见 §8 通过标准）
  - 可选 `--json` 写出基线快照，供后续参数搜索对比

用法:
    /usr/bin/python3 scripts/summarize_eval.py \
        --dir /tmp/eval_results --terrain plaza \
        --json /tmp/eval_results/baseline_v1.json
"""

from __future__ import annotations

import argparse
import glob
import json
import statistics as st
import sys


def load(paths):
    runs = []
    for f in paths:
        try:
            with open(f) as fh:
                runs += json.load(fh)["runs"]
        except (OSError, KeyError, json.JSONDecodeError) as exc:
            print(f"跳过 {f}: {exc}", file=sys.stderr)
    return runs


def cv(values) -> float:
    """变异系数（std/mean）。样本不足 2 个时返回 nan。"""
    if len(values) < 2:
        return float("nan")
    m = st.mean(values)
    return st.stdev(values) / m if m else float("nan")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default="/tmp/eval_results")
    ap.add_argument("--terrain", default="plaza")
    ap.add_argument("--json", default="")
    ap.add_argument("--cv-limit", type=float, default=0.20)
    args = ap.parse_args()

    paths = sorted(glob.glob(f"{args.dir}/eval_{args.terrain}_*.json"))
    runs = load(paths)
    if not runs:
        print(f"没有找到 {args.dir}/eval_{args.terrain}_*.json")
        return 1

    # 按目标分组
    groups: dict[tuple, list] = {}
    for r in runs:
        groups.setdefault((round(r["goal"][0], 2), round(r["goal"][1], 2)), []).append(r)

    print("=" * 96)
    print(f"基线汇总  terrain={args.terrain}  "
          f"{len(runs)} 次运行 / {len(groups)} 个目标  "
          f"(来自 {len(paths)} 个结果文件)")
    print("=" * 96)
    print(f"{'目标':>14} {'成功':>6} {'耗时 mean±std':>20} {'CV':>7} "
          f"{'终点 mean±std':>18} {'均速 mean':>10} {'间距 min':>9}")

    per_goal = []
    for (gx, gy), rs in sorted(groups.items()):
        ok = [r for r in rs if r["ok"]]
        times = [r["time_s"] for r in ok]
        dists = [r["final_dist"] for r in ok]
        spds = [r["avg_speed"] for r in ok if r["avg_speed"]]
        clr = [r["min_clearance"] for r in rs]
        c = cv(times) if len(times) >= 2 else float("nan")
        t_txt = (f"{st.mean(times):.1f}±{st.stdev(times):.1f}s"
                 if len(times) >= 2 else (f"{times[0]:.1f}s" if times else "—"))
        d_txt = (f"{st.mean(dists):.2f}±{st.stdev(dists):.2f}m"
                 if len(dists) >= 2 else (f"{dists[0]:.2f}m" if dists else "—"))
        cv_txt = f"{c*100:.0f}%" if c == c else "—"
        print(f"  ({gx:5.2f},{gy:5.2f}) {len(ok):>3}/{len(rs):<2} {t_txt:>20} "
              f"{cv_txt:>7} {d_txt:>18} "
              f"{(st.mean(spds) if spds else 0):>10.3f} {min(clr):>8.2f}")
        per_goal.append({
            "goal": [gx, gy],
            "n": len(rs),
            "success": len(ok),
            "time_mean": round(st.mean(times), 2) if times else None,
            "time_std": round(st.stdev(times), 2) if len(times) >= 2 else None,
            "time_cv": round(c, 3) if c == c else None,
            "final_dist_mean": round(st.mean(dists), 3) if dists else None,
            "speed_mean": round(st.mean(spds), 4) if spds else None,
            "min_clearance": round(min(clr), 3),
        })

    ok_all = [r for r in runs if r["ok"]]
    times_all = [r["time_s"] for r in ok_all]
    spds_all = [r["avg_speed"] for r in ok_all if r["avg_speed"]]
    cvs = [g["time_cv"] for g in per_goal if g["time_cv"] is not None]

    print("-" * 96)
    print(f"成功率      : {len(ok_all)}/{len(runs)} = {100*len(ok_all)/len(runs):.0f}%")
    if times_all:
        print(f"到达时间    : 平均 {st.mean(times_all):.1f}s  "
              f"范围 {min(times_all):.1f}~{max(times_all):.1f}s")
    if spds_all:
        print(f"平均速度    : {st.mean(spds_all):.3f} m/s  "
              f"(上限 0.5, 达成 {100*st.mean(spds_all)/0.5:.0f}%)")
    print(f"碰撞        : {sum(1 for r in runs if r['collided'])} 次")
    print(f"失败        : {len(runs)-len(ok_all)} 次")

    # 方差判定（§8 通过标准）
    verdict = "无法判定（每目标样本 <2）"
    if cvs:
        worst = max(cvs)
        verdict = (f"{'✅ 达标' if worst < args.cv_limit else '❌ 未达标'}  "
                   f"最大 CV {worst*100:.0f}% (门槛 {args.cv_limit*100:.0f}%)")
    print(f"耗时方差    : {verdict}")
    print("=" * 96)

    if args.json:
        payload = {
            "terrain": args.terrain,
            "n_runs": len(runs),
            "n_goals": len(groups),
            "summary": {
                "success": len(ok_all),
                "success_rate": round(len(ok_all) / len(runs), 3),
                "time_mean": round(st.mean(times_all), 2) if times_all else None,
                "speed_mean": round(st.mean(spds_all), 4) if spds_all else None,
                "collisions": sum(1 for r in runs if r["collided"]),
                "max_time_cv": round(max(cvs), 3) if cvs else None,
                "variance_ok": bool(cvs and max(cvs) < args.cv_limit),
            },
            "per_goal": per_goal,
        }
        with open(args.json, "w") as f:
            json.dump(payload, f, indent=2, ensure_ascii=False)
        print(f"基线快照 → {args.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

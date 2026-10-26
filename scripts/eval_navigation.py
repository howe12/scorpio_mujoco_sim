#!/usr/bin/env python3
"""导航评估执行器：按 docs/EVALUATION.md 的口径测量单次或多次导航。

与 benchmark_navigation.py 的区别:
  - 记录完整 /odom 轨迹，事后核算路径长度 / 平均速度 / 最小间距 / 平滑度
  - 用 /sdf_map 快照算实际轨迹的障碍间距（判碰撞）
  - 输出 JSON，便于参数搜索脚本消费
  - 支持 --repeat 重复与统计

用法:
    # 单次（会话已启动）
    /usr/bin/python3 scripts/eval_navigation.py --goals "1.5,-1.5 3,-0.5" \
        --json /tmp/eval.json

    # 每个目标独立会话（权威口径）
    bash scripts/eval_isolated.sh "1.5,-1.5 3,-0.5 3,-2 2.5,1.5 2,2"

退出码: 0 全部成功; 1 有失败
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from se2_grid_msgs.msg import SE2Grid
from std_msgs.msg import String


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def norm_angle(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class EvalNode(Node):
    def __init__(self) -> None:
        super().__init__("nav_eval")
        self.odom = None
        self.fsm = ""
        self.sdf = None          # (grid, resolution, origin_x, origin_y)
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.create_subscription(String, "/planner/status", self._fsm_cb, 10)
        self.create_subscription(SE2Grid, "/sdf_map", self._sdf_cb, 10)
        self.pub = self.create_publisher(PoseStamped, "/goal_pose", 10)

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom = msg

    def _fsm_cb(self, msg: String) -> None:
        self.fsm = msg.data

    def _sdf_cb(self, msg: SE2Grid) -> None:
        idx = None
        for i, name in enumerate(msg.layers):
            if name == "sdf":
                idx = i
                break
        if idx is None:
            return
        info = msg.info
        res = float(info.pos_resolution)
        if res <= 0.0:
            return
        rows = int(round(info.length_y / res))
        cols = int(round(info.length_x / res))
        n = rows * cols
        raw = np.array(msg.data[idx].data, dtype=np.float32)
        if raw.size < n:
            return
        g = raw[:n].reshape((rows, cols))
        sr = int(msg.start_index_row) % rows
        sc = int(msg.start_index_col) % cols
        g = np.roll(np.roll(g, -sr, axis=0), -sc, axis=1)
        ox = info.pose.position.x - info.length_x / 2.0
        oy = info.pose.position.y - info.length_y / 2.0
        self.sdf = (g, res, ox, oy)

    def pose(self):
        if self.odom is None:
            return None
        p = self.odom.pose.pose
        return p.position.x, p.position.y, yaw_of(p.orientation)

    def speed(self) -> float:
        if self.odom is None:
            return 0.0
        t = self.odom.twist.twist.linear
        return math.hypot(t.x, t.y)

    def send_goal(self, x: float, y: float, yaw: float = 0.0) -> bool:
        """发布目标并确认 planner 已接收（FSM 离开 WAIT_TARGET）。"""
        m = PoseStamped()
        m.header.frame_id = "camera_init"
        m.pose.position.x = float(x)
        m.pose.position.y = float(y)
        m.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.orientation.w = math.cos(yaw / 2.0)
        for _ in range(8):
            m.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(m)
            t0 = time.time()
            while time.time() - t0 < 0.8:
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.fsm in ("SEQUENTIAL_START", "EXEC_TRAJ", "REPLAN_TRAJ"):
                    return True
        return False


def clearance_field(sdf):
    """由 SDF 快照构建距离场（米，障碍内为负）。"""
    if sdf is None:
        return None
    grid, res, ox, oy = sdf
    occ = np.asarray(grid) < 0.0
    if not occ.any():
        return {"field": np.full_like(grid, 9.0), "res": res, "ox": ox, "oy": oy}
    from scipy.ndimage import distance_transform_edt
    d_out = distance_transform_edt(~occ) * res
    d_in = distance_transform_edt(occ) * res
    return {"field": (d_out - d_in).astype(np.float32), "res": res, "ox": ox, "oy": oy}


def clearance_at(clr, x: float, y: float) -> float:
    f, res, ox, oy = clr["field"], clr["res"], clr["ox"], clr["oy"]
    h, w = f.shape
    fx = min(max((x - ox) / res, 0.0), w - 1.001)
    fy = min(max((y - oy) / res, 0.0), h - 1.001)
    c0, r0 = int(fx), int(fy)
    dx, dy = fx - c0, fy - r0
    return float(
        f[r0, c0] * (1 - dx) * (1 - dy) + f[r0, c0 + 1] * dx * (1 - dy)
        + f[r0 + 1, c0] * (1 - dx) * dy + f[r0 + 1, c0 + 1] * dx * dy
    )


def run_one(node: EvalNode, gx: float, gy: float, args) -> dict:
    p0 = node.pose()
    traj = []           # (t, x, y, yaw)
    clr = clearance_field(node.sdf)
    t0 = time.time()
    accepted = node.send_goal(gx, gy, 0.0)
    t_send = time.time()

    best_dist = float("inf")
    last_move = t_send
    last_xy = None
    verdict = "timeout"
    while True:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.time()
        el = now - t_send
        p = node.pose()
        if p is None:
            if el > args.timeout:
                break
            continue
        x, y, yaw = p
        traj.append((now - t_send, x, y, yaw))
        dist = math.hypot(gx - x, gy - y)
        yaw_err = abs(norm_angle(0.0 - yaw))
        spd = node.speed()

        if dist < best_dist - 0.02:
            best_dist = dist
            last_move = now
        if last_xy is None or math.hypot(x - last_xy[0], y - last_xy[1]) > 0.05:
            last_move = now
            last_xy = (x, y)

        if dist < args.tol_dist and yaw_err < args.tol_yaw and spd < args.tol_vel:
            verdict = "arrived"
            break
        if el > args.timeout:
            verdict = "timeout"
            break
        if now - last_move > args.stall:
            verdict = "stalled"
            break

    dur = (time.time() - t_send)
    x, y, yaw = node.pose()
    dist = math.hypot(gx - x, gy - y)
    yaw_err = abs(norm_angle(0.0 - yaw))

    # 轨迹级指标
    path_len = 0.0
    min_clr = 9.0
    yaw_rates = []
    prev = None
    for (tt, tx, ty, tyaw) in traj:
        if prev is not None:
            dt = tt - prev[0]
            ds = math.hypot(tx - prev[1], ty - prev[2])
            path_len += ds
            if dt > 1e-6:
                yaw_rates.append(abs(norm_angle(tyaw - prev[3])) / dt)
        if clr is not None:
            min_clr = min(min_clr, clearance_at(clr, tx, ty))
        prev = (tt, tx, ty, tyaw)

    # 轨迹降采样（最多 40 点），用于诊断绕圈/振荡
    step = max(1, len(traj) // 40)
    traj_ds = [[round(t, 2), round(x, 2), round(y, 2)]
               for (t, x, y, _y) in traj[::step]]

    straight = math.hypot(gx - p0[0], gy - p0[1]) if p0 else float("nan")
    return {
        "goal": [gx, gy],
        "start": [p0[0], p0[1]] if p0 else None,
        "ok": verdict == "arrived",
        "verdict": verdict,
        "accepted": accepted,
        "time_s": round(dur, 2),
        "final_dist": round(dist, 3),
        "final_yaw_err_deg": round(math.degrees(yaw_err), 1),
        "best_dist": round(best_dist, 3),
        "path_len": round(path_len, 3),
        "straight_dist": round(straight, 3),
        "path_efficiency": round(straight / path_len, 3) if path_len > 1e-6 else None,
        "avg_speed": round(path_len / dur, 4) if dur > 1e-6 else None,
        "min_clearance": round(min_clr, 3),
        "collided": bool(min_clr < 0.0),
        "yaw_rate_rms": round(float(np.sqrt(np.mean(np.square(yaw_rates)))), 3)
                       if yaw_rates else None,
        "n_samples": len(traj),
        "trajectory": traj_ds,
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--terrain", default="plaza")
    ap.add_argument("--goals", default="", help='例如 "1.5,-1.5 3,-0.5"')
    ap.add_argument("--timeout", type=float, default=80.0)
    ap.add_argument("--stall", type=float, default=20.0)
    ap.add_argument("--tol-dist", type=float, default=0.8)
    ap.add_argument("--tol-yaw", type=float, default=0.5)
    ap.add_argument("--tol-vel", type=float, default=0.05)
    ap.add_argument("--repeat", type=int, default=1,
                    help="同一会话内对每个目标重复次数（会状态污染，权威值用 1）")
    ap.add_argument("--json", default="", help="结果写入 JSON")
    args = ap.parse_args()

    if args.goals:
        goals = []
        for tok in args.goals.split():
            a, b = tok.split(",")
            goals.append((float(a), float(b)))
    else:
        goals = [(1.5, -1.5), (3.0, -0.5), (3.0, -2.0), (2.5, 1.5), (2.0, 2.0)]

    rclpy.init()
    node = EvalNode()
    t = time.time()
    while (node.odom is None or node.sdf is None) and time.time() - t < 40:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.odom is None:
        print("❌ 收不到 /odom")
        return 1
    if node.sdf is None:
        print("⚠️ 收不到 /sdf_map，间距/碰撞指标将不可用")

    print("=" * 90)
    print(f"导航评估  terrain={args.terrain}  目标 {len(goals)} 个  重复 {args.repeat}")
    print(f"判据 dist<{args.tol_dist}m yaw<{args.tol_yaw}rad v<{args.tol_vel}  "
          f"超时 {args.timeout:.0f}s 停滞 {args.stall:.0f}s")
    print("=" * 90)

    results = []
    for k in range(args.repeat):
        for i, (gx, gy) in enumerate(goals, 1):
            tag = f"[{i}/{len(goals)}]" + (f" r{k+1}" if args.repeat > 1 else "")
            print(f"\n{tag} 目标 ({gx}, {gy})")
            r = run_one(node, gx, gy, args)
            results.append(r)
            mark = "✅" if r["ok"] else "❌"
            print(f"  {mark} {r['verdict']}  {r['time_s']:.1f}s  "
                  f"终点距离 {r['final_dist']:.2f}m  yaw {r['final_yaw_err_deg']:.0f}°  "
                  f"路径 {r['path_len']:.1f}m  "
                  f"均速 {r['avg_speed'] or 0:.3f}m/s  "
                  f"最小间距 {r['min_clearance']:.2f}m"
                  + ("  ⚠️碰撞" if r["collided"] else ""))

    # 汇总
    ok = [r for r in results if r["ok"]]
    times = [r["time_s"] for r in ok]
    spds = [r["avg_speed"] for r in ok if r["avg_speed"]]
    print("\n" + "=" * 90)
    print(f"成功率      : {len(ok)}/{len(results)} = {100*len(ok)/len(results):.0f}%")
    if times:
        print(f"到达时间    : 平均 {np.mean(times):.1f}s  最好 {min(times):.1f}s  "
              f"最差 {max(times):.1f}s")
    if spds:
        print(f"平均速度    : {np.mean(spds):.3f} m/s  (上限 0.5, 达成率 "
              f"{100*np.mean(spds)/0.5:.0f}%)")
    print(f"碰撞        : {sum(1 for r in results if r['collided'])} 次")
    print(f"超时/卡住   : {sum(1 for r in results if r['verdict'] != 'arrived')} 次")
    effs = [r["path_efficiency"] for r in results if r["path_efficiency"]]
    if effs:
        print(f"路径效率    : {np.mean(effs):.2f} (1.0=直线)")
    print("=" * 90)

    if args.json:
        payload = {
            "terrain": args.terrain,
            "criteria": {"tol_dist": args.tol_dist, "tol_yaw": args.tol_yaw,
                         "tol_vel": args.tol_vel, "timeout": args.timeout,
                         "stall": args.stall},
            "summary": {
                "n": len(results),
                "success": len(ok),
                "success_rate": round(len(ok) / len(results), 3),
                "time_mean": round(float(np.mean(times)), 2) if times else None,
                "speed_mean": round(float(np.mean(spds)), 4) if spds else None,
                "collisions": sum(1 for r in results if r["collided"]),
                "failures": sum(1 for r in results if r["verdict"] != "arrived"),
                "path_efficiency_mean": round(float(np.mean(effs)), 3) if effs else None,
            },
            "runs": results,
        }
        with open(args.json, "w") as f:
            json.dump(payload, f, indent=2, ensure_ascii=False)
        print(f"JSON → {args.json}")

    rclpy.shutdown()
    return 0 if len(ok) == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())

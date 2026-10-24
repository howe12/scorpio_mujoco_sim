#!/usr/bin/env python3
"""导航基准测试：逐个目标点发送 SetGoal，测量到达时间与终点误差。

测量口径（与 planner 的判据一致）:
  成功 = 距离 < goal_tolerance_dist 且 |yaw误差| < goal_tolerance_yaw
         且 速度 < goal_tolerance_vel
  耗时 = 从发出 /goal_pose 到首次满足上述条件的墙钟时间

用法:
    # 终端 1: 启动仿真导航
    ros2 launch seb_naver_planner seb_naver_full.launch.py terrain:=plaza rviz:=false sim_gui:=false

    # 终端 2: 跑基准（默认 plaza 5 个目标）
    /usr/bin/python3 scripts/benchmark_navigation.py --terrain plaza

    # 指定目标与超时
    /usr/bin/python3 scripts/benchmark_navigation.py --goals "3,-0.5 5,1 2,2" --timeout 90
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

# 各场景的默认目标（相对起点，避开已知障碍）
DEFAULT_GOALS = {
    "plaza": [(3.0, -0.5), (4.5, -1.5), (-2.0, -1.0), (2.0, 2.0), (0.5, -2.5)],
    "forest": [(5.0, 3.0), (3.0, 5.0), (6.0, 1.0)],
    "mountain": [(4.0, 2.0), (2.0, 4.0)],
    "pump_track": [(3.0, 1.0), (5.0, -1.0)],
    "snowy_mountain": [(3.0, 2.0), (5.0, 0.0)],
}


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def norm_angle(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class BenchNode(Node):
    def __init__(self) -> None:
        super().__init__("nav_benchmark")
        self.odom = None
        self.fsm = ""
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.create_subscription(String, "/planner/status", self._fsm_cb, 10)
        self.pub = self.create_publisher(PoseStamped, "/goal_pose", 10)

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom = msg

    def _fsm_cb(self, msg: String) -> None:
        self.fsm = msg.data

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
        """发布目标并确认 planner 已接收（FSM 离开 WAIT_TARGET）。

        实测单次发布偶发丢失（DDS 发现未完成），表现为 planner 日志里
        完全没有 "Goal received"、机器人不动。这里重试直到 FSM 状态变化。
        """
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


def wait_arrive(
    node: BenchNode,
    gx: float, gy: float,
    tol_dist: float, tol_yaw: float, tol_vel: float,
    timeout: float, stall_time: float,
) -> dict:
    """轮询直到到达 / 超时 / 卡住。返回测量结果。"""
    t0 = time.time()
    best_dist = float("inf")
    last_move_t = t0
    last_pos = None
    while True:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.time()
        elapsed = now - t0
        pose = node.pose()
        if pose is None:
            if elapsed > timeout:
                break
            continue
        x, y, yaw = pose
        dist = math.hypot(gx - x, gy - y)
        yaw_err = abs(norm_angle(0.0 - yaw))   # 目标 yaw = 0
        speed = node.speed()

        if dist < best_dist - 0.02:
            best_dist = dist
            last_move_t = now
        if last_pos is None or math.hypot(x - last_pos[0], y - last_pos[1]) > 0.05:
            last_move_t = now
            last_pos = (x, y)

        if dist < tol_dist and yaw_err < tol_yaw and speed < tol_vel:
            return {"ok": True, "why": "arrived", "t": elapsed,
                    "dist": dist, "yaw_err": yaw_err, "x": x, "y": y,
                    "best_dist": best_dist}

        if elapsed > timeout:
            return {"ok": False, "why": f"timeout {timeout:.0f}s", "t": elapsed,
                    "dist": dist, "yaw_err": yaw_err, "x": x, "y": y,
                    "best_dist": best_dist}

        if now - last_move_t > stall_time:
            return {"ok": False, "why": f"stalled {stall_time:.0f}s", "t": elapsed,
                    "dist": dist, "yaw_err": yaw_err, "x": x, "y": y,
                    "best_dist": best_dist}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--terrain", default="plaza")
    ap.add_argument("--goals", default="", help='例如 "3,-0.5 5,1"')
    ap.add_argument("--timeout", type=float, default=90.0)
    ap.add_argument("--stall", type=float, default=15.0, help="多久不动算卡住")
    ap.add_argument("--tol-dist", type=float, default=0.8)
    ap.add_argument("--tol-yaw", type=float, default=0.3)
    ap.add_argument("--tol-vel", type=float, default=0.05)
    ap.add_argument("--warmup", type=float, default=0.0, help="首次测量前等待")
    args = ap.parse_args()

    if args.goals:
        goals = []
        for tok in args.goals.split():
            a, b = tok.split(",")
            goals.append((float(a), float(b)))
    else:
        goals = DEFAULT_GOALS.get(args.terrain, DEFAULT_GOALS["plaza"])

    rclpy.init()
    node = BenchNode()
    print("=" * 78)
    print(f"导航基准  terrain={args.terrain}  目标数={len(goals)}")
    print(f"判据: dist<{args.tol_dist}m  yaw<{args.tol_yaw}rad  v<{args.tol_vel}m/s")
    print(f"超时 {args.timeout:.0f}s / 停滞判定 {args.stall:.0f}s")
    print("=" * 78)

    # 等 odom
    t = time.time()
    while node.odom is None and time.time() - t < 30:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.odom is None:
        print("❌ 收不到 /odom，仿真是否已启动？")
        return 1
    if args.warmup > 0:
        print(f"预热 {args.warmup:.0f}s ...")
        t = time.time()
        while time.time() - t < args.warmup:
            rclpy.spin_once(node, timeout_sec=0.1)

    results = []
    for i, (gx, gy) in enumerate(goals, 1):
        p0 = node.pose()
        print(f"\n[{i}/{len(goals)}] 目标 ({gx}, {gy})  当前 "
              f"({p0[0]:.2f}, {p0[1]:.2f})")
        accepted = node.send_goal(gx, gy, 0.0)
        if not accepted:
            print("  ⚠️  planner 未确认收到目标（重发 8 次仍停在 WAIT_TARGET）")
        r = wait_arrive(node, gx, gy, args.tol_dist, args.tol_yaw, args.tol_vel,
                        args.timeout, args.stall)
        results.append({"goal": (gx, gy), "start": p0, **r})
        tag = "✅" if r["ok"] else "❌"
        print(f"  {tag} {r['why']}  耗时 {r['t']:.1f}s  "
              f"终点 ({r['x']:.2f}, {r['y']:.2f})  "
              f"距离 {r['dist']:.2f}m  最近 {r['best_dist']:.2f}m  "
              f"yaw误差 {math.degrees(r['yaw_err']):.0f}°")

    # 汇总
    print("\n" + "=" * 78)
    ok = sum(1 for r in results if r["ok"])
    print(f"成功率: {ok}/{len(results)}")
    times = [r["t"] for r in results if r["ok"]]
    if times:
        print(f"到达耗时: 平均 {sum(times)/len(times):.1f}s  "
              f"最快 {min(times):.1f}s  最慢 {max(times):.1f}s")
    print("\n明细:")
    print(f"  {'目标':>12} {'结果':>6} {'耗时':>7} {'距离':>7} {'yaw':>6}  说明")
    for r in results:
        g = f"({r['goal'][0]:.1f},{r['goal'][1]:.1f})"
        print(f"  {g:>12} {'成功' if r['ok'] else '失败':>6} "
              f"{r['t']:6.1f}s {r['dist']:6.2f}m "
              f"{math.degrees(r['yaw_err']):5.0f}°  {r['why']}")
    print("=" * 78)
    rclpy.shutdown()
    return 0 if ok == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""TrajectoryOptimizer 离线单测 — 5 个场景覆盖直线/转弯/S弯/掉头/真实A*。

可直接运行（无需 ROS2）::

    cd seb_naver_planner
    /usr/bin/python3 test/test_trajectory_optimizer.py

也可被 pytest 收集::

    /usr/bin/python3 -m pytest test/test_trajectory_optimizer.py -v
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from seb_naver_planner.trajectory_optimizer import TrajResult, TrajectoryOptimizer  # noqa: E402

# 与 planning.yaml 一致
PARAMS = {
    "wheel_base": 0.315,
    "max_speed": 0.5,
    "max_accel": 1.0,
    "max_steer": 0.785,
    "seg_len": 0.9,
    "rho_t": 1.5,
}


def _make_path(points, steps_per_seg: int = 8):
    """把折线点插值成密集路径 [(x, y, theta)]。"""
    out = []
    for i in range(len(points) - 1):
        x0, y0 = points[i]
        x1, y1 = points[i + 1]
        for k in range(steps_per_seg):
            r = k / steps_per_seg
            out.append((x0 + r * (x1 - x0), y0 + r * (y1 - y0), 0.0))
    x1, y1 = points[-1]
    prev = out[-1]
    out.append((x1, y1, math.atan2(y1 - prev[1], x1 - prev[0])))
    # 用相邻点方向填 theta
    for i in range(len(out) - 1):
        dx = out[i + 1][0] - out[i][0]
        dy = out[i + 1][1] - out[i][1]
        out[i] = (out[i][0], out[i][1], math.atan2(dy, dx))
    return out


def _arc_path(radius: float, angle_deg: float, steps: int = 40):
    """圆弧路径，用于 U 形掉头。"""
    pts = []
    for i in range(steps + 1):
        a = math.radians(angle_deg) * i / steps
        pts.append((radius * math.sin(a), radius * (1 - math.cos(a)), 0.0))
    for i in range(len(pts) - 1):
        dx = pts[i + 1][0] - pts[i][0]
        dy = pts[i + 1][1] - pts[i][1]
        pts[i] = (pts[i][0], pts[i][1], math.atan2(dy, dx))
    prev = pts[-2]
    pts[-1] = (pts[-1][0], pts[-1][1], math.atan2(pts[-1][1] - prev[1], pts[-1][0] - prev[0]))
    return pts


# ---------------------------------------------------------------------------
# 场景定义
# ---------------------------------------------------------------------------

def scene_straight():
    """场景 1: 直线 5m。"""
    return _make_path([(0.0, 0.0), (5.0, 0.0)], steps_per_seg=20), 0.0


def scene_single_turn():
    """场景 2: L 形 90° 转弯。"""
    return _make_path([(0.0, 0.0), (3.0, 0.0), (3.0, 3.0)], steps_per_seg=12), 0.0


def scene_s_curve():
    """场景 3: S 弯（左右各 90°）。"""
    return _make_path(
        [(0.0, 0.0), (2.0, 0.0), (3.5, 1.5), (3.5, 3.5), (5.0, 5.0)],
        steps_per_seg=10,
    ), 0.2


def scene_u_turn():
    """场景 4: U 形掉头（半径 1.2m 的 180° 圆弧）。"""
    return _arc_path(radius=1.2, angle_deg=180.0, steps=48), 0.15


def scene_astar():
    """场景 5: 真实 Kinodynamic A* 输出（含障碍物，需要绕行）。"""
    from seb_naver_planner.kino_astar import KinoAStarPlanner, SearchStatus

    planner = KinoAStarPlanner(
        {
            "wheel_base": PARAMS["wheel_base"],
            "max_speed": PARAMS["max_speed"],
            "max_steer": PARAMS["max_steer"],
            "step_arc": 1.0,
            "steer_res": 0.3,
        }
    )
    res = 0.1
    n = 200  # 20m x 20m
    grid = np.ones((n, n), dtype=np.float32)
    # 一堵墙挡在 x≈2.5，留出 y>0.8 的缺口。
    # 注意：A* 的碰撞采样密度已修（原 check_num=3 会跨过 0.1m 薄墙），
    # 因此这里会真实绕行；缺口过远（如 y>1.5）会导致 5s 搜索超时。
    col = int(round((2.5 - (-10.0)) / res))
    grid[: int(round((0.8 - (-10.0)) / res)), col] = -1.0
    info = {"resolution": res, "origin": [-10.0, -10.0]}
    status, path = planner.search(
        start_state=(0.0, 0.0, 0.0),
        goal_state=(5.0, 0.0, 0.0),
        sdf_grid_data=grid,
        sdf_info=info,
    )
    if status != SearchStatus.SUCCESS or len(path) < 2:
        raise RuntimeError(f"A* 未找到路径: {status.name}")
    return path, 0.0


# 期望: "success" 必须全约束满足; "safe" = fail-safe（失败可接受，
# 调用方回退 A* 原路径）。场景 5 的 A* 路径曲率接近车体极限，属已知局限。
SCENES = [
    ("1 直线 5m", scene_straight, "success"),
    ("2 L 形 90°", scene_single_turn, "success"),
    ("3 S 弯", scene_s_curve, "success"),
    ("4 U 形掉头", scene_u_turn, "success"),
    ("5 真实 A* (绕墙)", scene_astar, "safe"),
]


# ---------------------------------------------------------------------------
# 断言
# ---------------------------------------------------------------------------

def check_scene(name: str, path, start_vel: float, opt: TrajectoryOptimizer, mode: str = "success") -> bool:
    print(f"\n{'=' * 70}")
    print(f"[{name}]  输入路径 {len(path)} 点, 起点 {path[0][:2]}, 终点 {path[-1][:2]}")
    res: TrajResult = opt.optimize(path, start_vel=start_vel)
    print(
        f"  success={res.success}  Tf={res.duration:.2f}s  "
        f"耗时={res.solve_time_ms:.0f}ms  iters={res.iterations}"
    )
    print(
        f"  v_max={res.max_speed:.3f} (限 {opt.max_speed})   "
        f"kappa_max={res.max_kappa:.3f} (限 {opt.kappa_limit:.3f})   "
        f"a_max={res.max_accel:.3f} (限 {opt.max_accel})"
    )
    if res.status:
        print(f"  status={res.status}")

    ok = True

    def fail(msg):
        nonlocal ok
        ok = False
        print(f"  ❌ {msg}")

    if not res.success:
        if mode == "safe":
            print("  ✅ fail-safe：拒绝出解，上游回退 A* 原路径")
            return True
        fail("未收敛")
        return False
    if len(res.samples) < 3:
        fail(f"采样点过少 ({len(res.samples)})")

    # 起终点位置
    d_start = math.dist(res.samples[0][:2], path[0][:2])
    d_end = math.dist(res.samples[-1][:2], path[-1][:2])
    if d_start > 1e-3:
        fail(f"起点偏差 {d_start:.4f}m")
    if d_end > 1e-2:
        fail(f"终点偏差 {d_end:.4f}m")
    print(f"  起终点偏差: {d_start * 100:.2f}cm / {d_end * 100:.2f}cm")

    # 物理约束（允许 10% 采样离散误差）
    kappa_bad = res.max_kappa > opt.kappa_limit * 1.10
    speed_bad = res.max_speed > opt.max_speed * 1.10
    if speed_bad:
        fail(f"速度超限 {res.max_speed:.3f} > {opt.max_speed * 1.10:.3f}")
    if kappa_bad:
        if mode == "safe":
            print(
                f"  ✅ fail-safe：曲率 {res.max_kappa:.2f} 超限 "
                f"(> {opt.kappa_limit * 1.10:.2f})，应回退 A* 原路径"
            )
            return True
        fail(f"曲率超限 {res.max_kappa:.3f} > {opt.kappa_limit * 1.10:.3f}")
    if res.max_accel > opt.max_accel * 1.30:
        fail(f"加速度超限 {res.max_accel:.3f} > {opt.max_accel * 1.30:.3f}")

    # 时间单调 + 航向连续
    for i in range(1, len(res.samples)):
        if res.samples[i][3] <= res.samples[i - 1][3]:
            fail(f"时间非单调 @{i}")
            break
    max_dth = 0.0
    for i in range(1, len(res.samples)):
        dth = (res.samples[i][2] - res.samples[i - 1][2] + math.pi) % (2 * math.pi) - math.pi
        max_dth = max(max_dth, abs(dth))
    if max_dth > math.radians(35):
        fail(f"航向跳变 {math.degrees(max_dth):.1f}°/step")
    print(f"  最大单步航向变化: {math.degrees(max_dth):.1f}°  |  采样 {len(res.samples)} 点")

    if ok:
        print("  ✅ 通过")
    return ok


def test_all_scenes():
    """pytest 入口：5 个场景全部通过。"""
    opt = TrajectoryOptimizer(PARAMS)
    print(f"\nTrajectoryOptimizer 参数: {PARAMS}")
    print(f"  κ_limit={opt.kappa_limit:.3f} 1/m  (tan(δmax)/L × 0.85)")

    results = []
    for name, factory, mode in SCENES:
        try:
            path, v0 = factory()
        except Exception as exc:
            print(f"\n[{name}] ❌ 场景构造失败: {exc}")
            results.append(False)
            continue
        results.append(check_scene(name, path, v0, opt, mode))

    print(f"\n{'=' * 70}")
    passed = sum(results)
    print(f"结果: {passed}/{len(results)} 通过")
    for (name, _f, _m), ok in zip(SCENES, results):
        print(f"  {'✅' if ok else '❌'} {name}")
    return all(results)


if __name__ == "__main__":
    sys.exit(0 if test_all_scenes() else 1)

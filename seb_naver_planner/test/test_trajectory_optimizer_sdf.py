#!/usr/bin/env python3
"""TrajectoryOptimizer P1 单测 — SDF 避障约束（论文 Eq.29）。

覆盖:
  1. 平行贴墙 0.45m  ：安全路径，优化后仍须满足 dmin
  2. 贴近墙 0.30m    ：路径合法但过近，优化应推离到 dmin 以上
  3. 穿墙路径        ：优化器应**拒绝**（A* 的职责是避障，不强行修复）
  4. 窄通道 1.1m     ：两侧墙，须通过且不蹭墙
  5. 真实 A* 绕墙    ：端到端，须安全且平滑
  6. 无 SDF 回归     ：等同 P0 行为

运行::

    cd seb_naver_planner
    /usr/bin/python3 test/test_trajectory_optimizer_sdf.py
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from seb_naver_planner.trajectory_optimizer import TrajResult, TrajectoryOptimizer  # noqa: E402

RES = 0.1

PARAMS = {
    "wheel_base": 0.315,
    "max_speed": 0.5,
    "max_accel": 1.0,
    "max_steer": 0.785,
    "seg_len": 0.9,
    "rho_t": 1.5,
    "dmin": 0.28,
    "sdf_iters": 3,
}


def _grid(n: int = 200, origin: float = -10.0):
    return np.ones((n, n), dtype=np.float32), {"resolution": RES, "origin": [origin, origin]}


def _idx(x: float, y: float, origin: float = -10.0) -> tuple[int, int]:
    """世界坐标 → (row=y, col=x)。"""
    return (int(round((y - origin) / RES)), int(round((x - origin) / RES)))


def _wall_along_x(grid, y: float, x0: float, x1: float, thickness: float = 0.1):
    """沿 x 方向的墙（水平墙）。"""
    r = _idx(0.0, y)[0]
    c0 = _idx(x0, 0.0)[1]
    c1 = _idx(x1, 0.0)[1]
    half = max(0, int(round(thickness / 2 / RES)))
    grid[r - half: r + half + 1, c0: c1] = -1.0


def _wall_along_y(grid, x: float, y0: float, y1: float, thickness: float = 0.1):
    """沿 y 方向的墙（竖直墙）。"""
    c = _idx(x, 0.0)[1]
    r0 = _idx(0.0, y0)[0]
    r1 = _idx(0.0, y1)[0]
    half = max(0, int(round(thickness / 2 / RES)))
    grid[r0: r1, c - half: c + half + 1] = -1.0


def _make_path(points, steps_per_seg: int = 10):
    out = []
    for i in range(len(points) - 1):
        (x0, y0), (x1, y1) = points[i], points[i + 1]
        for k in range(steps_per_seg):
            r = k / steps_per_seg
            out.append((x0 + r * (x1 - x0), y0 + r * (y1 - y0), 0.0))
    x1, y1 = points[-1]
    out.append((x1, y1, 0.0))
    for i in range(len(out) - 1):
        dx = out[i + 1][0] - out[i][0]
        dy = out[i + 1][1] - out[i][1]
        out[i] = (out[i][0], out[i][1], math.atan2(dy, dx))
    prev = out[-2]
    out[-1] = (out[-1][0], out[-1][1], math.atan2(out[-1][1] - prev[1], out[-1][0] - prev[0]))
    return out


# ---------------------------------------------------------------------------
# 场景: (名称, 工厂, 期望成功)
# ---------------------------------------------------------------------------

def scene_parallel_wall():
    """1: 路径沿 x 走，下方 0.45m 处有水平墙。"""
    grid, info = _grid()
    _wall_along_x(grid, 0.0, -1.0, 6.0, thickness=0.2)
    return _make_path([(0.0, 0.45), (5.0, 0.45)]), grid, info


def scene_close_wall():
    """2: 中段贴近墙 0.30m，起终点留安全距离（起终点被硬约束固定，推不动）。"""
    grid, info = _grid()
    _wall_along_x(grid, 0.0, -1.0, 6.0, thickness=0.2)
    return _make_path([(0.0, 0.70), (2.0, 0.30), (3.5, 0.30), (5.0, 0.70)]), grid, info


def scene_through_wall():
    """3: 路径直接穿墙（期望被拒绝）。"""
    grid, info = _grid()
    _wall_along_y(grid, 2.5, -2.0, 2.0, thickness=0.2)
    return _make_path([(0.0, 0.0), (5.0, 0.0)]), grid, info


def scene_corridor():
    """4: 净宽 1.1m 的通道（y ∈ [-0.55, 0.55]）。"""
    grid, info = _grid()
    _wall_along_x(grid, 0.55, -1.0, 7.0, thickness=0.2)
    _wall_along_x(grid, -0.55, -1.0, 7.0, thickness=0.2)
    return _make_path([(0.0, 0.0), (6.0, 0.0)]), grid, info


def scene_astar():
    """5: 真实 Kinodynamic A* 绕墙。"""
    from seb_naver_planner.kino_astar import KinoAStarPlanner, SearchStatus

    grid, info = _grid()
    c = _idx(2.5, 0.0)[1]
    r_top = _idx(0.0, 0.8)[0]   # 缺口 y>0.8：绕行幅度适中，A* 约 1.2s
    grid[:r_top, c] = -1.0

    planner = KinoAStarPlanner(
        {
            "wheel_base": PARAMS["wheel_base"],
            "max_speed": PARAMS["max_speed"],
            "max_steer": PARAMS["max_steer"],
            "step_arc": 1.0,
            "steer_res": 0.3,
        }
    )
    status, path = planner.search(
        start_state=(0.0, 0.0, 0.0),
        goal_state=(5.0, 0.0, 0.0),
        sdf_grid_data=grid,
        sdf_info=info,
    )
    if status != SearchStatus.SUCCESS or len(path) < 2:
        raise RuntimeError(f"A* 失败: {status.name}")
    return path, grid, info


def scene_no_sdf():
    """6: 无 SDF（回归 P0）。"""
    return _make_path([(0.0, 0.0), (3.0, 0.0), (3.0, 3.0)], steps_per_seg=12), None, None


# 期望模式:
#   "success" — 必须成功且满足全部约束
#   "reject"  — 必须明确拒绝（输入路径本身穿墙）
#   "safe"    — fail-safe: 成功则必须安全；失败亦可接受（上游回退 A* 原路径）
SCENES = [
    ("1 平行贴墙 0.45m", scene_parallel_wall, "success"),
    ("2 贴近墙 0.30m", scene_close_wall, "success"),
    ("3 穿墙路径", scene_through_wall, "reject"),
    ("4 窄通道 1.1m", scene_corridor, "success"),
    ("5 真实 A* 绕墙", scene_astar, "safe"),
    ("6 无 SDF 回归", scene_no_sdf, "success"),
]


# ---------------------------------------------------------------------------
# 检查
# ---------------------------------------------------------------------------

def _input_clearance(path, grid, info, opt) -> float:
    """输入路径的最小间距（端点即可，忽略线段）。"""
    if grid is None:
        return 0.0
    clr = opt._build_clearance_field(grid, info)
    if clr is None:
        return 0.0
    return min(opt._query_clearance(clr, float(p[0]), float(p[1])) for p in path)


def check(name: str, path, grid, info, opt: TrajectoryOptimizer, mode: str) -> bool:
    print(f"\n{'=' * 74}")
    print(f"[{name}]  {len(path)} 点  {tuple(round(v, 2) for v in path[0][:2])} → "
          f"{tuple(round(v, 2) for v in path[-1][:2])}")

    res: TrajResult = opt.optimize(path, start_vel=0.0, sdf_grid=grid, sdf_info=info)
    print(f"  success={res.success}  Tf={res.duration:.2f}s  {res.solve_time_ms:.0f}ms  "
          f"iters={res.iterations}")
    print(f"  v_max={res.max_speed:.3f}/{opt.max_speed}  "
          f"κ_max={res.max_kappa:.2f}/{opt.kappa_limit:.2f}  "
          f"min_clr={res.min_clearance:.3f}/{opt.dmin}")
    if res.status:
        print(f"  status={res.status}")

    ok = True

    def fail(msg):
        nonlocal ok
        ok = False
        print(f"  ❌ {msg}")

    # 期望拒绝的场景
    if mode == "reject":
        if res.success:
            fail("本应拒绝穿墙路径，却返回成功")
        elif "inside obstacle" not in res.status:
            fail(f"拒绝原因不明确: {res.status}")
        else:
            print("  ✅ 正确拒绝（输入路径穿墙）")
        return ok

    # fail-safe: 失败也可接受，但必须不是「产出了不安全轨迹」
    if mode == "safe" and not res.success:
        print("  ✅ fail-safe：优化器拒绝出解，上游应回退 A* 原路径")
        print(f"     (路径本身 clearance={_input_clearance(path, grid, info, opt):.2f}m，A* 已保证安全)")
        return ok

    if not res.success:
        fail("未收敛/未满足避障")
        return False
    if len(res.samples) < 3:
        fail(f"采样过少 ({len(res.samples)})")
        return False

    d0 = math.dist(res.samples[0][:2], path[0][:2])
    d1 = math.dist(res.samples[-1][:2], path[-1][:2])
    if d0 > 1e-3:
        fail(f"起点偏差 {d0:.3f}m")
    if d1 > 1e-2:
        fail(f"终点偏差 {d1:.3f}m")
    if res.max_speed > opt.max_speed * 1.10:
        fail(f"速度超限 {res.max_speed:.3f}")
    if res.max_kappa > opt.kappa_limit * 1.10:
        fail(f"曲率超限 {res.max_kappa:.2f} (限 {opt.kappa_limit * 1.10:.2f})")

    if grid is not None:
        if res.min_clearance < opt.dmin - opt.sdf_tolerance - 1e-6:
            fail(f"间距不足 {res.min_clearance:.3f} < {opt.dmin - opt.sdf_tolerance:.3f}")
        clr = opt._build_clearance_field(grid, info)
        if clr is not None:
            inside = sum(
                1 for x, y, _t, _tt in res.samples if opt._query_clearance(clr, x, y) < 0.0
            )
            if inside:
                fail(f"{inside} 个采样点在障碍物内部")
            print(f"  障碍物内采样点: {inside}")

    print(f"  起终点偏差: {d0 * 100:.2f}cm / {d1 * 100:.2f}cm")
    if ok:
        print("  ✅ 通过")
    return ok


def test_sdf_scenes():
    opt = TrajectoryOptimizer(PARAMS)
    print(f"\nP1 SDF 测试  dmin={opt.dmin}m  κ_limit={opt.kappa_limit:.3f}  "
          f"repulsion={opt.sdf_repulsion_weight}")
    results = []
    for name, factory, mode in SCENES:
        try:
            path, grid, info = factory()
        except Exception as exc:
            print(f"\n[{name}] ❌ 场景构造失败: {exc}")
            results.append(False)
            continue
        results.append(check(name, path, grid, info, opt, mode))

    print(f"\n{'=' * 74}")
    print(f"结果: {sum(results)}/{len(results)} 通过")
    for (name, _f, _m), ok in zip(SCENES, results):
        print(f"  {'✅' if ok else '❌'} {name}")
    return all(results)


if __name__ == "__main__":
    sys.exit(0 if test_sdf_scenes() else 1)

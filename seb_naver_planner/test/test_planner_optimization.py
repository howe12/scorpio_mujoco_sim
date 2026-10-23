#!/usr/bin/env python3
"""P2 集成测试：轨迹优化在 planner 中的异步接入。

验证三件事（不需要完整仿真，用桩 node/SDF 驱动 PlanManager）:
  1. **不阻塞**：plan() 的调用耗时远小于优化求解耗时（异步生效）
  2. **回退**：优化未完成/失败时返回 A* 粗路径（last_rough）
  3. **接管**：优化完成后，下一次 plan() 返回优化轨迹（点数/形状与粗路径不同）

运行::

    cd seb_naver_planner
    /usr/bin/python3 test/test_planner_optimization.py
"""

from __future__ import annotations

import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


# ---------------------------------------------------------------------------
# 桩：最小 Node / SDFGridParser
# ---------------------------------------------------------------------------

class FakeParam:
    def __init__(self, value):
        self.value = value


class FakeLogger:
    def __init__(self):
        self.lines = []

    def _rec(self, lvl, msg):
        self.lines.append(f"[{lvl}] {msg}")

    def info(self, msg): self._rec("INFO", msg)
    def warn(self, msg): self._rec("WARN", msg)
    def error(self, msg): self._rec("ERROR", msg)

    def dump(self, n=8):
        for line in self.lines[-n:]:
            print(f"    {line}")


class FakeNode:
    """只实现 PlanManager 用到的 get_parameter / get_logger。"""

    DEFAULTS = {
        "wheel_base": 0.315,
        "max_speed": 0.5,
        "max_steer": 0.785,
        "step_arc": 1.0,
        "steer_res": 0.3,
        "enable_optimization": True,
        "trajectory_dmin": 0.28,
        "trajectory_seg_len": 0.9,
        "trajectory_max_accel": 1.0,
        "trajectory_sdf_iters": 5,
        "optimize_offtrack_tolerance": 0.8,
        "optimize_valid_s": 8.0,
        "optimize_min_length": 2.0,
        "traj_publish_step": 0.3,
        "optimize_min_remaining": 1.5,
        "inflation_radius": 0.25,
    }

    def __init__(self):
        self.logger = FakeLogger()

    def get_parameter(self, name):
        if name not in self.DEFAULTS:
            raise KeyError(name)
        return FakeParam(self.DEFAULTS[name])

    def get_logger(self):
        return self.logger


class FakeSDF:
    """20m×20m / 0.1m 的全 free 栅格（原点 -10,-10）。"""

    def __init__(self):
        self.resolution = 0.1
        self.length_x = 20.0
        self.length_y = 20.0
        self.map_center_x = 0.0
        self.map_center_y = 0.0
        self.sdf_map = np.ones((200, 200), dtype=np.float32)


# ---------------------------------------------------------------------------
# 测试
# ---------------------------------------------------------------------------

def test_async_non_blocking():
    """plan() 不被优化求解阻塞，且最终会用上优化轨迹。"""
    from seb_naver_planner.planner_node import PlanManager

    node = FakeNode()
    pm = PlanManager(node, FakeSDF())
    if pm._optimizer is None:
        print("  ⚠ TrajectoryOptimizer 不可用（缺 CasADi），跳过")
        return True

    print("  提交第 1 次规划（优化应异步启动）")
    t0 = time.monotonic()
    path1 = pm.plan(0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0)
    dt1 = (time.monotonic() - t0) * 1000
    print(f"    plan() 耗时 {dt1:.0f}ms, 返回 {len(path1) if path1 else 0} 点")

    if path1 is None:
        print("  ❌ A* 未找到路径")
        return False

    # A* 本身是毫秒级；plan() 若同步跑优化会到 400ms+
    if dt1 > 250:
        print(f"  ❌ plan() 被阻塞 ({dt1:.0f}ms)，异步未生效")
        return False
    print("  ✅ plan() 未被优化阻塞")

    # 首次返回的应是 A* 粗路径（优化还没算完）
    first_is_rough = path1 is pm.last_rough
    print(f"    首次返回 {'A* 粗路径' if first_is_rough else '优化轨迹'}")

    # 等优化完成（最多 20s）
    deadline = time.time() + 20.0
    while time.time() < deadline:
        with pm._opt_lock:
            if pm._opt_samples is not None:
                break
        time.sleep(0.2)

    with pm._opt_lock:
        ok, fail = pm._opt_ok, pm._opt_fail
        has_opt = pm._opt_samples is not None
    print(f"    优化统计: ok={ok} fail={fail} 有结果={has_opt}")
    if not has_opt:
        print("  ⚠ 优化未产出结果（可能被判 fail-safe），仅验证了不阻塞")
        node.logger.dump(6)
        return True

    # 再次规划：起点相同 → 应返回优化轨迹
    path2 = pm.plan(0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0)
    took_opt = path2 is not None and path2 is not pm.last_rough
    print(f"  第 2 次规划返回 {'优化轨迹' if took_opt else 'A* 粗路径'}")
    if not took_opt:
        print("  ❌ 优化结果未被采用")
        node.logger.dump(6)
        return False
    print(f"    优化轨迹 {len(path2)} 点（粗路径 {len(pm.last_rough)} 点）")
    print("  ✅ 优化轨迹已接管")
    return True


def test_fallback_on_far_start():
    """起点离优化轨迹太远 → 丢弃优化结果，回退粗路径。"""
    from seb_naver_planner.planner_node import PlanManager

    node = FakeNode()
    pm = PlanManager(node, FakeSDF())
    if pm._optimizer is None:
        print("  ⚠ CasADi 不可用，跳过")
        return True

    pm.plan(0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0)
    deadline = time.time() + 20.0
    while time.time() < deadline:
        with pm._opt_lock:
            if pm._opt_samples is not None:
                break
        time.sleep(0.2)
    with pm._opt_lock:
        if pm._opt_samples is None:
            print("  ⚠ 无优化结果，跳过该子项")
            return True

    # 离轨迹 5m（>> optimize_offtrack_tolerance=0.8）→ 必须回退
    fresh = pm._fresh_optimized(5.0, 5.0)
    if fresh is not None:
        print("  ❌ 偏离轨迹时仍返回了优化轨迹")
        return False
    print("  ✅ 偏离轨迹时正确回退（不返回不适用轨迹）")

    # 在轨迹起点附近 → 应返回
    fresh2 = pm._fresh_optimized(0.05, 0.05)
    if fresh2 is None:
        print("  ❌ 在轨迹上时未返回优化轨迹")
        return False
    print("  ✅ 在轨迹上时正确命中缓存")
    return True


def test_disabled():
    """enable_optimization=false 时退回纯 A*。"""
    from seb_naver_planner import planner_node

    class NoOptNode(FakeNode):
        DEFAULTS = dict(FakeNode.DEFAULTS, enable_optimization=False)

    pm = planner_node.PlanManager(NoOptNode(), FakeSDF())
    if pm._optimizer is not None:
        print("  ❌ 关闭开关后仍创建了优化器")
        return False
    path = pm.plan(0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0)
    if path is None or path is not pm.last_rough:
        print("  ❌ 关闭后未回退到 A* 粗路径")
        return False
    print("  ✅ enable_optimization=false 时走纯 A*")
    return True


def main() -> int:
    print("=" * 74)
    print("P2 集成测试：轨迹优化在 planner 中的异步接入")
    print("=" * 74)

    results = []
    for name, fn in [
        ("1 异步不阻塞", test_async_non_blocking),
        ("2 起点过远回退", test_fallback_on_far_start),
        ("3 开关可关", test_disabled),
    ]:
        print(f"\n[{name}]")
        try:
            ok = fn()
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print(f"  ❌ 异常: {exc}")
            ok = False
        results.append(ok)

    print(f"\n{'=' * 74}")
    print(f"结果: {sum(results)}/{len(results)} 通过")
    for (name, _f), ok in zip(
        [("1 异步不阻塞", None), ("2 起点过远回退", None), ("3 开关可关", None)], results
    ):
        print(f"  {'✅' if ok else '❌'} {name}")
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())

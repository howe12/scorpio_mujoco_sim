#!/usr/bin/env python3
"""P3 单测：地形姿态约束（论文 Eq.27 的 φx/φy）。

验证:
  1. 平地         : 无不可通行区、姿态为 0
  2. 缓坡 (20°)   : 坡度阈值(25°)以下 → 可通行，但俯仰超 20° 会被验收拦下
  3. 陡坡 (35°)   : 标为不可通行（A* 应绕开）
  4. 横向斜坡     : 横滚角正确计算，且比俯仰更早触发（车体窄）
  5. 未观测区域   : upper-lower 过大 → 不当成陡坡（否则地图边缘变墙）
  6. 圆形缓冲     : start_index 偏移正确还原

运行::

    cd seb_naver_planner
    /usr/bin/python3 test/test_terrain_attitude.py
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from seb_naver_planner.planner_node import ElevationGridParser  # noqa: E402

RES = 0.1
N = 100          # 10m x 10m
CENTER = 0.0


def _parser(slope_x=0.0, slope_y=0.0, valid=True, z0=0.0) -> ElevationGridParser:
    """造一个均匀倾斜的地形：z = z0 + slope_x·x + slope_y·y。

    slope 用「每米上升量」表示（tan），例如 slope=tan(20°)=0.364。
    """
    p = ElevationGridParser()
    p.resolution = RES
    p.rows = p.cols = N
    p.length_x = p.length_y = N * RES
    p.map_center_x = p.map_center_y = CENTER

    xs = (np.arange(N) - N / 2) * RES
    ys = (np.arange(N) - N / 2) * RES
    gx, gy = np.meshgrid(xs, ys)
    p.elevation = (z0 + slope_x * gx + slope_y * gy).astype(np.float32)

    if valid:
        p.valid = np.ones((N, N), dtype=bool)
    else:
        p.valid = np.zeros((N, N), dtype=bool)
    p.updated = True
    return p


def check(name, cond, detail=""):
    if cond:
        print(f"  ✅ {name} {detail}")
    else:
        print(f"  ❌ {name} {detail}")
    return cond


def test_flat() -> bool:
    print("\n[1 平地]")
    p = _parser()
    mask = p.untraversable_mask(math.radians(25))
    att = p.attitude_at(0.0, 0.0, 0.0)
    ok = True
    ok &= check("无不可通行区", not mask.any(), f"(mask={int(mask.sum())})")
    ok &= check("姿态为 0", att is not None and abs(att[0]) < 1e-6 and abs(att[1]) < 1e-6,
                f"(pitch={math.degrees(att[0]):.2f}°, roll={math.degrees(att[1]):.2f}°)" if att else "")
    return ok


def test_gentle_slope() -> bool:
    print("\n[2 缓坡 20°]")
    tan20 = math.tan(math.radians(20.0))
    p = _parser(slope_x=tan20)
    mask = p.untraversable_mask(math.radians(25))
    # 沿 +x 车头朝 0 → pitch = 20°
    att = p.attitude_at(0.0, 0.0, 0.0)
    ok = True
    ok &= check("坡度 20° < 25° 可通行", not mask.any(), f"(mask={int(mask.sum())})")
    if att is None:
        return False
    pitch_deg = math.degrees(att[0])
    ok &= check("俯仰 ≈ 20°", abs(pitch_deg - 20.0) < 1.0, f"({pitch_deg:.2f}°)")
    ok &= check("俯仰超 20° 阈值会被验收拦下", abs(att[0]) >= math.radians(20.0) - 1e-3,
                "(需要 max_pitch_deg >= 21 才放行)")
    return ok


def test_steep_slope() -> bool:
    print("\n[3 陡坡 35°]")
    tan35 = math.tan(math.radians(35.0))
    p = _parser(slope_x=tan35)
    mask = p.untraversable_mask(math.radians(25))
    frac = mask.sum() / mask.size
    ok = check("大部分区域标为不可通行", frac > 0.5, f"(占比 {frac * 100:.0f}%)")
    return ok


def test_lateral() -> bool:
    print("\n[4 横向斜坡]")
    tan15 = math.tan(math.radians(15.0))
    # 沿 +y 倾斜，车头朝 +x → 地形坡在车的侧向 → 产生横滚
    p = _parser(slope_y=tan15)
    att = p.attitude_at(0.0, 0.0, 0.0)
    if att is None:
        return False
    pitch, roll = att
    ok = True
    ok &= check("俯仰 ≈ 0", abs(math.degrees(pitch)) < 1.0, f"({math.degrees(pitch):.2f}°)")
    ok &= check("横滚 ≈ 15°", abs(abs(math.degrees(roll)) - 15.0) < 1.0,
                f"({math.degrees(roll):.2f}°)")
    # 车体窄，roll 阈值 15° 比 pitch 20° 更敏感
    ok &= check("横滚恰好触及 15° 阈值", abs(roll) >= math.radians(15.0) - 1e-3)
    return ok


def test_unobserved() -> bool:
    print("\n[5 未观测区域]")
    p = _parser()
    # 模拟未观测: 一半区域 valid=False，elevation 保留陡峭的默认值
    p.elevation[: N // 2, :] = 100.0
    p.valid[: N // 2, :] = False
    mask = p.untraversable_mask(math.radians(25))
    upper_half = mask[: N // 2, :].sum()
    ok = check("未观测区不算陡坡", upper_half == 0, f"(误判 {int(upper_half)} 格)")
    return ok


def test_circular_buffer() -> bool:
    print("\n[6 圆形缓冲]")
    # 构造一个 start_index 非零的消息，验证数据被正确 roll 回
    p = ElevationGridParser()
    rows = cols = 10
    z = np.arange(rows * cols, dtype=np.float32).reshape(rows, cols)

    class FakeInfo:
        pos_resolution = 0.1
        length_x = 1.0
        length_y = 1.0

        class pose:
            class position:
                x = 0.0
                y = 0.0

    class FakeData:
        def __init__(self, arr):
            self.data = arr.ravel().tolist()

    class FakeMsg:
        layers = ["elevation", "lower_bound", "upper_bound"]
        info = FakeInfo()
        start_index_row = 3
        start_index_col = 4

        def __init__(self):
            d = [FakeData(z), FakeData(z - 0.01), FakeData(z + 0.01)]
            self.data = d

    p.update_from_msg(FakeMsg())
    if p.elevation is None:
        print("  ❌ 解析失败")
        return False
    expect = np.roll(np.roll(z, -3, axis=0), -4, axis=1)
    ok = check("圆形缓冲还原正确", np.allclose(p.elevation, expect))
    ok &= check("valid 全为真", p.valid is not None and p.valid.all())
    return ok


def main() -> int:
    print("=" * 70)
    print("P3 单测：地形姿态约束 (φx/φy)")
    print("=" * 70)
    results = []
    for fn in [
        ("1 平地", test_flat),
        ("2 缓坡 20°", test_gentle_slope),
        ("3 陡坡 35°", test_steep_slope),
        ("4 横向斜坡", test_lateral),
        ("5 未观测区域", test_unobserved),
        ("6 圆形缓冲", test_circular_buffer),
    ]:
        try:
            results.append(fn[1]())
        except Exception as exc:
            import traceback
            traceback.print_exc()
            print(f"  ❌ 异常: {exc}")
            results.append(False)

    print(f"\n{'=' * 70}")
    print(f"结果: {sum(results)}/{len(results)} 通过")
    names = ["1 平地", "2 缓坡 20°", "3 陡坡 35°", "4 横向斜坡", "5 未观测区域", "6 圆形缓冲"]
    for nm, ok in zip(names, results):
        print(f"  {'✅' if ok else '❌'} {nm}")
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())

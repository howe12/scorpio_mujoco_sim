#!/usr/bin/env python3
"""SE(2) 轨迹优化器 — SEB-Naver 论文 §VI-B 的 PHR-ALM 轨迹优化（Python/CasADi 移植）。

论文把局部轨迹优化建模为非线性规划（Eq.22）：

    min  ∫‖j(t)‖²dt + ρt·Tf + ρr·∫Risk²dt
    s.t. 五次分段多项式连续性与边界条件      (Eq.23-24)
         ẋ²(s)+ẏ²(s) ≥ δ₊,  Tf > 0           (Eq.25)
         δ² ≤ δmax²,  vx² ≤ vmax²            (Eq.26)
         ax² ≤ amax_lon²,  ay² ≤ amax_lat²    (Eq.27)
         Risk(sr(t)) ≤ rmax                  (Eq.28)
         Gc(sr(t)) ≥ dmin   (SDF 避障)        (Eq.29)

论文用 PHR-ALM（Powell-Hestenes-Rockafellar 增广拉格朗日）求解。本移植用 CasADi
的 IPOPT（内点法）解同一个 NLP：论文规模是 SE(2) 大规模问题 + CUDA，本工程是
N≤10 段、约 120 变量的中等规模问题，IPOPT 收敛更稳且无需手写 ALM 迭代。

与论文的差异（本实现范围）:
  - 优化变量用多项式系数 (CX, CY) + Tf，不做 Eq.23 的 Mp/Ms 消元（消元需推导
    6N-2 维约束矩阵，易错；直接优化系数等价且更易调试）。
  - 不含中间变量 s(t) 与换挡点 em（本工程 max_speed 0.5 m/s，鲜少换挡）。
  - SDF 避障（Eq.29）与 Risk 项（Eq.22 第三项 / Eq.28）留待 P1/P4，接口已预留。

坐标与单位: 世界系 (camera_init) 米/弧度；时间秒。
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

try:
    import casadi as ca
except ImportError as exc:  # pragma: no cover - 环境缺 CasADi 时给出明确指引
    raise ImportError(
        "需要 CasADi: /usr/bin/python3 -m pip install casadi==3.8.1"
    ) from exc


# ---------------------------------------------------------------------------
# 数据结构
# ---------------------------------------------------------------------------

@dataclass
class TrajResult:
    """优化结果。"""

    samples: List[Tuple[float, float, float, float]] = field(default_factory=list)
    """等时间间隔采样 [(x, y, theta, t)]，theta 为航向角 (rad)。"""

    duration: float = 0.0
    """总时长 Tf (s)。"""

    success: bool = False
    """是否收敛。False 时 samples 为空，调用方应回退到 A* 粗路径。"""

    solve_time_ms: float = 0.0
    """求解耗时 (ms)。"""

    max_speed: float = 0.0
    """采样点上的最大线速度 (m/s)。"""

    max_kappa: float = 0.0
    """采样点上的最大曲率 (1/m)。"""

    max_accel: float = 0.0
    """采样点上的最大加速度 (m/s²)。"""

    iterations: int = 0
    """IPOPT 迭代次数。"""

    status: str = ""
    """求解器返回状态字符串。"""

    min_clearance: float = 0.0
    """采样点上的最小障碍物间距 (m)。未提供 SDF 时为 0。"""


# ---------------------------------------------------------------------------
# 优化器
# ---------------------------------------------------------------------------

class TrajectoryOptimizer:
    """基于五次分段多项式的 SE(2) 轨迹优化。

    对给定路径（通常来自 Kinodynamic A*）求最小 jerk 的平滑轨迹，同时满足
    转向角、速度、加速度与曲率限制。

    典型用法::

        opt = TrajectoryOptimizer({'wheel_base': 0.315, 'max_speed': 0.5})
        result = opt.optimize(astar_path, start_vel=0.3)
        if result.success:
            poses = [(x, y, th) for x, y, th, _ in result.samples]
    """

    #: 目标函数中时间项的权重 ρt。越大越倾向快（论文 Eq.22 的 ρt）。
    DEFAULT_RHO_T = 1.5

    #: 每段半长的目标弧长 (m)，用于按路径长度决定分段数。
    DEFAULT_SEG_LEN = 0.9

    #: 输出采样时间间隔 (s)。
    DEFAULT_SAMPLE_DT = 0.1

    def __init__(self, params: Optional[Dict] = None) -> None:
        p = params or {}
        self.wheel_base: float = float(p.get("wheel_base", 0.315))
        self.max_speed: float = float(p.get("max_speed", 0.5))
        self.max_accel: float = float(p.get("max_accel", 1.0))
        self.max_steer: float = float(p.get("max_steer", 0.785))

        self.min_segments: int = int(p.get("min_segments", 3))
        self.max_segments: int = int(p.get("max_segments", 10))
        self.seg_len: float = float(p.get("seg_len", self.DEFAULT_SEG_LEN))
        # Douglas-Peucker 简化容差 (m)：保留给 _dp_simplify 的兼容参数
        self.dp_eps: float = float(p.get("dp_eps", 0.08))
        # 曲率加权强度：弯道段长相对直线段的压缩比
        self.curvature_weight: float = float(p.get("curvature_weight", 3.0))

        self.rho_t: float = float(p.get("rho_t", self.DEFAULT_RHO_T))
        # 曲率使用裕度：优化到 0.85·κmax，给 MPC 跟踪留余量
        self.kappa_limit: float = (
            math.tan(self.max_steer) / self.wheel_base
        ) * float(p.get("kappa_margin", 0.85))

        self.quad_points: int = int(p.get("quad_points", 4))    # Simpson 求积点数(偶数)
        self.constraint_samples: int = int(p.get("constraint_samples", 6))  # 每段约束采样
        self.sample_dt: float = float(p.get("sample_dt", self.DEFAULT_SAMPLE_DT))

        self.max_iter: int = int(p.get("max_iter", 600))
        self.tol: float = float(p.get("tol", 5e-2))
        # 是否把分段点作为硬约束经过（P0 默认 True，保证跟住 A* 的几何）
        # 分段点约束模式: auto | hard | soft | none
        #   auto: 无 SDF → hard（跟住 A* 几何）; 有 SDF → soft（让几何为避障让路）
        self.follow_mode: str = str(p.get("follow_mode", "auto"))
        self._follow_mode_opt = p.get("follow_path", None)  # 兼容旧参数
        # _solve 在未显式传入 follow_hard 时的默认行为
        self.follow_path: bool = bool(
            self._follow_mode_opt if self._follow_mode_opt is not None
            else self.follow_mode in ("hard", "auto")
        )
        # 软引导权重（follow_mode=soft 时使用）
        # 有障碍物时用中等权重，避免平滑过程切角侵入障碍；
        # 无障碍物时用很低的权重，让优化器自由圆化尖角（κ 才不会超限）。
        self.path_guidance: float = float(p.get("path_guidance", 4.0))
        self.path_guidance_no_sdf: float = float(p.get("path_guidance_no_sdf", 0.5))

        # --- 避障 (论文 Eq.29: Gc(sr) ≥ dmin) ---
        # 车体碰撞盒半宽 0.211m + 余量
        self.dmin: float = float(p.get("dmin", 0.28))
        # 距离场截断：超过此距离梯度无意义，也限制插值表规模
        self.sdf_dmax: float = float(p.get("sdf_dmax", 1.0))
        # 每段施加 SDF 约束的采样点数（比动力学约束更密：障碍物是 0.1m 栅格）
        self.sdf_samples: int = int(p.get("sdf_samples", 10))
        # 距离场缓存：(内容哈希, 距离场 dict)
        self._sdf_cache: Optional[Tuple] = None
        self._last_warning: str = ""
        # outside-in 迭代次数与排斥代价权重
        self.sdf_iters: int = int(p.get("sdf_iters", 5))
        self.sdf_repulsion_weight: float = float(p.get("sdf_repulsion_weight", 60.0))
        # 最终验收容差（间距允许比 dmin 小这么多仍判成功）
        self.sdf_tolerance: float = float(p.get("sdf_tolerance", 0.05))

    # ------------------------------------------------------------------
    # 公共接口
    # ------------------------------------------------------------------

    def optimize(
        self,
        astar_path: Sequence[Sequence[float]],
        start_vel: float = 0.0,
        sdf_grid: Optional[np.ndarray] = None,
        sdf_info: Optional[Dict] = None,
    ) -> TrajResult:
        """求解最优轨迹。

        Args:
            astar_path: 路径点序列，元素为 ``(x, y, theta)`` 或
                ``(x, y, theta, singular)``（Kinodynamic A* 的输出格式）。
            start_vel: 起点线速度 (m/s)，用于起点速度约束。
            sdf_grid: 可选 SDF 栅格（P1 阶段用于避障约束，当前忽略）。
            sdf_info: 可选 SDF 元数据 ``{'resolution', 'origin': [x, y]}``。

        Returns:
            TrajResult。``success=False`` 时 ``samples`` 为空。
        """
        # 预留 SDF 参数（P1 实现避障约束）
        self._sdf_grid = sdf_grid
        self._sdf_info = sdf_info

        t_start = time.monotonic()

        pts = self._prepare_path(astar_path)
        if pts is None:
            return TrajResult(success=False, status="invalid path")

        waypoints, n_seg = pts
        # Tf 初值：按 0.8·max_speed 估计（偏快但可行，给 IPOPT 好的起点）
        total_len = self._path_length(waypoints)
        v_guess = max(0.8 * self.max_speed, 0.1)
        tf_guess = max(total_len / v_guess, 1.0)

        # ---- 障碍物距离场 ----
        clearing: Optional[Dict] = None
        if sdf_grid is not None and sdf_info is not None:
            try:
                clearing = self._build_clearance_field(sdf_grid, sdf_info)
            except Exception as exc:
                clearing = None
                self._last_warning = f"clearance field failed: {exc}"

        # ---- 输入路径安全检查 ----
        # A* 负责避障，优化器负责平滑且不破坏安全性。若输入路径本身穿过障碍物，
        # 强行用排斥项「拉出来」通常不可行（实测 IPOPT Maximum_Iterations_Exceeded），
        # 此时应判定失败让上游重规划，而不是产出一条可能撞墙的轨迹。
        if clearing is not None:
            path_clr = min(
                self._query_clearance(clearing, float(px), float(py))
                for px, py in np.asarray([(q[0], q[1]) for q in astar_path], dtype=float)
            )
            if path_clr < 0.0:
                return TrajResult(
                    success=False,
                    solve_time_ms=(time.monotonic() - t_start) * 1000.0,
                    status=f"input path inside obstacle (clearance {path_clr:.2f}m)",
                    min_clearance=path_clr,
                )

        # 有障碍物时允许几何偏离 A*（避免「分段点钉死在障碍旁」导致不可行）
        if clearing is not None:
            follow_hard = False if self.follow_mode == "auto" else self.follow_path
        else:
            follow_hard = True if self.follow_mode == "auto" else self.follow_path

        # ---- outside-in 迭代: 无约束解 → 找违反点 → 加排斥 → 重解 ----
        # 硬约束（分段点等式）在尖角路径上不可行：例如 0.6m 段内要转 90°，
        # 需要最小转弯半径 0.3m，而车体极限是 L/tan(δmax)=0.315m。
        # 这类路径应当由优化器**圆角**，因此硬约束失败时自动降级为软引导重试。
        violations: List[Dict] = []
        n_iters = self.sdf_iters if clearing is not None else 1
        follow_attempts = [follow_hard, False] if follow_hard else [False]
        cx = cy = None
        tf = iters = 0
        status = ""
        last_exc: Optional[Exception] = None
        solved = False
        for fh in follow_attempts:
            violations = []
            tf_try = tf_guess
            try:
                for _ in range(max(1, n_iters)):
                    cx, cy, tf, iters, status = self._solve(
                        waypoints, n_seg, start_vel, tf_try,
                        violations=violations or None,
                        follow_hard=fh,
                    )
                    if clearing is None:
                        break
                    snap = self._sample(cx, cy, n_seg, tf)
                    new_v = self._find_violations(snap, clearing)
                    if not new_v:
                        violations = []
                        break
                    # 累积而不是覆盖：覆盖会让上一轮推开的点在下一轮被丢弃，
                    # 轨迹在两处障碍之间来回摆（实测 min_clr 卡在 -0.07）
                    violations = self._merge_violations(violations, new_v)
                    tf_try = tf
                solved = True
                break
            except Exception as exc:  # 求解器异常不应打断规划主循环
                last_exc = exc
                continue
        if not solved:
            return TrajResult(
                success=False,
                solve_time_ms=(time.monotonic() - t_start) * 1000.0,
                status=f"solver error: {last_exc}",
            )

        # 起始航向用于低速采样的航向兜底
        d0 = waypoints[1] - waypoints[0]
        init_heading = math.atan2(float(d0[1]), float(d0[0])) if np.linalg.norm(d0) > 1e-6 else 0.0
        samples = self._sample(cx, cy, n_seg, tf, init_heading=init_heading)
        stats = self._evaluate(samples)

        result = TrajResult(
            samples=samples,
            duration=tf,
            success=True,
            solve_time_ms=(time.monotonic() - t_start) * 1000.0,
            iterations=iters,
            status=status,
            max_speed=stats["max_speed"],
            max_kappa=stats["max_kappa"],
            max_accel=stats["max_accel"],
        )

        # ---- 避障验收：最终轨迹的最小间距必须满足 dmin - tolerance ----
        if clearing is not None:
            min_c = min(
                (self._query_clearance(clearing, x, y) for x, y, _th, _t in samples),
                default=0.0,
            )
            result.min_clearance = min_c
            if min_c < self.dmin - self.sdf_tolerance:
                result.success = False
                result.status += (
                    f" | clearance {min_c:.2f} < {self.dmin - self.sdf_tolerance:.2f}"
                )

        # 求解“成功”但严重违反物理约束时，判为失败并让调用方回退
        if (
            result.max_speed > self.max_speed * 1.15
            or result.max_kappa > self.kappa_limit * 1.3
        ):
            result.success = False
            result.status += " | constraint violation"

        return result

    # ------------------------------------------------------------------
    # 路径预处理
    # ------------------------------------------------------------------

    def _prepare_path(
        self, astar_path: Sequence[Sequence[float]]
    ) -> Optional[Tuple[np.ndarray, int]]:
        """校验路径并挑选分段点。

        Returns:
            ``(waypoints, n_seg)``，waypoints 形状 (n_seg+1, 2)。
        """
        if astar_path is None or len(astar_path) < 2:
            return None

        arr = np.asarray([(float(p[0]), float(p[1])) for p in astar_path], dtype=float)
        if not np.all(np.isfinite(arr)):
            return None

        # 去掉重复点
        keep = [0]
        for i in range(1, len(arr)):
            if np.linalg.norm(arr[i] - arr[keep[-1]]) > 1e-3:
                keep.append(i)
        arr = arr[keep]
        if len(arr) < 2:
            return None

        total = self._path_length(arr)
        cum = self._cumulative_length(arr)

        # 段数按弧长。段长不能太大：弯道段内要完成转向会导致曲率尖峰
        # （实测 seg_len=1.2 时 κ 到 44，seg_len=0.8 时正常）。
        # DP 特征点会保证转角落在段边界上，见下方合并逻辑。
        n_seg = int(round(total / self.seg_len))
        n_seg = max(self.min_segments, min(self.max_segments, n_seg))

        # 分段点：等弧长（P0 已验证的配置），并合并过短段。
        #
        # 尝试过但**失败**的替代方案（都回退了，别再试）：
        #  - DP 特征点 + 等弧长合并：特征点聚集处产生 0.44m 极短段，
        #    五次多项式在短段内病态，κ 从 2.7 爆到 10.6（无 SDF 也复现）；
        #  - 纯曲率加权等弧长：弯道段过短，κ 反而升到 4.7；
        #  - 按航向总变化加段：段数触顶 10，κ 到 553。
        # 结论：在「多项式系数 + 硬/强软引导」这套参数化下，分段策略的鲁棒性
        # 上限就在等弧长 + 短段合并。要真正解决弯道，需把分段点改成优化变量
        # （论文的 P），属于 P2 的重构，不要用启发式在这里打补丁。
        targets = np.linspace(0.0, cum[-1], n_seg + 1)
        s_final = [float(v) for v in targets]
        min_seg = max(0.50, 0.5 * self.seg_len)
        merged_final = [s_final[0]]
        for v in s_final[1:]:
            if v - merged_final[-1] >= min_seg:
                merged_final.append(v)
        if cum[-1] - merged_final[-1] > 1e-6:
            if cum[-1] - merged_final[-1] < min_seg and len(merged_final) > 2:
                merged_final[-1] = float(cum[-1])
            else:
                merged_final.append(float(cum[-1]))
        s_final = merged_final
        n_final = len(s_final) - 1
        if n_final < 2:
            return None

        wps = np.zeros((n_final + 1, 2))
        for k, s in enumerate(s_final):
            idx = int(np.searchsorted(cum, s, side="right") - 1)
            idx = max(0, min(idx, len(arr) - 2))
            seg = cum[idx + 1] - cum[idx]
            ratio = 0.0 if seg < 1e-9 else (s - cum[idx]) / seg
            wps[k] = arr[idx] + ratio * (arr[idx + 1] - arr[idx])
        return wps, n_final

    @staticmethod
    def _local_curvature(pts: np.ndarray) -> np.ndarray:
        """三点 Menger 曲率 (1/m)，与采样间距无关。"""
        n = len(pts)
        if n < 3:
            return np.zeros(n)
        k = np.zeros(n)
        for i in range(1, n - 1):
            a = np.linalg.norm(pts[i + 1] - pts[i])
            b = np.linalg.norm(pts[i + 1] - pts[i - 1])
            c = np.linalg.norm(pts[i] - pts[i - 1])
            if min(a, b, c) < 1e-9:
                continue
            cross = abs(
                (pts[i][0] - pts[i - 1][0]) * (pts[i + 1][1] - pts[i - 1][1])
                - (pts[i + 1][0] - pts[i - 1][0]) * (pts[i][1] - pts[i - 1][1])
            )
            k[i] = 2.0 * cross / (a * b * c)
        if n > 2:
            k[0], k[-1] = k[1], k[-2]
        return k

    @staticmethod
    def _dp_simplify(pts: np.ndarray, eps: float) -> List[int]:
        """Douglas-Peucker 折线简化，返回保留点的索引（含首末）。

        用递归实现；对上千点的路径深度是 O(log n) 级，不会爆栈。
        """
        n = len(pts)
        keep = {0, n - 1}
        stack = [(0, n - 1)]
        while stack:
            i0, i1 = stack.pop()
            if i1 <= i0 + 1:
                continue
            p0, p1 = pts[i0], pts[i1]
            d = p1 - p0
            norm = np.linalg.norm(d)
            best_i, best_d = -1, -1.0
            for i in range(i0 + 1, i1):
                v = pts[i] - p0
                if norm < 1e-9:
                    dist = float(np.linalg.norm(v))
                else:
                    dist = abs(float(d[0] * v[1] - d[1] * v[0])) / norm
                if dist > best_d:
                    best_i, best_d = i, dist
            if best_d > eps:
                keep.add(best_i)
                stack.append((i0, best_i))
                stack.append((best_i, i1))
        return sorted(keep)

    @staticmethod
    def _cumulative_length(pts: np.ndarray) -> np.ndarray:
        d = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        return np.concatenate([[0.0], np.cumsum(d)])

    def _path_length(self, pts: np.ndarray) -> float:
        return float(self._cumulative_length(pts)[-1])

    # ------------------------------------------------------------------
    # 障碍物距离场 (P1: 论文 Eq.29 Gc(sr) ≥ dmin)
    # ------------------------------------------------------------------

    def _build_clearance_field(
        self, grid: np.ndarray, info: Dict
    ) -> Optional[Dict]:
        """把离散占用栅格转成连续距离场（欧氏距离变换）。

        terrain_analyzer 发的 ``sdf`` 层是二值占用（-1 occupied / 1 free / 0.5 unknown），
        **不是**带梯度的距离场，直接拿来做约束会让 IPOPT 得不到下降方向。这里用
        EDT 重建到最近障碍的欧氏距离（米），并截断到 ``sdf_dmax``。

        unknown(0.5) 按 free 处理——否则地图边缘会形成一堵虚拟墙。

        Returns:
            ``{'field': (H,W) float, 'grad_y': (H,W), 'grad_x': (H,W),
               'origin': (ox, oy), 'res': r, 'shape': (H, W)}``；无 obstacle 时 None。
        """
        key = (hash(grid.tobytes()), float(info.get("resolution", 0.1)),
               float(info.get("origin", [0.0, 0.0])[0]),
               float(info.get("origin", [0.0, 0.0])[1]))
        if self._sdf_cache is not None and self._sdf_cache[0] == key:
            return self._sdf_cache[1]

        res = float(info.get("resolution", 0.1))
        origin = info.get("origin", [0.0, 0.0])
        ox, oy = float(origin[0]), float(origin[1])

        occupied = np.asarray(grid, dtype=np.float32) < 0.0
        if not occupied.any():
            self._sdf_cache = (key, None)
            return None

        # 外部距离 - 内部距离 → 有符号距离（米），障碍内为负
        from scipy.ndimage import distance_transform_edt

        dist_out = distance_transform_edt(~occupied) * res
        dist_in = distance_transform_edt(occupied) * res
        field = np.clip(dist_out - dist_in, -self.sdf_dmax, self.sdf_dmax).astype(np.float32)

        gy, gx = np.gradient(field, res)
        out = {
            "field": field,
            "grad_y": gy.astype(np.float32),
            "grad_x": gx.astype(np.float32),
            "origin": (ox, oy),
            "res": res,
            "shape": field.shape,
        }
        self._sdf_cache = (key, out)
        return out

    @staticmethod
    def _bilinear(arr: np.ndarray, fx: float, fy: float) -> float:
        """对 arr[row=y, col=x] 做双线性插值，越界取最近边界。"""
        h, w = arr.shape
        fx = min(max(fx, 0.0), w - 1.0001)
        fy = min(max(fy, 0.0), h - 1.0001)
        c0, r0 = int(fx), int(fy)
        dx, dy = fx - c0, fy - r0
        v00 = arr[r0, c0]
        v01 = arr[r0, c0 + 1]
        v10 = arr[r0 + 1, c0]
        v11 = arr[r0 + 1, c0 + 1]
        return float(
            v00 * (1 - dx) * (1 - dy) + v01 * dx * (1 - dy)
            + v10 * (1 - dx) * dy + v11 * dx * dy
        )

    def _query_clearance(self, clr: Dict, x: float, y: float) -> float:
        """查 (x,y) 处到最近障碍的距离 (m)。"""
        ox, oy = clr["origin"]
        r = clr["res"]
        return self._bilinear(clr["field"], (x - ox) / r, (y - oy) / r)

    def _query_clearance_grad(self, clr: Dict, x: float, y: float) -> Tuple[float, float]:
        """查距离场梯度 (dGc/dx, dGc/dy)，指向远离障碍的方向。"""
        ox, oy = clr["origin"]
        r = clr["res"]
        fx, fy = (x - ox) / r, (y - oy) / r
        return (
            self._bilinear(clr["grad_x"], fx, fy),
            self._bilinear(clr["grad_y"], fx, fy),
        )

    @staticmethod
    def _merge_violations(old: List[Dict], new: List[Dict]) -> List[Dict]:
        """按归一化时间合并违反点，保留每处最严格（间距最小）的排斥目标。"""
        merged: Dict[int, Dict] = {}
        for v in list(old) + list(new):
            key = int(round(float(v["s"]) * 100))  # 1% 时间分辨率
            cur = merged.get(key)
            if cur is None or float(v["clearance"]) < float(cur["clearance"]):
                merged[key] = v
        return [merged[k] for k in sorted(merged)]

    def _find_violations(
        self,
        samples: Sequence[Tuple[float, float, float, float]],
        clr: Dict,
    ) -> List[Dict]:
        """找出间距不足 dmin 的采样点，返回排斥目标（含归一化时间）。"""
        if not samples:
            return []
        tf = samples[-1][3]
        if tf <= 1e-6:
            return []
        out: List[Dict] = []
        for x, y, _th, t in samples:
            s_norm = t / tf
            # 起终点位置由边界等式刚性固定，排斥项推不动它们；
            # 对端点施加排斥会让 NLP 直接不可行（实测 Restoration_Failed）。
            if s_norm < 0.05 or s_norm > 0.95:
                continue
            c = self._query_clearance(clr, x, y)
            if c >= self.dmin:
                continue
            gx, gy = self._query_clearance_grad(clr, x, y)
            n = math.hypot(gx, gy)
            if n < 1e-6:
                continue
            push = (self.dmin - c) * 1.5 + 0.10     # 推到阈值外并留余量
            out.append(
                {
                    "s": t / tf,                     # 归一化时间，与 Tf 变化解耦
                    "x": x + push * gx / n,          # 排斥目标位置
                    "y": y + push * gy / n,
                    "clearance": c,
                }
            )
        return out

    # ------------------------------------------------------------------
    # 求解
    # ------------------------------------------------------------------

    @staticmethod
    def _beta(t, order: int):
        """五次多项式基 [1,t,t²,t³,t⁴,t⁵] 的 order 阶解析导数 (CasADi 表达式)。

        用解析式而非 ``ca.jacobian``：jacobian 要求纯符号输入，而这里 t 含变量 Tf。
        """
        if order == 0:
            return ca.vertcat(1, t, t ** 2, t ** 3, t ** 4, t ** 5)
        if order == 1:
            return ca.vertcat(0, 1, 2 * t, 3 * t ** 2, 4 * t ** 3, 5 * t ** 4)
        if order == 2:
            return ca.vertcat(0, 0, 2, 6 * t, 12 * t ** 2, 20 * t ** 3)
        if order == 3:
            return ca.vertcat(0, 0, 0, 6, 24 * t, 60 * t ** 2)
        if order == 4:
            return ca.vertcat(0, 0, 0, 0, 24, 120 * t)
        return ca.vertcat(0, 0, 0, 0, 0, 120)

    def _solve(
        self,
        wps: np.ndarray,
        n_seg: int,
        start_vel: float,
        tf_guess: float,
        violations: Optional[List[Dict]] = None,
        follow_hard: Optional[bool] = None,
    ):
        opti = ca.Opti()
        n = n_seg

        cx = [opti.variable(6) for _ in range(n)]
        cy = [opti.variable(6) for _ in range(n)]
        tf = opti.variable()

        # 时长边界：物理下限（总长/最高速）与宽松上限
        total_len = self._path_length(wps)
        tf_min = max(total_len / self.max_speed * 0.8, 0.5)
        tf_max = max(tf_min * 6.0, 30.0)
        opti.subject_to(tf >= tf_min)
        opti.subject_to(tf <= tf_max)
        tp = tf / n  # 每段时长

        # ---- 目标: ∫‖jerk‖²dt（Simpson 求积）+ ρt·Tf ----
        cost = 0.0
        lq = max(2, self.quad_points)
        if lq % 2:
            lq += 1
        for i in range(n):
            acc = 0.0
            for k in range(lq + 1):
                tk = tp * k / lq
                jx = ca.dot(cx[i], self._beta(tk, 3))
                jy = ca.dot(cy[i], self._beta(tk, 3))
                w = 1.0 if k in (0, lq) else (4.0 if k % 2 else 2.0)
                acc += w * (jx ** 2 + jy ** 2)
            cost += acc * (tp / lq) / 3.0
        cost += self.rho_t * tf

        # 路径跟随 / 避障几何。
        # follow_hard=True 时把分段点作为等式约束硬性经过（保证跟住 A* 几何）；
        # 否则用软引导，让几何有自由度避开障碍（配合下面的排斥项）。
        use_hard = self.follow_path if follow_hard is None else follow_hard
        if use_hard:
            for i in range(n):
                opti.subject_to(
                    ca.dot(cx[i], self._beta(tp, 0)) == float(wps[i + 1, 0])
                )
                opti.subject_to(
                    ca.dot(cy[i], self._beta(tp, 0)) == float(wps[i + 1, 1])
                )
        else:
            w_pg = self.path_guidance if violations is not None else self.path_guidance_no_sdf
            if w_pg > 0.0:
                for i in range(n):
                    ex = ca.dot(cx[i], self._beta(tp, 0)) - float(wps[i + 1, 0])
                    ey = ca.dot(cy[i], self._beta(tp, 0)) - float(wps[i + 1, 1])
                    cost += w_pg * (ex ** 2 + ey ** 2)

        # 避障排斥项: 对上一轮解中间距不足的采样点，在其归一化时间处把轨迹
        # 拉向「推离障碍」的目标点。CasADi 3.8 的 interpolant 多维不可用
        # （n_in=1 / 返回 nan），所以不在 NLP 里插值 SDF，改用 outside-in 迭代。
        if violations:
            w_rep = self.sdf_repulsion_weight
            for v in violations:
                s_norm = min(max(float(v["s"]), 0.0), 0.9999)
                idx = min(int(s_norm * n), n - 1)
                tl = (s_norm * n - idx) * tp
                ex = ca.dot(cx[idx], self._beta(tl, 0)) - float(v["x"])
                ey = ca.dot(cy[idx], self._beta(tl, 0)) - float(v["y"])
                cost += w_rep * (ex ** 2 + ey ** 2)
        opti.minimize(cost)

        # ---- 边界条件 ----
        dx0 = float(wps[1, 0] - wps[0, 0])
        dy0 = float(wps[1, 1] - wps[0, 1])
        n0 = math.hypot(dx0, dy0)
        heading = math.atan2(dy0, dx0) if n0 > 1e-6 else 0.0
        v0 = max(0.0, min(start_vel, self.max_speed))

        for c, p0, p_end, v0_comp in (
            (cx, float(wps[0, 0]), float(wps[-1, 0]), v0 * math.cos(heading)),
            (cy, float(wps[0, 1]), float(wps[-1, 1]), v0 * math.sin(heading)),
        ):
            opti.subject_to(ca.dot(c[0], self._beta(0, 0)) == p0)        # 起点位置
            opti.subject_to(ca.dot(c[0], self._beta(0, 1)) == v0_comp)   # 起点速度
            opti.subject_to(ca.dot(c[0], self._beta(0, 2)) == 0.0)       # 起点加速度
            opti.subject_to(ca.dot(c[-1], self._beta(tp, 0)) == p_end)   # 终点位置
            opti.subject_to(ca.dot(c[-1], self._beta(tp, 1)) == 0.0)     # 终点速度

        # ---- 段间 C⁴ 连续 ----
        for i in range(n - 1):
            for order in range(5):
                for c in (cx, cy):
                    opti.subject_to(
                        ca.dot(c[i], self._beta(tp, order))
                        == ca.dot(c[i + 1], self._beta(0, order))
                    )

        # ---- 运动学可行性（每段 K 点采样）----
        # 采样密度必须随段长自适应：段长 1.2m 的 90° 弯用 6 点会漏掉段内曲率峰值
        # （实测 κ 冲到 14，限值 2.7）。按 ~0.15m 一个采样点，下限 6。
        seg_len_m = total_len / n
        k_samples = max(
            self.constraint_samples,
            int(math.ceil(seg_len_m / 0.10)),
        )
        k_samples = min(k_samples, 24)
        kappa2 = self.kappa_limit ** 2
        # 曲率约束的数值保护: κ = cross / v2^1.5，v2→0 时微分病态。
        # 用 (v2 + eps) 代替硬性下界 v2 >= eps —— 后者在起点速度=0 时直接不可行
        # （静止起步是常规工况）。v2→0 时 cross 同样→0，约束自动满足。
        eps_v2 = 1e-3
        for i in range(n):
            for k in range(k_samples + 1):
                tk = tp * k / k_samples
                vx = ca.dot(cx[i], self._beta(tk, 1))
                vy = ca.dot(cy[i], self._beta(tk, 1))
                ax = ca.dot(cx[i], self._beta(tk, 2))
                ay = ca.dot(cy[i], self._beta(tk, 2))
                v2 = vx ** 2 + vy ** 2
                opti.subject_to(v2 <= self.max_speed ** 2)
                opti.subject_to(ax ** 2 + ay ** 2 <= self.max_accel ** 2)
                cross = vx * ay - vy * ax
                opti.subject_to(cross ** 2 <= kappa2 * (v2 + eps_v2) ** 3)

        # ---- 初值：匀速直线近似 + A* 分段点 ----
        for i in range(n):
            seg_dx = float(wps[i + 1, 0] - wps[i, 0])
            seg_dy = float(wps[i + 1, 1] - wps[i, 1])
            tp0 = tf_guess / n
            opti.set_initial(cx[i], np.array([wps[i, 0], seg_dx / tp0, 0, 0, 0, 0]))
            opti.set_initial(cy[i], np.array([wps[i, 1], seg_dy / tp0, 0, 0, 0, 0]))
        opti.set_initial(tf, tf_guess)

        opti.solver(
            "ipopt",
            {
                "ipopt.print_level": 0,
                "print_time": False,
                "ipopt.max_iter": self.max_iter,
                "ipopt.tol": self.tol,
                "ipopt.warm_start_init_point": "yes",
                "ipopt.mu_strategy": "adaptive",
                "ipopt.hessian_approximation": "limited-memory",
            },
        )

        sol = opti.solve()
        cx_v = [np.array(sol.value(c)).ravel() for c in cx]
        cy_v = [np.array(sol.value(c)).ravel() for c in cy]
        tf_v = float(sol.value(tf))
        try:
            iters = int(sol.stats().get("iter_count", 0))
        except Exception:
            iters = 0
        try:
            status = str(sol.stats().get("return_status", "ok"))
        except Exception:
            status = "ok"
        return cx_v, cy_v, tf_v, iters, status

    # ------------------------------------------------------------------
    # 采样与评估
    # ------------------------------------------------------------------

    @staticmethod
    def _beta_np(t: float, order: int) -> np.ndarray:
        """_beta 的 numpy 版本，用于求解后采样。"""
        if order == 0:
            return np.array([1, t, t ** 2, t ** 3, t ** 4, t ** 5], dtype=float)
        if order == 1:
            return np.array([0, 1, 2 * t, 3 * t ** 2, 4 * t ** 3, 5 * t ** 4], dtype=float)
        if order == 2:
            return np.array([0, 0, 2, 6 * t, 12 * t ** 2, 20 * t ** 3], dtype=float)
        return np.array([0, 0, 0, 6, 24 * t, 60 * t ** 2], dtype=float)

    def _sample(
        self,
        cx: List[np.ndarray],
        cy: List[np.ndarray],
        n_seg: int,
        tf: float,
        init_heading: float = 0.0,
    ) -> List[Tuple[float, float, float, float]]:
        """等时间采样成 [(x, y, theta, t)]。

        航向由速度方向 atan2(vy, vx) 给出，但**低速时速度方向无意义**
        （起点静止、终点停车、段边界都可能 v≈0，atan2(0,0) 会给出 0 并造成
        上百度的假跳变）。因此低速时沿用上一采样点的航向，首个点用路径起始航向。
        """
        if tf <= 1e-6:
            return []
        n_pts = max(2, int(math.ceil(tf / self.sample_dt)) + 1)
        tp = tf / n_seg
        v_eps = 0.02  # m/s，低于此速度认为方向不可靠
        prev_th = init_heading
        out: List[Tuple[float, float, float, float]] = []
        for j in range(n_pts):
            t = tf * j / (n_pts - 1)
            i = min(int(t / tp), n_seg - 1)
            tl = t - i * tp
            x = float(self._beta_np(tl, 0) @ cx[i])
            y = float(self._beta_np(tl, 0) @ cy[i])
            vx = float(self._beta_np(tl, 1) @ cx[i])
            vy = float(self._beta_np(tl, 1) @ cy[i])
            if math.hypot(vx, vy) >= v_eps:
                prev_th = math.atan2(vy, vx)
            out.append((x, y, prev_th, t))
        return out

    def _evaluate(self, samples: Sequence[Tuple[float, float, float, float]]) -> Dict[str, float]:
        """统计采样点上的速度/曲率/加速度峰值。

        曲率用 Menger 曲率 ``κ = 2·|cross| / (a·b·c)``（三点三角形外接圆倒数），
        对等时间采样点比航向差分更稳，也不受航向 wrap 影响。
        """
        if len(samples) < 2:
            return {"max_speed": 0.0, "max_kappa": 0.0, "max_accel": 0.0}

        speeds: List[float] = []
        for i in range(1, len(samples)):
            x0, y0, _, t0 = samples[i - 1]
            x1, y1, _, t1 = samples[i]
            dt = t1 - t0
            speeds.append(math.hypot(x1 - x0, y1 - y0) / dt if dt > 1e-9 else 0.0)

        max_v = max(speeds) if speeds else 0.0

        max_a = 0.0
        for i in range(1, len(speeds)):
            dt = samples[i + 1][3] - samples[i][3]
            if dt > 1e-9:
                max_a = max(max_a, abs(speeds[i] - speeds[i - 1]) / dt)

        max_kappa = 0.0
        for i in range(2, len(samples)):
            p0 = samples[i - 2][:2]
            p1 = samples[i - 1][:2]
            p2 = samples[i][:2]
            a = math.dist(p1, p2)
            b = math.dist(p0, p2)
            c = math.dist(p0, p1)
            # 边长过小时 Menger 曲率 = 2·cross/(abc) 会被数值噪声放大：
            # 低速段相邻采样点几乎重合，实测会算出 κ=887 这种假值，
            # 导致本来合格的轨迹被验收判为「曲率超限」。1cm 以下不予评估。
            if min(a, b, c) < 0.01:
                continue
            cross = abs((p1[0] - p0[0]) * (p2[1] - p0[1]) - (p2[0] - p0[0]) * (p1[1] - p0[1]))
            max_kappa = max(max_kappa, 2.0 * cross / (a * b * c))

        return {"max_speed": max_v, "max_kappa": max_kappa, "max_accel": max_a}

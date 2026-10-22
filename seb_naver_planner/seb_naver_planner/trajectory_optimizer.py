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
    DEFAULT_SEG_LEN = 1.2

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

        self.rho_t: float = float(p.get("rho_t", self.DEFAULT_RHO_T))
        # 曲率使用裕度：优化到 0.85·κmax，给 MPC 跟踪留余量
        self.kappa_limit: float = (
            math.tan(self.max_steer) / self.wheel_base
        ) * float(p.get("kappa_margin", 0.85))

        self.quad_points: int = int(p.get("quad_points", 4))    # Simpson 求积点数(偶数)
        self.constraint_samples: int = int(p.get("constraint_samples", 6))  # 每段约束采样
        self.sample_dt: float = float(p.get("sample_dt", self.DEFAULT_SAMPLE_DT))

        self.max_iter: int = int(p.get("max_iter", 200))
        self.tol: float = float(p.get("tol", 1e-2))
        # 是否把分段点作为硬约束经过（P0 默认 True，保证跟住 A* 的几何）
        self.follow_path: bool = bool(p.get("follow_path", True))
        # 不启用 follow_path 时的软引导权重
        self.path_guidance: float = float(p.get("path_guidance", 0.5))

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

        try:
            cx, cy, tf, iters, status = self._solve(
                waypoints, n_seg, start_vel, tf_guess
            )
        except Exception as exc:  # 求解器异常不应打断规划主循环
            return TrajResult(
                success=False,
                solve_time_ms=(time.monotonic() - t_start) * 1000.0,
                status=f"solver error: {exc}",
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
        n_seg = int(round(total / self.seg_len))
        n_seg = max(self.min_segments, min(self.max_segments, n_seg))

        # 按弧长等间隔重采样出 n_seg+1 个分段点
        cum = self._cumulative_length(arr)
        targets = np.linspace(0.0, cum[-1], n_seg + 1)
        wps = np.zeros((n_seg + 1, 2))
        for k, s in enumerate(targets):
            idx = int(np.searchsorted(cum, s, side="right") - 1)
            idx = max(0, min(idx, len(arr) - 2))
            seg = cum[idx + 1] - cum[idx]
            ratio = 0.0 if seg < 1e-9 else (s - cum[idx]) / seg
            wps[k] = arr[idx] + ratio * (arr[idx + 1] - arr[idx])
        return wps, n_seg

    @staticmethod
    def _cumulative_length(pts: np.ndarray) -> np.ndarray:
        d = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        return np.concatenate([[0.0], np.cumsum(d)])

    def _path_length(self, pts: np.ndarray) -> float:
        return float(self._cumulative_length(pts)[-1])

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

        # 路径跟随: P0 阶段没有 SDF 硬约束，必须把分段点作为等式约束硬性经过。
        # 否则优化会把轨迹拉成「起点到终点的最短平滑曲线」—— 弯道切角，甚至穿墙。
        # （论文里 P 是自由变量，靠 SDF/Risk 约束保证安全；P1 接上 SDF 后可改软引导）
        if self.follow_path:
            for i in range(n):
                opti.subject_to(
                    ca.dot(cx[i], self._beta(tp, 0)) == float(wps[i + 1, 0])
                )
                opti.subject_to(
                    ca.dot(cy[i], self._beta(tp, 0)) == float(wps[i + 1, 1])
                )
        elif self.path_guidance > 0.0:
            for i in range(n):
                ex = ca.dot(cx[i], self._beta(tp, 0)) - float(wps[i + 1, 0])
                ey = ca.dot(cy[i], self._beta(tp, 0)) - float(wps[i + 1, 1])
                cost += self.path_guidance * (ex ** 2 + ey ** 2)
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
        k_samples = max(2, self.constraint_samples)
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
            if min(a, b, c) < 1e-9:
                continue
            cross = abs((p1[0] - p0[0]) * (p2[1] - p0[1]) - (p2[0] - p0[0]) * (p1[1] - p0[1]))
            max_kappa = max(max_kappa, 2.0 * cross / (a * b * c))

        return {"max_speed": max_v, "max_kappa": max_kappa, "max_accel": max_a}

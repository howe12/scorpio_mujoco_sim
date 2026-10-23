#!/usr/bin/env python3
"""
SEB-Naver Planner Node — ReplanFSM + PlanManager

Orchestrates the full planning pipeline for SEB-Naver navigation:
  INIT → WAIT_TARGET → SEQUENTIAL_START → EXEC_TRAJ ⇄ REPLAN_TRAJ

Subscribes:
  /Odometry                   (nav_msgs/Odometry)       — FAST-LIO2 state
  /terrain_analyzer/sdf_map   (se2_grid_msgs/SE2Grid)   — SDF grid map
  /goal_pose                  (geometry_msgs/PoseStamped) — navigation goal

Publishes:
  /planner/trajectory         (nav_msgs/Path) — optimized path for MPC
  /planner/rough_path         (nav_msgs/Path) — kinodynamic A* raw path
  /planner/status             (std_msgs/String) — current FSM state name
"""

from __future__ import annotations

import math
import threading
import time
from concurrent.futures import Future, ThreadPoolExecutor
from enum import Enum, auto
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String, Header, Float32MultiArray, MultiArrayDimension
from se2_grid_msgs.msg import SE2Grid

from seb_naver_planner.kino_astar import KinoAStarPlanner, SearchStatus

try:
    from seb_naver_planner.trajectory_optimizer import TrajectoryOptimizer
except ImportError:  # CasADi 缺失时降级为「只用 A*」
    TrajectoryOptimizer = None  # type: ignore[assignment]


# ---------------------------------------------------------------------------
# FSM States
# ---------------------------------------------------------------------------
class FSMState(Enum):
    """ReplanFSM states."""
    INIT = auto()
    WAIT_TARGET = auto()
    SEQUENTIAL_START = auto()
    EXEC_TRAJ = auto()
    REPLAN_TRAJ = auto()


# ---------------------------------------------------------------------------
# Helper: quaternion ↔ yaw
# ---------------------------------------------------------------------------
def quat_to_yaw(q: Quaternion) -> float:
    """Extract yaw angle from a quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quat(yaw: float) -> Quaternion:
    """Create a quaternion from a yaw angle."""
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    return q


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


# ---------------------------------------------------------------------------
# SDF Grid Parser
# ---------------------------------------------------------------------------
class SDFGridParser:
    """Parse SE2Grid messages into a 2D numpy SDF array.

    Handles circular-buffer indexing via start_index_row / start_index_col.
    """

    def __init__(self) -> None:
        self.sdf_map: Optional[np.ndarray] = None
        self.resolution: float = 0.0
        self.length_x: float = 0.0
        self.length_y: float = 0.0
        self.map_center_x: float = 0.0
        self.map_center_y: float = 0.0
        self.rows: int = 0
        self.cols: int = 0
        self.updated: bool = False

    def update_from_msg(self, msg: SE2Grid) -> None:
        """Extract the 'sdf' layer from an SE2Grid message."""
        # Find sdf layer index
        sdf_idx: Optional[int] = None
        for i, name in enumerate(msg.layers):
            if name == "sdf":
                sdf_idx = i
                break

        if sdf_idx is None:
            return  # no sdf layer present

        info = msg.info
        self.resolution = info.pos_resolution
        self.length_x = info.length_x
        self.length_y = info.length_y
        self.map_center_x = info.pose.position.x
        self.map_center_y = info.pose.position.y

        # Compute grid dimensions
        self.rows = int(round(self.length_y / self.resolution))
        self.cols = int(round(self.length_x / self.resolution))

        if self.rows <= 0 or self.cols <= 0:
            return

        # Extract raw data from Float32MultiArray
        raw_data = np.array(msg.data[sdf_idx].data, dtype=np.float32)
        expected_size = self.rows * self.cols
        if raw_data.size < expected_size:
            return  # incomplete data

        # Reshape to 2D grid (row-major)
        grid_2d = raw_data[:expected_size].reshape((self.rows, self.cols))

        # Handle circular buffer offsets
        start_row = int(msg.start_index_row) % self.rows
        start_col = int(msg.start_index_col) % self.cols

        # Roll the grid so that (0,0) corresponds to the logical origin
        self.sdf_map = np.roll(np.roll(grid_2d, -start_row, axis=0), -start_col, axis=1)
        self.updated = True

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid indices."""
        col = int(round((x - (self.map_center_x - self.length_x / 2.0)) / self.resolution))
        row = int(round((y - (self.map_center_y - self.length_y / 2.0)) / self.resolution))
        return row, col

    def grid_to_world(self, row: int, col: int) -> Tuple[float, float]:
        """Convert grid indices to world coordinates."""
        x = self.map_center_x - self.length_x / 2.0 + col * self.resolution
        y = self.map_center_y - self.length_y / 2.0 + row * self.resolution
        return x, y

    def get_sdf_value(self, x: float, y: float) -> float:
        """Get SDF value at world position. Returns 0.0 if out of bounds."""
        if self.sdf_map is None:
            return 0.0
        row, col = self.world_to_grid(x, y)
        if 0 <= row < self.rows and 0 <= col < self.cols:
            return float(self.sdf_map[row, col])
        return 0.0


# ---------------------------------------------------------------------------
# Elevation Grid Parser (P3: 地形姿态约束)
# ---------------------------------------------------------------------------
class ElevationGridParser:
    """解析 /fused_map 的 elevation / lower_bound / upper_bound 层。

    用于计算地形坡度与车体姿态（论文 Eq.27 的 φx/φy 约束）。
    地形是逐点融合的，未观测区域的 elevation 保持默认值，必须靠
    ``upper_bound - lower_bound`` 判别有效性，否则会把「没看过的地方」
    当成陡坡。
    """

    def __init__(self) -> None:
        self.elevation: Optional[np.ndarray] = None
        self.valid: Optional[np.ndarray] = None
        self.resolution: float = 0.0
        self.length_x: float = 0.0
        self.length_y: float = 0.0
        self.map_center_x: float = 0.0
        self.map_center_y: float = 0.0
        self.rows: int = 0
        self.cols: int = 0
        self.updated: bool = False

    @staticmethod
    def _extract(msg: SE2Grid, name: str, rows: int, cols: int) -> Optional[np.ndarray]:
        """取某一层并按圆形缓冲偏移 roll 回逻辑原点。"""
        idx: Optional[int] = None
        for i, nm in enumerate(msg.layers):
            if nm == name:
                idx = i
                break
        if idx is None:
            return None
        raw = np.array(msg.data[idx].data, dtype=np.float32)
        n = rows * cols
        if raw.size < n:
            return None
        grid = raw[:n].reshape((rows, cols))
        start_row = int(msg.start_index_row) % rows
        start_col = int(msg.start_index_col) % cols
        return np.roll(np.roll(grid, -start_row, axis=0), -start_col, axis=1)

    def update_from_msg(self, msg: SE2Grid) -> None:
        info = msg.info
        if info.pos_resolution <= 0.0:
            return
        self.resolution = float(info.pos_resolution)
        self.length_x = float(info.length_x)
        self.length_y = float(info.length_y)
        self.map_center_x = float(info.pose.position.x)
        self.map_center_y = float(info.pose.position.y)
        self.rows = int(round(self.length_y / self.resolution))
        self.cols = int(round(self.length_x / self.resolution))
        if self.rows <= 0 or self.cols <= 0:
            return

        elev = self._extract(msg, "elevation", self.rows, self.cols)
        if elev is None:
            return
        self.elevation = elev

        lo = self._extract(msg, "lower_bound", self.rows, self.cols)
        hi = self._extract(msg, "upper_bound", self.rows, self.cols)
        finite = np.isfinite(elev)
        if lo is not None and hi is not None:
            # 置信区间窄的地方才是真正观测过的地形
            self.valid = finite & np.isfinite(hi) & np.isfinite(lo) & ((hi - lo) < 0.5)
        else:
            self.valid = finite
        self.updated = True

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        col = int(round((x - (self.map_center_x - self.length_x / 2.0)) / self.resolution))
        row = int(round((y - (self.map_center_y - self.length_y / 2.0)) / self.resolution))
        return row, col

    #: 差分半窗（格数）。1 格 (0.1m) 基线会把厘米级高程噪声放大成度数：
    #: 5cm 高差 / 0.2m ≈ 14°，实测平地 plaza 也会被判姿态超限。
    SLOPE_HALF_WINDOW = 2

    def slope_at(self, x: float, y: float) -> Optional[Tuple[float, float]]:
        """返回 (dz/dx, dz/dy)；该处地形未观测时返回 None。"""
        if self.elevation is None or self.valid is None:
            return None
        row, col = self.world_to_grid(x, y)
        w = self.SLOPE_HALF_WINDOW
        if not (w <= row < self.rows - w and w <= col < self.cols - w):
            return None
        if not self.valid[row, col]:
            return None
        span = 2.0 * w * self.resolution
        dzdx = (self.elevation[row, col + w] - self.elevation[row, col - w]) / span
        dzdy = (self.elevation[row + w, col] - self.elevation[row - w, col]) / span
        return float(dzdx), float(dzdy)

    def attitude_at(self, x: float, y: float, yaw: float) -> Optional[Tuple[float, float]]:
        """车体在地形上的俯仰/横滚角 (rad)：φx 俯仰、φy 横滚。

        俯仰 = 沿车头方向的地形坡角；横滚 = 沿车侧方向的地形坡角。
        """
        g = self.slope_at(x, y)
        if g is None:
            return None
        dzdx, dzdy = g
        c, s = math.cos(yaw), math.sin(yaw)
        # 车头方向 (c, s)、左侧方向 (-s, c)
        dz_fwd = dzdx * c + dzdy * s
        dz_lat = -dzdx * s + dzdy * c
        return math.atan(dz_fwd), math.atan(dz_lat)

    # ---- 可通行性风险评估 (论文 Algorithm 1, 按需计算) ----
    #: 机器人足迹椭圆半轴 (m)，与 terrain_analyzer 的 ellipsoid_x/y 一致
    FOOTPRINT_A = 0.20
    FOOTPRINT_B = 0.15

    def risk_at(
        self,
        x: float,
        y: float,
        yaw: float,
        max_curvature: float = 1000.0,
        max_pitch: float = 0.786,
        max_roll: float = 1.571,
        w_curvature: float = 0.6,
        w_phi: float = 0.2,
    ) -> Optional[float]:
        """论文 Algorithm 1：对给定位姿评估可通行性风险 ∈ [0, 1]。

        做法：取足迹椭圆内的地形点 → 3×3 协方差 → 最小特征向量作为曲面法线
        → 由法线得曲率与俯仰/横滚 → 加权求和。超过硬限则 risk=1。

        与 terrain_analyzer 的 C++ 全图实现数学一致，但只算单个点（约 20μs），
        所以可以对轨迹的几十个采样点实时求值。

        Returns:
            risk ∈ [0,1]；地形未观测时返回 None。
        """
        if self.elevation is None or self.valid is None:
            return None
        r0, c0 = self.world_to_grid(x, y)
        sa = int(math.ceil(max(self.FOOTPRINT_A, self.FOOTPRINT_B) / self.resolution)) + 1
        if not (sa <= r0 < self.rows - sa and sa <= c0 < self.cols - sa):
            return None

        # 向量化取邻域窗口（Python 逐点循环实测 478μs/次，
        # 对 A* 的数千个抽样点不可接受；切片后约 1/8 开销）
        win = self.elevation[r0 - sa:r0 + sa + 1, c0 - sa:c0 + sa + 1]
        ok = self.valid[r0 - sa:r0 + sa + 1, c0 - sa:c0 + sa + 1]
        n_win = win.shape[0]
        idx = (np.arange(n_win) - sa) * self.resolution
        dxg, dyg = np.meshgrid(idx, idx)   # 都是 (n_win, n_win)

        ca, sa_ = math.cos(yaw), math.sin(yaw)
        # 世界系偏移 → 车身系
        rx = ca * dxg + sa_ * dyg
        ry = -sa_ * dxg + ca * dyg
        inside = ((rx * rx) / (self.FOOTPRINT_A ** 2)
                  + (ry * ry) / (self.FOOTPRINT_B ** 2)) <= 1.0
        sel = inside & ok & np.isfinite(win)
        if sel.sum() <= 7:
            return None  # 有效点太少，与 C++ 版一致

        z = win[sel].astype(np.float64)
        ex = dxg[sel].astype(np.float64)
        ey = dyg[sel].astype(np.float64)
        dz = z - z.mean()
        cov = np.empty((3, 3), dtype=np.float64)
        cov[0, 0] = (ex * ex).mean()
        cov[1, 1] = (ey * ey).mean()
        cov[2, 2] = (dz * dz).mean()
        cov[0, 1] = cov[1, 0] = (ex * ey).mean()
        cov[0, 2] = cov[2, 0] = (ex * dz).mean()
        cov[1, 2] = cov[2, 1] = (ey * dz).mean()

        evals, evecs = np.linalg.eigh(cov)      # 升序
        normal = evecs[:, 0]
        if normal[2] < 0:
            normal = -normal
        curvature = 3.0 * evals[0] / (evals.sum() + 1e-10)

        # 由法线重建车体系，求俯仰/横滚
        zb = normal
        xyaw = np.array([ca, sa_, 0.0])
        yb = np.cross(zb, xyaw)
        nyn = np.linalg.norm(yb)
        if nyn < 1e-9:
            return 1.0
        yb = yb / nyn
        xb = np.cross(yb, zb)
        xb = xb / max(np.linalg.norm(xb), 1e-9)
        up = np.array([0.0, 0.0, 1.0])
        phi_x = abs(math.asin(min(1.0, abs(float(up @ xb)))))
        phi_y = abs(math.asin(min(1.0, abs(float(up @ yb)))))

        risk = (
            w_curvature * (curvature / max(max_curvature, 1e-9))
            + w_phi * (phi_x / max(max_pitch, 1e-9))
            + w_phi * (phi_y / max(max_roll, 1e-9))
        )
        if curvature > max_curvature or phi_x > max_pitch or phi_y > max_roll:
            return 1.0
        return float(min(max(risk, 0.0), 1.0))

    def untraversable_mask(self, max_slope_rad: float) -> np.ndarray:
        """坡度超过阈值的格子（用于喂给 A* 当障碍）。"""
        n = self.rows * self.cols
        mask = np.zeros((self.rows, self.cols), dtype=bool)
        if self.elevation is None or self.valid is None or max_slope_rad <= 0.0:
            return mask
        z = np.where(self.valid, self.elevation, np.nan)
        # 3×3 均值平滑：单格噪声在 0.1m 基线上会放大成几十度的假坡度
        try:
            from scipy.ndimage import uniform_filter
            filled = np.where(np.isfinite(z), z, np.nanmean(z[np.isfinite(z)]) if np.isfinite(z).any() else 0.0)
            z = np.where(self.valid, uniform_filter(filled, size=3), np.nan)
        except ImportError:
            pass
        dzdy, dzdx = np.gradient(z, self.resolution)
        slope = np.arctan(np.hypot(dzdx, dzdy))
        mask = np.isfinite(slope) & (slope > max_slope_rad)
        return mask


# ---------------------------------------------------------------------------
# PlanManager
# ---------------------------------------------------------------------------
class PlanManager:
    """规划流水线：Kinodynamic A* → 轨迹优化（异步）→ 输出。

    轨迹优化的单次求解耗时 0.4~2.5s（IPOPT，N≤10 段），而 FSM 定时器是 50Hz。
    同步调用会阻塞 FSM 整整 0.4~2.5s（期间不判到达、不响应重规划），因此优化放在
    单线程 executor 里**异步**执行：

      - ``plan()`` 只做 A*（毫秒级）并立即返回；
      - 若当前有可用的优化结果（起点接近、未过期），返回优化轨迹，否则返回 A* 粗路径；
      - 优化任务在后台完成后才替换结果，失败则丢弃（继续用 A* 粗路径）。

    这样最坏情况只是「晚一次重规划才用上优化轨迹」，不会卡住 FSM。
    """

    def __init__(
        self,
        node: Node,
        sdf_parser: SDFGridParser,
        elev_parser: Optional["ElevationGridParser"] = None,
    ) -> None:
        self._node = node
        self._sdf = sdf_parser
        self._elev = elev_parser

        # Parameters (declared on the node)
        self.wheel_base: float = node.get_parameter("wheel_base").value
        self.max_speed: float = node.get_parameter("max_speed").value
        self.max_steer: float = node.get_parameter("max_steer").value
        self.step_arc: float = node.get_parameter("step_arc").value
        self.steer_res: float = node.get_parameter("steer_res").value

        # Kinodynamic A* planner (沿用 steer_res/step_arc → 更平滑的转弯路径)
        self._kino_astar = KinoAStarPlanner({
            'wheel_base': self.wheel_base,
            'max_speed': self.max_speed,
            'max_steer': self.max_steer,
            'step_arc': self.step_arc,
            'steer_res': self.steer_res,
        })

        # ---- 轨迹优化（P2：异步接入）----
        self.enable_optimization: bool = node.get_parameter("enable_optimization").value
        # 有效性判据：机器人当前位置到优化轨迹的最近距离。
        # 不用「离优化起点多远」——机器人每秒移动 0.5m，1s 后就超过 0.8m，
        # 优化结果会不断失效（实测几乎全部 fallback）。
        self.optimize_offtrack_tolerance: float = node.get_parameter(
            "optimize_offtrack_tolerance").value
        self.optimize_valid_s: float = node.get_parameter("optimize_valid_s").value

        self._optimizer = None
        self._executor: Optional[ThreadPoolExecutor] = None
        self._opt_future: Optional[Future] = None
        self._opt_lock = threading.Lock()
        self._opt_samples: Optional[List[Tuple[float, float, float, float]]] = None
        self._opt_start: Optional[Tuple[float, float]] = None
        self._opt_time: float = 0.0
        self._opt_ok: int = 0
        self._opt_fail: int = 0
        self._opt_skip_short: int = 0
        self._opt_attitude_reject: int = 0
        self._slope_fallback: int = 0
        self._opt_risk_reject: int = 0
        # 缓存：坡度 mask 与膨胀栅格。两者都是对整幅 200×200 图的重操作
        # （np.gradient / binary_dilation / binary_opening 约 200ms），
        # 而地形和 SDF 在相邻两次规划间通常不变。按内容哈希命中缓存。
        self._slope_cache: Optional[Tuple] = None
        self._inflate_cache: Optional[Tuple] = None
        # 过短的路径不优化：N 会被压到 3 段、段长 <0.7m，五次多项式病态，
        # 优化结果几乎必然 κ 超限（实测短路径场景 17/19 被拒）。
        self.min_optimize_length: float = node.get_parameter(
            "optimize_min_length").value
        # 优化轨迹按时间等间隔采样（0.1s），点间距只有几厘米；
        # MPC 用「最近点 + N 个点」取参考，视界会被压到 20×0.04≈0.8m，
        # 导致跟踪极慢、转弯提前量不足。发布前按弧长重采样到 0.3m。
        self.traj_publish_step: float = node.get_parameter("traj_publish_step").value
        # 车体膨胀半径：A* 的碰撞检测只看格子占用，不考虑车体尺寸，
        # 实测规划出的路径线段最小间距只有 0.157m，而车体半宽 0.211m —— 车身会蹭墙。
        # 把障碍物按此半径膨胀后再给 A*，路径才会真正留出车宽。
        self.inflation_radius: float = node.get_parameter("inflation_radius").value
        # ---- 地形姿态 (P3, 论文 Eq.27 的 φx/φy) ----
        # 坡度超过 max_slope 的区域直接当作不可通行（A* 避开陡坡）
        self.max_slope_deg: float = node.get_parameter("max_slope_deg").value
        # 轨迹验收：俯仰/横滚超过阈值判失败
        self.max_pitch_deg: float = node.get_parameter("max_pitch_deg").value
        self.max_roll_deg: float = node.get_parameter("max_roll_deg").value
        # 验收容差：高程图有厘米级噪声，轻微超限不拒绝整条轨迹
        self.attitude_tolerance: float = node.get_parameter(
            "attitude_tolerance").value
        # ---- 可通行性风险 (P4, 论文 Eq.22 的 ρr 项 + Eq.28 的 rmax 约束) ----
        self.risk_limit: float = node.get_parameter("risk_limit").value
        self.risk_soft_limit: float = node.get_parameter("risk_soft_limit").value
        # A* 代价加权的风险惩罚系数（软偏好低风险，不禁止通行）
        self.risk_astar_weight: float = node.get_parameter("risk_astar_weight").value
        # 剩余轨迹短于此值则视为不可用，等待下一次优化
        self.optimize_min_remaining: float = node.get_parameter(
            "optimize_min_remaining").value

        #: 最近一次 A* 粗路径 [(x, y, yaw)]，供发布 /planner/rough_path
        self.last_rough: List[Tuple[float, float, float]] = []
        #: 最近一次实际交给 MPC 的轨迹（优化结果或粗路径）
        self.last_traj: List[Tuple[float, float, float]] = []

        if self.enable_optimization and TrajectoryOptimizer is not None:
            try:
                self._optimizer = TrajectoryOptimizer({
                    "wheel_base": self.wheel_base,
                    "max_speed": self.max_speed,
                    "max_accel": node.get_parameter("trajectory_max_accel").value,
                    "max_steer": self.max_steer,
                    "seg_len": node.get_parameter("trajectory_seg_len").value,
                    "dmin": node.get_parameter("trajectory_dmin").value,
                    "sdf_iters": node.get_parameter("trajectory_sdf_iters").value,
                })
                self._executor = ThreadPoolExecutor(
                    max_workers=1, thread_name_prefix="traj_opt")
                node.get_logger().info(
                    "TrajectoryOptimizer enabled (async, dmin="
                    f"{node.get_parameter('trajectory_dmin').value}m)")
            except Exception as exc:
                self._optimizer = None
                node.get_logger().warn(f"TrajectoryOptimizer disabled: {exc}")
        elif self.enable_optimization:
            node.get_logger().warn(
                "TrajectoryOptimizer unavailable (CasADi missing) → A* only")

    def plan(
        self,
        start_x: float,
        start_y: float,
        start_yaw: float,
        start_vel: float,
        goal_x: float,
        goal_y: float,
        goal_yaw: float,
    ) -> Optional[list]:
        """Run the planning pipeline.

        Returns a list of waypoints [(x, y, yaw), ...] or None on failure.
        Currently uses kinodynamic A* directly (PHR-ALM not yet implemented).
        """
        if self._sdf.sdf_map is None:
            self._node.get_logger().warn("PlanManager: SDF map not available, skipping planning")
            return None

        # Step 1–3: Kinodynamic A* search
        sdf_info = {
            'resolution': self._sdf.resolution,
            # kino_astar 读取 'origin' key (list [x,y]), 不是 origin_x/origin_y
            'origin': [self._sdf.map_center_x - self._sdf.length_x / 2.0,
                       self._sdf.map_center_y - self._sdf.length_y / 2.0],
            'width': self._sdf.sdf_map.shape[1] if self._sdf.sdf_map is not None else 0,
            'height': self._sdf.sdf_map.shape[0] if self._sdf.sdf_map is not None else 0,
        }
        grid_obstacles = self._inflate_sdf(self._sdf.sdf_map)
        # 坡度超限区域视为障碍（与障碍物同等处理，A* 自然绕开陡坡）
        grid_for_astar = self._apply_slope_mask(grid_obstacles, sdf_info)
        status, path = self._kino_astar.search(
            start_state=(start_x, start_y, start_yaw),
            goal_state=(goal_x, goal_y, goal_yaw),
            sdf_grid_data=grid_for_astar,
            sdf_info=sdf_info,
        )
        if status != SearchStatus.SUCCESS and grid_for_astar is not grid_obstacles:
            # 坡度过滤把唯一通路挡住了 → 退化为「只避开障碍物」再找一次。
            # 宁可走一段陡坡（姿态验收会在轨迹层面拦下极端情况），
            # 也不要因为地图边缘的坡度噪声整段规划失败（实测 15~68 次失败）。
            self._slope_fallback += 1
            status, path = self._kino_astar.search(
                start_state=(start_x, start_y, start_yaw),
                goal_state=(goal_x, goal_y, goal_yaw),
                sdf_grid_data=grid_obstacles,
                sdf_info=sdf_info,
            )

        if status != SearchStatus.SUCCESS or not path:
            self._node.get_logger().error(f"PlanManager: A* search failed (status={status.name})")
            return None

        # Convert path format: list of (x, y, theta, singular) → list of (x, y, yaw)
        rough = [(float(p[0]), float(p[1]), float(p[2])) for p in path]
        self.last_rough = rough

        # ---- 异步触发轨迹优化（不阻塞本次规划）----
        if self._optimizer is not None:
            plen = 0.0
            for i in range(1, len(rough)):
                plen += math.dist(rough[i - 1][:2], rough[i][:2])
            if plen < self.min_optimize_length:
                self._opt_skip_short += 1
            elif self._fresh_optimized(start_x, start_y) is None:
                # 仅在「没有可用优化轨迹」时才提交新任务：
                # 每 1s 重规划都提交会让 executor 一直忙、结果还没用上就过期
                self._maybe_submit_optimization(
                    path, self._sdf.sdf_map, sdf_info, start_vel, start_x, start_y)

        # ---- 选择本次交给 MPC 的轨迹 ----
        optimized = self._fresh_optimized(start_x, start_y)
        traj = optimized if optimized is not None else rough

        # 姿态验收（论文 Eq.27）: 俯仰/横滚超限则丢弃优化结果，回退 A* 粗路径
        worst = self._check_attitude(traj)
        if worst is not None and optimized is not None:
            limit = math.radians(max(self.max_pitch_deg, self.max_roll_deg))
            if worst > limit * self.attitude_tolerance:
                self._opt_attitude_reject += 1
                self._node.get_logger().warn(
                    f"traj rejected by attitude: {math.degrees(worst):.1f}° > "
                    f"max(pitch={self.max_pitch_deg:.0f}°, roll={self.max_roll_deg:.0f}°) "
                    f"→ 用 A* 粗路径")
                traj = rough

        # 风险验收（论文 Eq.28 / Eq.22 的 ρr 项）
        if optimized is not None and traj is optimized:
            risk_res = self._check_risk(traj)
            if risk_res is not None:
                max_risk, mean_risk = risk_res
                if max_risk > self.risk_limit:
                    self._opt_risk_reject += 1
                    self._node.get_logger().warn(
                        f"traj rejected by risk: max={max_risk:.2f} > limit={self.risk_limit:.2f}"
                        " → 用 A* 粗路径")
                    traj = rough
                elif mean_risk > self.risk_soft_limit:
                    self._opt_risk_reject += 1
                    self._node.get_logger().warn(
                        f"traj rejected by mean risk: {mean_risk:.2f} > soft={self.risk_soft_limit:.2f}"
                        " → 用 A* 粗路径")
                    traj = rough

        self.last_traj = traj
        return traj

    def _apply_slope_mask(self, grid: np.ndarray, sdf_info: Dict) -> np.ndarray:
        """把坡度超限的地形格标为 occupied。

        论文 Eq.27 用 φx²/φy² ≤ 上限约束车体姿态。姿态由地形决定，**不是**
        速度或轨迹形状能改变的，所以它本质上是「路径可行性」约束：等价于把
        过陡区域当作障碍，交给 A* 绕开。
        """
        if self._elev is None or self.max_slope_deg <= 0.0:
            return grid
        if self._elev.elevation is None:
            return grid
        max_slope = math.radians(self.max_slope_deg)
        elev_arr = np.asarray(self._elev.elevation)
        sdf_arr = np.asarray(self._sdf.sdf_map) if self._sdf.sdf_map is not None else None
        key = (
            hash(elev_arr.tobytes()),
            hash(sdf_arr.tobytes()) if sdf_arr is not None else 0,
            round(max_slope, 6),
            self.risk_limit,
            self.risk_astar_weight,
        )
        if self._slope_cache is not None and self._slope_cache[0] == key:
            mask, risk_mask = self._slope_cache[1], self._slope_cache[2]
        else:
            mask, risk_mask = self._compute_slope_masks(grid, max_slope, sdf_arr)
            self._slope_cache = (key, mask, risk_mask)

        if mask is None:
            return grid
        out = np.array(grid, copy=True)
        if mask.any():
            out[mask] = -1.0
        if risk_mask is not None and risk_mask.any():
            out[risk_mask] = -1.0
        return out

    def _compute_slope_masks(self, grid, max_slope, sdf_arr):
        """计算坡度超限 mask 与 risk 超限 mask（结果会被缓存）。"""
        if self._elev is None or self._elev.elevation is None:
            return None, None
        mask = self._elev.untraversable_mask(max_slope)
        if mask.shape != grid.shape:
            return None, None

        if sdf_arr is not None and sdf_arr.shape == mask.shape:
            unknown = np.abs(sdf_arr - 0.5) < 1e-3
            mask = mask & ~unknown

        if mask.any():
            try:
                from scipy.ndimage import binary_opening
                mask = binary_opening(mask, structure=np.ones((2, 2)))
            except ImportError:
                pass

        # P4: risk 硬约束（Eq.28）。默认 risk_astar_weight<0 时不做全图过滤
        # （单点约 500μs，数千抽样点需数秒，不可实时）。
        risk_mask = None
        if self.risk_astar_weight >= 0.0 and self.risk_limit < 1.0:
            risk_mask = self._risk_mask_for_astar(grid.shape)
        return mask, risk_mask

    def _risk_mask_for_astar(self, shape) -> Optional[np.ndarray]:
        """在网格上抽样评估 risk，返回 risk > risk_limit 的 mask。"""
        if self._elev is None or self._elev.elevation is None:
            return None
        elev = self._elev
        if elev.rows != shape[0] or elev.cols != shape[1]:
            return None
        mask = np.zeros(shape, dtype=bool)
        step = 3          # 每 3 格（0.3m）抽样一次
        hit = 0
        for r in range(step, shape[0] - step, step):
            for c in range(step, shape[1] - step, step):
                if not elev.valid[r, c]:
                    continue
                x, y = elev.grid_to_world(r, c)
                # 风险与朝向相关，但 A* 不知道朝向 → 取 4 个主朝向的最坏值
                worst = 0.0
                for yaw in (0.0, math.pi / 2, math.pi, -math.pi / 2):
                    rv = elev.risk_at(x, y, yaw)
                    if rv is not None and rv > worst:
                        worst = rv
                if worst > self.risk_limit:
                    mask[r - step:r + step, c - step:c + step] = True
                    hit += 1
        return mask if hit else None

    def _check_attitude(
        self, traj: List[Tuple[float, float, float]]
    ) -> Optional[float]:
        """验收轨迹上的俯仰/横滚角，返回最大姿态角 (rad)；无地形数据返回 None。"""
        if self._elev is None or self._elev.elevation is None or not traj:
            return None
        max_pitch = math.radians(self.max_pitch_deg)
        max_roll = math.radians(self.max_roll_deg)
        worst = 0.0
        for x, y, th in traj:
            att = self._elev.attitude_at(x, y, th)
            if att is None:
                continue
            pitch, roll = att
            worst = max(worst, abs(pitch), abs(roll))
            if abs(pitch) > max_pitch or abs(roll) > max_roll:
                return worst
        return worst

    def _check_risk(
        self, traj: List[Tuple[float, float, float]]
    ) -> Optional[Tuple[float, float]]:
        """验收轨迹上的可通行风险，返回 (max_risk, mean_risk)；无地形数据返回 None。

        对应论文 Eq.28（Risk ≤ rmax 硬约束）与 Eq.22 的 ρr·∫Risk² 项：
        硬约束逐点检查，软项用全程平均风险近似（我们不对 risk 做插值积分）。
        """
        if self._elev is None or self._elev.elevation is None or not traj:
            return None
        risks: List[float] = []
        for x, y, th in traj:
            rv = self._elev.risk_at(x, y, th)
            if rv is not None:
                risks.append(rv)
        if not risks:
            return None
        return max(risks), sum(risks) / len(risks)

    def _inflate_sdf(self, grid: np.ndarray) -> np.ndarray:
        """按车体半径膨胀障碍物（返回新的栅格，不修改输入）。"""
        if self.inflation_radius <= 0.0:
            return grid
        try:
            from scipy.ndimage import binary_dilation
        except ImportError:
            return grid
        res = self._sdf.resolution
        iters = int(math.ceil(self.inflation_radius / max(res, 1e-6)))
        if iters <= 0:
            return grid
        occupied = np.asarray(grid) < 0.0
        if not occupied.any():
            return grid
        inflated = binary_dilation(occupied, iterations=iters)
        out = np.array(grid, copy=True)
        out[inflated] = -1.0
        return out

    # ------------------------------------------------------------------
    # 轨迹优化（异步）
    # ------------------------------------------------------------------

    def _maybe_submit_optimization(
        self,
        astar_path: list,
        sdf_grid,
        sdf_info: Dict,
        start_vel: float,
        start_x: float,
        start_y: float,
    ) -> None:
        """若上一个优化任务已完成则提交新任务。"""
        if self._executor is None:
            return
        if self._opt_future is not None and not self._opt_future.done():
            return  # 上一次还在算，跳过本次（越级提交只会堆积）
        self._opt_future = self._executor.submit(
            self._optimize_worker,
            list(astar_path), sdf_grid, dict(sdf_info), float(start_vel),
            float(start_x), float(start_y),
        )

    def _optimize_worker(
        self,
        astar_path: list,
        sdf_grid,
        sdf_info: Dict,
        start_vel: float,
        start_x: float,
        start_y: float,
    ) -> None:
        """executor 线程内运行：求解并原子替换结果。"""
        try:
            res = self._optimizer.optimize(
                astar_path,
                start_vel=start_vel,
                sdf_grid=sdf_grid,
                sdf_info=sdf_info,
            )
        except Exception as exc:
            with self._opt_lock:
                self._opt_fail += 1
            self._node.get_logger().warn(f"traj opt exception: {exc}")
            return

        with self._opt_lock:
            if res.success and res.samples:
                self._opt_samples = list(res.samples)
                self._opt_start = (start_x, start_y)
                self._opt_time = time.time()
                self._opt_ok += 1
            else:
                # 失败即让旧结果失效，避免继续用不安全/过期的轨迹
                self._opt_samples = None
                self._opt_start = None
                self._opt_fail += 1

        if res.success:
            self._node.get_logger().info(
                f"traj opt ok: Tf={res.duration:.1f}s "
                f"min_clr={res.min_clearance:.2f}m "
                f"k={res.max_kappa:.2f} {res.solve_time_ms:.0f}ms "
                f"(ok/fail={self._opt_ok}/{self._opt_fail})"
            )
        else:
            opt = self._optimizer
            self._node.get_logger().warn(
                f"traj opt rejected: v={res.max_speed:.2f}/{opt.max_speed:.2f} "
                f"k={res.max_kappa:.2f}/{opt.kappa_limit:.2f} "
                f"| {res.status[:60]} (ok/fail={self._opt_ok}/{self._opt_fail}) "
                "→ 用 A* 粗路径")

    def _fresh_optimized(
        self, start_x: float, start_y: float
    ) -> Optional[List[Tuple[float, float, float]]]:
        """取可用的优化轨迹：起点接近且未过期，否则 None。"""
        with self._opt_lock:
            samples = self._opt_samples
            opt_start = self._opt_start
            opt_time = self._opt_time

        if not samples:
            return None
        if time.time() - opt_time > self.optimize_valid_s:
            return None
        # 机器人必须仍在轨迹附近（横向偏离小），否则说明轨迹已不适用
        dists = [math.dist((start_x, start_y), (sx, sy)) for sx, sy, _th, _t in samples]
        idx_min = min(range(len(dists)), key=lambda i: dists[i])
        if dists[idx_min] > self.optimize_offtrack_tolerance:
            return None

        # 裁掉机器人身后（已走过）的轨迹段。
        # 不裁剪会把身后的点也发给 MPC：MPC 的参考点可能落在机器人后方，
        # 于是输出负速度试图「退回」轨迹起点（实测 v 均值 -0.067 m/s，一直在倒车）。
        pts = [(float(x), float(y), float(th))
               for x, y, th, _t in samples[idx_min:]]
        if len(pts) < 2:
            return None

        # 剩余轨迹太短时不要用（会让 MPC 视界枯竭、反复外推）
        remaining = sum(
            math.dist(pts[i - 1][:2], pts[i][:2]) for i in range(1, len(pts))
        )
        if remaining < self.optimize_min_remaining:
            return None

        return self._resample_by_arc(pts, self.traj_publish_step)

    @staticmethod
    def _resample_by_arc(
        pts: List[Tuple[float, float, float]], step: float
    ) -> List[Tuple[float, float, float]]:
        """按固定弧长间隔重采样（保留首末点）。"""
        if len(pts) < 3 or step <= 0.0:
            return pts
        out = [pts[0]]
        acc = 0.0
        for i in range(1, len(pts)):
            acc += math.dist(pts[i - 1][:2], pts[i][:2])
            if acc >= step:
                out.append(pts[i])
                acc = 0.0
        if out[-1] is not pts[-1]:
            out.append(pts[-1])
        return out


# ---------------------------------------------------------------------------
# ReplanFSM Node
# ---------------------------------------------------------------------------
class SebNaverPlannerNode(Node):
    """ROS2 node implementing the ReplanFSM for SEB-Naver navigation."""

    def __init__(self) -> None:
        super().__init__("seb_naver_planner")

        # ---- Declare parameters ----
        self.declare_parameter("wheel_base", 0.315)
        self.declare_parameter("max_speed", 0.5)
        self.declare_parameter("max_steer", 0.785)
        self.declare_parameter("step_arc", 1.0)
        self.declare_parameter("steer_res", 0.3)
        self.declare_parameter("goal_tolerance_dist", 1.0)
        self.declare_parameter("goal_tolerance_yaw", 0.15)
        self.declare_parameter("goal_tolerance_vel", 0.05)
        self.declare_parameter("replan_interval", 0.5)
        # ---- 轨迹优化（P2）----
        self.declare_parameter("enable_optimization", True)
        self.declare_parameter("trajectory_dmin", 0.28)
        self.declare_parameter("trajectory_seg_len", 0.9)
        self.declare_parameter("trajectory_max_accel", 1.0)
        self.declare_parameter("trajectory_sdf_iters", 5)
        # 优化结果的有效性：起点须接近、结果须新鲜
        self.declare_parameter("optimize_offtrack_tolerance", 0.8)
        self.declare_parameter("optimize_valid_s", 4.0)
        self.declare_parameter("optimize_min_length", 2.0)
        self.declare_parameter("traj_publish_step", 0.3)
        self.declare_parameter("optimize_min_remaining", 1.5)
        self.declare_parameter("inflation_radius", 0.25)
        # ---- 地形姿态 (P3) ----
        self.declare_parameter("max_slope_deg", 25.0)   # 超过则不可通行 (A* 绕开)
        self.declare_parameter("max_pitch_deg", 20.0)   # 轨迹验收: 俯仰
        self.declare_parameter("max_roll_deg", 15.0)    # 轨迹验收: 横滚
        self.declare_parameter("attitude_tolerance", 1.3)  # 验收容差倍数
        # ---- 可通行性风险 (P4) ----
        self.declare_parameter("risk_limit", 0.8)       # 论文 rmax: 超过当障碍
        self.declare_parameter("risk_soft_limit", 0.5)  # 轨迹平均风险软阈值
        self.declare_parameter("risk_astar_weight", -1.0)  # >=0 启用 A* risk 过滤

        # Cache tolerances
        self._tol_dist: float = self.get_parameter("goal_tolerance_dist").value
        self._tol_yaw: float = self.get_parameter("goal_tolerance_yaw").value
        self._tol_vel: float = self.get_parameter("goal_tolerance_vel").value
        self._replan_interval: float = self.get_parameter("replan_interval").value

        # ---- Internal state ----
        self._fsm_state: FSMState = FSMState.INIT
        self._odom: Optional[Odometry] = None
        self._goal: Optional[PoseStamped] = None
        self._current_path: Optional[list] = None  # list of (x, y, yaw)
        self._last_replan_time: float = 0.0

        # SDF grid parser
        self._sdf_parser = SDFGridParser()
        self._elev_parser = ElevationGridParser()

        # Plan manager
        self._plan_manager = PlanManager(self, self._sdf_parser, self._elev_parser)

        # ---- QoS profiles ----
        # /Odometry (FAST-LIO2) 和 /sdf_map (terrain_analyzer) 都用 RELIABLE 发布
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---- Subscribers ----
        self._odom_sub = self.create_subscription(
            Odometry, "/Odometry", self._odom_callback, sensor_qos
        )
        self._sdf_sub = self.create_subscription(
            SE2Grid, "/sdf_map", self._sdf_callback, sensor_qos
        )
        self._fused_sub = self.create_subscription(
            SE2Grid, "/fused_map", self._fused_callback, sensor_qos
        )
        self._goal_sub = self.create_subscription(
            PoseStamped, "/goal_pose", self._goal_callback, 10
        )

        # ---- Publishers ----
        self._traj_pub = self.create_publisher(Path, "/planner/trajectory", 10)
        self._rough_path_pub = self.create_publisher(Path, "/planner/rough_path", 10)
        self._status_pub = self.create_publisher(String, "/planner/status", 10)

        # ---- FSM timer (50 Hz) ----
        self._fsm_timer = self.create_timer(0.02, self._fsm_tick)

        self.get_logger().info("SebNaverPlannerNode initialized, entering INIT state")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _odom_callback(self, msg: Odometry) -> None:
        self._odom = msg

    def _sdf_callback(self, msg: SE2Grid) -> None:
        self._sdf_parser.update_from_msg(msg)

    def _fused_callback(self, msg: SE2Grid) -> None:
        """地形高程图（P3）: 用于坡度过滤与姿态验收。"""
        self._elev_parser.update_from_msg(msg)

    def _goal_callback(self, msg: PoseStamped) -> None:
        self._goal = msg
        self.get_logger().info(
            f"Goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})"
        )
        if self._fsm_state == FSMState.WAIT_TARGET:
            self._transition(FSMState.SEQUENTIAL_START)

    # ------------------------------------------------------------------
    # FSM transitions
    # ------------------------------------------------------------------
    def _transition(self, new_state: FSMState) -> None:
        old = self._fsm_state.name
        self._fsm_state = new_state
        self.get_logger().info(f"FSM: {old} → {new_state.name}")
        status_msg = String()
        status_msg.data = new_state.name
        self._status_pub.publish(status_msg)

    # ------------------------------------------------------------------
    # FSM tick (50 Hz)
    # ------------------------------------------------------------------
    def _fsm_tick(self) -> None:
        state = self._fsm_state

        if state == FSMState.INIT:
            self._transition(FSMState.WAIT_TARGET)

        elif state == FSMState.WAIT_TARGET:
            # Idle — waiting for goal via callback
            pass

        elif state == FSMState.SEQUENTIAL_START:
            self._do_initial_plan()

        elif state == FSMState.EXEC_TRAJ:
            self._exec_traj_tick()

        elif state == FSMState.REPLAN_TRAJ:
            self._do_replan()

    # ------------------------------------------------------------------
    # Planning actions
    # ------------------------------------------------------------------
    def _do_initial_plan(self) -> None:
        """First planning after receiving a goal (SEQUENTIAL_START)."""
        if self._odom is None:
            self.get_logger().warn("SEQUENTIAL_START: No odometry yet, waiting...")
            return

        if self._goal is None:
            self.get_logger().warn("SEQUENTIAL_START: No goal set, returning to WAIT_TARGET")
            self._transition(FSMState.WAIT_TARGET)
            return

        if self._sdf_parser.sdf_map is None:
            self.get_logger().warn("SEQUENTIAL_START: SDF map not received yet, waiting...")
            return

        path = self._run_planning_pipeline()
        if path is not None:
            self._current_path = path
            self._last_replan_time = time.time()
            self._transition(FSMState.EXEC_TRAJ)
        else:
            self.get_logger().error("SEQUENTIAL_START: Planning failed, staying in state")

    def _do_replan(self) -> None:
        """Replanning during execution (REPLAN_TRAJ)."""
        if self._odom is None or self._goal is None:
            self._transition(FSMState.WAIT_TARGET)
            return

        if self._sdf_parser.sdf_map is None:
            self.get_logger().warn("REPLAN_TRAJ: SDF map not available, returning to EXEC_TRAJ")
            self._transition(FSMState.EXEC_TRAJ)
            return

        path = self._run_planning_pipeline()
        if path is not None:
            self._current_path = path
            self._last_replan_time = time.time()

        self._transition(FSMState.EXEC_TRAJ)

    def _run_planning_pipeline(self) -> Optional[list]:
        """Execute the full planning pipeline and publish results.

        Returns the planned path or None on failure.
        """
        odom = self._odom
        goal = self._goal
        if odom is None or goal is None:
            return None

        # Current state from odometry
        start_x = odom.pose.pose.position.x
        start_y = odom.pose.pose.position.y
        start_yaw = quat_to_yaw(odom.pose.pose.orientation)
        start_vel = math.sqrt(
            odom.twist.twist.linear.x ** 2 + odom.twist.twist.linear.y ** 2
        )

        # Goal
        goal_x = goal.pose.position.x
        goal_y = goal.pose.position.y
        goal_yaw = quat_to_yaw(goal.pose.orientation)

        # Run planner
        path = self._plan_manager.plan(
            start_x, start_y, start_yaw, start_vel,
            goal_x, goal_y, goal_yaw,
        )

        if path is None:
            return None

        # A* 粗路径（可视化 / 调试）
        if self._plan_manager.last_rough:
            self._publish_path(self._plan_manager.last_rough, self._rough_path_pub, "map")
        # 交给 MPC 的轨迹（优化结果，或回退的 A* 粗路径）
        optimized = path is not self._plan_manager.last_rough
        self._publish_path(path, self._traj_pub, "map")

        self.get_logger().info(
            f"Planning pipeline: rough={len(self._plan_manager.last_rough)} pts, "
            f"traj={len(path)} pts ({'optimized' if optimized else 'A* fallback'})")
        return path

    # ------------------------------------------------------------------
    # Execution monitoring
    # ------------------------------------------------------------------
    def _exec_traj_tick(self) -> None:
        """Monitor trajectory execution: check arrival and replan timer."""
        if self._odom is None or self._goal is None:
            self.get_logger().warn("EXEC_TRAJ: Lost odometry or goal, returning to WAIT_TARGET")
            self._transition(FSMState.WAIT_TARGET)
            return

        # Check arrival condition
        if self._check_arrived():
            self.get_logger().info("Goal reached!")
            self._current_path = None
            self._goal = None
            self._transition(FSMState.WAIT_TARGET)
            return

        # Check replan timer
        elapsed = time.time() - self._last_replan_time
        if elapsed >= self._replan_interval:
            self._transition(FSMState.REPLAN_TRAJ)

    def _check_arrived(self) -> bool:
        """Check if the robot has arrived at the goal.

        Arrival criteria:
          - distance < goal_tolerance_dist
          - |yaw_error| < goal_tolerance_yaw
          - speed < goal_tolerance_vel
        """
        odom = self._odom
        goal = self._goal
        if odom is None or goal is None:
            return False

        dx = goal.pose.position.x - odom.pose.pose.position.x
        dy = goal.pose.position.y - odom.pose.pose.position.y
        dist = math.sqrt(dx * dx + dy * dy)

        current_yaw = quat_to_yaw(odom.pose.pose.orientation)
        goal_yaw = quat_to_yaw(goal.pose.orientation)
        yaw_err = abs(normalize_angle(goal_yaw - current_yaw))

        speed = math.sqrt(
            odom.twist.twist.linear.x ** 2 + odom.twist.twist.linear.y ** 2
        )

        return (
            dist < self._tol_dist
            and yaw_err < self._tol_yaw
            and speed < self._tol_vel
        )

    # ------------------------------------------------------------------
    # Path publishing
    # ------------------------------------------------------------------
    def _publish_path(
        self,
        waypoints: list,
        publisher,
        frame_id: str = "map",
    ) -> None:
        """Convert a list of (x, y, yaw) waypoints to nav_msgs/Path and publish."""
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = frame_id

        for wp in waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose.position = Point(x=float(wp[0]), y=float(wp[1]), z=0.0)
            yaw = float(wp[2]) if len(wp) > 2 else 0.0
            pose_stamped.pose.orientation = yaw_to_quat(yaw)
            path_msg.poses.append(pose_stamped)

        publisher.publish(path_msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None) -> None:
    rclpy.init(args=args)
    node = SebNaverPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

"""
Kinodynamic A* path planner for Ackermann car-like robots.

Ported from the SEB-Naver paper's C++ implementation. Searches in (x, y, θ)
state space using bicycle-model motion primitives with SDF-based collision
checking and Reeds-Shepp goal connection.

Designed for ~5 Hz replanning on typical indoor/outdoor maps.
"""

import heapq
import math
import time
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Dict, List, Optional, Tuple

import numpy as np


class SearchStatus(IntEnum):
    """Result codes for the search."""
    SUCCESS = 0
    TIMEOUT = 1
    NO_PATH = 2
    INVALID_START = 3
    INVALID_GOAL = 4


@dataclass
class KinoNode:
    """A node in the Kinodynamic A* open set."""
    g: float          # cost-so-far
    f: float          # g + λ*h
    x: float
    y: float
    theta: float
    steer: float      # steering angle used to reach this node
    direction: int    # +1 forward, -1 reverse
    parent_idx: int   # index into closed list, -1 for start
    singular: bool    # True if near-zero turning radius (straight segment)

    def __lt__(self, other: "KinoNode") -> bool:
        return self.f < other.f


# Discretization constants for state hashing
_POS_DISC = 0.1       # metres — match map_resolution default
_ANGLE_DISC = 0.1     # radians (~5.7°)
_STEER_DISC = 0.1     # radians for steering discretisation in hash


def _hash_state(x: float, y: float, theta: float) -> int:
    """Discretise a continuous (x, y, θ) state to an integer key."""
    ix = int(round(x / _POS_DISC))
    iy = int(round(y / _POS_DISC))
    ith = int(round(theta / _ANGLE_DISC))
    # Cantor-like pairing extended to 3D; use large primes to avoid collisions
    return ((ix * 73856093) ^ (iy * 19349663) ^ (ith * 83492791)) & 0x7FFFFFFF


def _normalize_angle(a: float) -> float:
    """Wrap angle to [-π, π]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class KinoAStarPlanner:
    """Kinodynamic A* planner for Ackermann / bicycle-model vehicles."""

    def __init__(self, params: dict):
        # Vehicle parameters
        self.wheel_base: float = params.get("wheel_base", 0.315)
        self.max_steer: float = params.get("max_steer", 0.785)

        # Motion primitive parameters
        self.step_arc: float = params.get("step_arc", 1.5)
        self.steer_res: float = params.get("steer_res", 0.5)
        self.check_num: int = params.get("check_num", 3)

        # Cost weights
        self.forward_penalty: float = params.get("forward_penalty", 1.0)
        self.back_penalty: float = params.get("back_penalty", 1.5)
        self.gear_switch_penalty: float = params.get("gear_switch_penalty", 2.0)
        self.steer_penalty: float = params.get("steer_penalty", 0.5)
        self.steer_change_penalty: float = params.get("steer_change_penalty", 0.3)

        # Search parameters
        self.lambda_heu: float = params.get("lambda_heu", 1.0)
        self.max_search_time: float = params.get("max_search_time", 5.0)
        self.map_resolution: float = params.get("map_resolution", 0.1)

        # RS connection threshold
        self.rs_threshold: float = params.get("rs_threshold", 1.5)

        # Pre-compute steering angles for motion primitives
        self._build_steering_angles()

        # Arc lengths for primitives
        self.arc_lengths: List[float] = [
            -self.step_arc,
            -0.5 * self.step_arc,
            0.5 * self.step_arc,
            self.step_arc,
        ]

        # SDF grid storage
        self._sdf_grid: Optional[np.ndarray] = None
        self._sdf_origin_x: float = 0.0
        self._sdf_origin_y: float = 0.0
        self._sdf_width: int = 0
        self._sdf_height: int = 0
        self._sdf_resolution: float = self.map_resolution

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set_sdf_grid(self, grid_data: np.ndarray, info: dict) -> None:
        """Update the SDF grid used for collision checking.

        Parameters
        ----------
        grid_data : np.ndarray
            2-D array of signed-distance values (metres). Positive = free.
        info : dict
            Must contain ``origin`` (list/tuple [x, y]), ``resolution`` (float),
            and optionally ``width`` / ``height`` (inferred from shape if absent).
        """
        self._sdf_grid = np.asarray(grid_data, dtype=np.float32)
        origin = info.get("origin", [0.0, 0.0])
        self._sdf_origin_x = float(origin[0])
        self._sdf_origin_y = float(origin[1])
        self._sdf_resolution = float(info.get("resolution", self.map_resolution))
        self._sdf_height, self._sdf_width = self._sdf_grid.shape

    def search(
        self,
        start_state: Tuple[float, float, float],
        goal_state: Tuple[float, float, float],
        sdf_grid_data: Optional[np.ndarray] = None,
        sdf_info: Optional[dict] = None,
    ) -> Tuple[SearchStatus, List[Tuple[float, float, float, bool]]]:
        """Run Kinodynamic A* search.

        Parameters
        ----------
        start_state : (x, y, theta)
        goal_state  : (x, y, theta)
        sdf_grid_data : optional update of the SDF grid
        sdf_info      : metadata accompanying *sdf_grid_data*

        Returns
        -------
        status : SearchStatus
        path   : list of (x, y, theta, singular) tuples from start to goal
        """
        if sdf_grid_data is not None and sdf_info is not None:
            self.set_sdf_grid(sdf_grid_data, sdf_info)

        sx, sy, sth = start_state
        gx, gy, gth = goal_state

        # Validate start/goal against SDF.
        # 越界时钳制到地图内 (地图滚动跟随机器人, 目标常落在窗口边界外)
        if self._sdf_grid is not None:
            if not self._point_in_bounds(sx, sy):
                sx, sy = self._clamp_to_bounds(sx, sy)
            if not self._point_in_bounds(gx, gy):
                gx, gy = self._clamp_to_bounds(gx, gy)

        t_start = time.monotonic()

        # Closed list: hash -> best g-cost
        closed: Dict[int, float] = {}
        # Flat list of nodes for parent back-tracking
        nodes: List[KinoNode] = []

        h0 = self._get_heuristic(sx, sy, gx, gy)
        start_node = KinoNode(
            g=0.0, f=self.lambda_heu * h0,
            x=sx, y=sy, theta=_normalize_angle(sth),
            steer=0.0, direction=1, parent_idx=-1, singular=False,
        )
        heapq.heapqueue = []  # type: ignore[attr-defined]  # will use local var
        open_set: List[KinoNode] = [start_node]
        nodes.append(start_node)
        start_hash = _hash_state(sx, sy, sth)
        closed[start_hash] = 0.0

        while open_set:
            # Timeout check every 1000 expansions
            if len(nodes) % 1000 == 0:
                if time.monotonic() - t_start > self.max_search_time:
                    return SearchStatus.TIMEOUT, []

            current = heapq.heappop(open_set)
            cur_idx = nodes.index(current) if current not in nodes else nodes.index(current)
            # Find actual index by identity
            cur_idx = -1
            for i in range(len(nodes) - 1, -1, -1):
                if nodes[i] is current:
                    cur_idx = i
                    break
            if cur_idx < 0:
                continue

            cur_hash = _hash_state(current.x, current.y, current.theta)
            if cur_hash in closed and closed[cur_hash] < current.g - 1e-9:
                continue

            # Goal proximity — try Reeds-Shepp shot
            dist_to_goal = math.hypot(current.x - gx, current.y - gy)
            if dist_to_goal < self.rs_threshold:
                rs_path = self._reeds_shepp_shot(
                    current.x, current.y, current.theta,
                    gx, gy, gth,
                )
                if rs_path is not None:
                    # Build full path
                    prefix = self._backtrack(nodes, cur_idx)
                    full_path = prefix + rs_path
                    return SearchStatus.SUCCESS, full_path

            # Expand motion primitives
            for arc in self.arc_lengths:
                direction = 1 if arc >= 0 else -1
                for psi in self._steer_angles:
                    nx, ny, nth, singular = self._state_trans(
                        current.x, current.y, current.theta, psi, arc,
                    )

                    # Collision check along the primitive
                    if not self._check_collision(
                        current.x, current.y, current.theta,
                        nx, ny, nth, psi, arc,
                    ):
                        continue

                    # Cost computation
                    abs_arc = abs(arc)
                    if direction == 1:
                        travel_cost = abs_arc * self.forward_penalty
                    else:
                        travel_cost = abs_arc * self.back_penalty

                    # Gear switch penalty
                    if direction != current.direction:
                        travel_cost += self.gear_switch_penalty

                    # Steering effort
                    travel_cost += self.steer_penalty * abs(psi) * abs_arc

                    # Steering change
                    travel_cost += self.steer_change_penalty * abs(psi - current.steer)

                    new_g = current.g + travel_cost
                    nhash = _hash_state(nx, ny, nth)

                    if nhash in closed and closed[nhash] <= new_g + 1e-9:
                        continue

                    h = self._get_heuristic(nx, ny, gx, gy)
                    child = KinoNode(
                        g=new_g,
                        f=new_g + self.lambda_heu * h,
                        x=nx, y=ny, theta=nth,
                        steer=psi, direction=direction,
                        parent_idx=cur_idx, singular=singular,
                    )
                    closed[nhash] = new_g
                    nodes.append(child)
                    heapq.heappush(open_set, child)

        return SearchStatus.NO_PATH, []

    # ------------------------------------------------------------------
    # Kinematic model
    # ------------------------------------------------------------------

    def _state_trans(
        self,
        x0: float, y0: float, th0: float,
        psi: float, s: float,
    ) -> Tuple[float, float, float, bool]:
        """Bicycle-model forward kinematics.

        Returns (x1, y1, θ1, singular) where *singular* is True when the
        turning radius is effectively infinite (straight-line motion).
        """
        singular = False
        if abs(psi) < 0.01:
            # Straight-line approximation
            x1 = x0 + s * math.cos(th0)
            y1 = y0 + s * math.sin(th0)
            th1 = th0
            singular = True
        else:
            k = self.wheel_base / math.tan(psi)
            dth = s / k
            x1 = x0 + k * (math.sin(th0 + dth) - math.sin(th0))
            y1 = y0 - k * (math.cos(th0 + dth) - math.cos(th0))
            th1 = th0 + dth

        return x1, y1, _normalize_angle(th1), singular

    # ------------------------------------------------------------------
    # Heuristic
    # ------------------------------------------------------------------

    @staticmethod
    def _get_heuristic(x: float, y: float, gx: float, gy: float) -> float:
        """Euclidean distance heuristic (admissible for any penalty ≥ 1)."""
        return math.hypot(x - gx, y - gy)

    # ------------------------------------------------------------------
    # Collision checking via SDF grid
    # ------------------------------------------------------------------

    def _point_in_bounds(self, x: float, y: float) -> bool:
        """Check whether a world-frame point falls inside the SDF grid."""
        if self._sdf_grid is None:
            return True
        col = (x - self._sdf_origin_x) / self._sdf_resolution
        row = (y - self._sdf_origin_y) / self._sdf_resolution
        # 含边界 (col <= width), 避免恰好落在右/上边缘时被误判越界
        return 0 <= col <= self._sdf_width and 0 <= row <= self._sdf_height

    def _clamp_to_bounds(self, x: float, y: float) -> Tuple[float, float]:
        """Clamp a world point into the valid SDF grid interior (避免边界外的点被拒绝)."""
        if self._sdf_grid is None:
            return x, y
        # 保留一格安全边距 (地图最外圈可能无数据)
        margin = self._sdf_resolution * 2.0
        max_x = self._sdf_origin_x + (self._sdf_width - 1) * self._sdf_resolution - margin
        max_y = self._sdf_origin_y + (self._sdf_height - 1) * self._sdf_resolution - margin
        min_x = self._sdf_origin_x + margin
        min_y = self._sdf_origin_y + margin
        return float(np.clip(x, min_x, max_x)), float(np.clip(y, min_y, max_y))

    def _query_sdf(self, x: float, y: float) -> float:
        """Bilinear-interpolated SDF lookup. Returns large positive if OOB."""
        if self._sdf_grid is None:
            return 1.0  # no grid → assume free

        fx = (x - self._sdf_origin_x) / self._sdf_resolution
        fy = (y - self._sdf_origin_y) / self._sdf_resolution

        c0 = int(math.floor(fx))
        r0 = int(math.floor(fy))

        if c0 < 0 or r0 < 0 or c0 + 1 >= self._sdf_width or r0 + 1 >= self._sdf_height:
            return -1.0  # out of bounds → collision

        dx = fx - c0
        dy = fy - r0

        # Bilinear interpolation
        v00 = self._sdf_grid[r0, c0]
        v10 = self._sdf_grid[r0, c0 + 1]
        v01 = self._sdf_grid[r0 + 1, c0]
        v11 = self._sdf_grid[r0 + 1, c0 + 1]

        val = (v00 * (1 - dx) * (1 - dy)
               + v10 * dx * (1 - dy)
               + v01 * (1 - dx) * dy
               + v11 * dx * dy)
        return float(val)

    def _check_collision(
        self,
        x0: float, y0: float, th0: float,
        x1: float, y1: float, th1: float,
        psi: float, arc: float,
    ) -> bool:
        """Return True if the motion primitive is collision-free.

        Samples *check_num* intermediate points along the arc using the
        same bicycle model so that curved trajectories are checked properly.
        """
        if self._sdf_grid is None:
            return True

        n = max(self.check_num, 1)
        for i in range(n + 1):
            frac = i / n
            s_frac = arc * frac
            px, py, _, _ = self._state_trans(x0, y0, th0, psi, s_frac)
            sdf_val = self._query_sdf(px, py)
            if sdf_val < 0.0:
                return False
        return True

    # ------------------------------------------------------------------
    # Reeds-Shepp goal connection
    # ------------------------------------------------------------------

    def _reeds_shepp_shot(
        self,
        x0: float, y0: float, th0: float,
        x1: float, y1: float, th1: float,
    ) -> Optional[List[Tuple[float, float, float, bool]]]:
        """Attempt a direct Reeds-Shepp connection to the goal.

        Returns a short list of waypoints if the path is collision-free,
        or None if it collides or exceeds curvature limits.
        """
        # Transform goal into start frame
        dx = x1 - x0
        dy = y1 - y0
        c = math.cos(th0)
        s = math.sin(th0)
        lx = c * dx + s * dy
        ly = -s * dx + c * dy
        lth = _normalize_angle(th1 - th0)

        # Minimum turning radius
        r_min = self.wheel_base / math.tan(self.max_steer)
        total_dist = math.hypot(lx, ly)

        if total_dist < 1e-6 and abs(lth) < 1e-6:
            return [(x1, y1, th1, False)]

        # Simple CSC / CCC approximation: sample a few candidate paths
        # For production use, a full RS library would be better; here we
        # implement the 6 canonical word families at minimum radius.
        best_path = self._rs_shortest(lx, ly, lth, r_min)
        if best_path is None:
            return None

        # Verify collision along the RS path
        path_world: List[Tuple[float, float, float, bool]] = []
        cx, cy, cth = x0, y0, th0
        for seg_type, seg_len in best_path:
            steps = max(int(abs(seg_len) / 0.1), 2)
            for j in range(1, steps + 1):
                frac = j / steps
                ds = seg_len * frac
                if seg_type == 'L':
                    # Left turn at min radius
                    kappa = 1.0 / r_min
                    dth = ds * kappa
                    if abs(dth) > 1e-9:
                        rc = r_min
                        ncx = cx + rc * (math.sin(cth + dth) - math.sin(cth))
                        ncy = cy - rc * (math.cos(cth + dth) - math.cos(cth))
                    else:
                        ncx = cx + ds * math.cos(cth)
                        ncy = cy + ds * math.sin(cth)
                    ncth = _normalize_angle(cth + dth)
                    path_world.append((ncx, ncy, ncth, False))
                elif seg_type == 'R':
                    kappa = -1.0 / r_min
                    dth = ds * kappa
                    if abs(dth) > 1e-9:
                        rc = r_min
                        ncx = cx - rc * (math.sin(cth + dth) - math.sin(cth))
                        ncy = cy + rc * (math.cos(cth + dth) - math.cos(cth))
                    else:
                        ncx = cx + ds * math.cos(cth)
                        ncy = cy + ds * math.sin(cth)
                    ncth = _normalize_angle(cth + dth)
                    path_world.append((ncx, ncy, ncth, False))
                else:  # 'S' straight
                    ncx = cx + ds * math.cos(cth)
                    ncy = cy + ds * math.sin(cth)
                    ncth = cth
                    path_world.append((ncx, ncy, ncth, True))
                # Collision check each sampled point
                if not self._point_in_bounds(ncx, ncy):
                    return None
                if self._query_sdf(ncx, ncy) < 0.0:
                    return None
            # Update current pose to end of segment
            cx, cy, cth = path_world[-1][0], path_world[-1][1], path_world[-1][2]

        return path_world

    def _rs_shortest(
        self, x: float, y: float, phi: float, r: float,
    ) -> Optional[List[Tuple[str, float]]]:
        """Compute shortest Reeds-Shepp path among the 6 canonical families.

        Returns list of (segment_type, signed_length) or None.
        Segment types: 'L' (left turn), 'R' (right turn), 'S' (straight).
        All lengths are normalised by r internally.
        """
        # Normalise to unit turning radius
        xi = x / r
        eta = y / r
        ph = _normalize_angle(phi)

        candidates: List[Tuple[float, List[Tuple[str, float]]]] = []

        # Helper: CSC families
        # L+S+L+, L+S+R+, R+S+R+, R+S+L+
        # and their reverse-gear variants
        tau, omega = self._tau_omega(xi, eta, ph)
        if tau is not None:
            # L+S+L+
            p = math.sqrt(xi * xi + eta * eta + 2.0 * xi * math.sin(ph)
                          - 2.0 * eta * math.cos(ph) + 2.0 - 2.0 * math.cos(ph - tau + omega))
            if not math.isnan(p):
                candidates.append((abs(tau) + abs(p) + abs(omega),
                                   [('L', tau * r), ('S', p * r), ('L', omega * r)]))

        # R+S+R+
        tau2, omega2 = self._tau_omega(-xi, -eta, -ph)
        if tau2 is not None:
            p2 = math.sqrt(xi * xi + eta * eta - 2.0 * xi * math.sin(ph)
                           + 2.0 * eta * math.cos(ph) + 2.0 - 2.0 * math.cos(ph - tau2 + omega2))
            if not math.isnan(p2):
                candidates.append((abs(tau2) + abs(p2) + abs(omega2),
                                   [('R', -tau2 * r), ('S', -p2 * r), ('R', -omega2 * r)]))

        # L+S+R+
        u_lr, t_lr = self._polar(xi - math.sin(ph), eta + math.cos(ph) - 1.0)
        if u_lr is not None and u_lr >= 0:
            v_lr = _normalize_angle(ph - t_lr)
            candidates.append((t_lr + u_lr + abs(v_lr),
                               [('L', t_lr * r), ('S', u_lr * r), ('R', v_lr * r)]))

        # R+S+L+
        u_rl, t_rl = self._polar(xi + math.sin(ph), eta - math.cos(ph) - 1.0)
        if u_rl is not None and u_rl >= 0:
            v_rl = _normalize_angle(ph - t_rl)
            candidates.append((t_rl + u_rl + abs(v_rl),
                               [('R', -t_rl * r), ('S', -u_rl * r), ('L', -v_rl * r)]))

        # CCC families (simplified — only LRL and RLR cusps)
        # L+R-L+
        xi2 = xi - math.sin(ph)
        eta2 = eta + math.cos(ph) - 1.0
        rho_ccc = (xi2 * xi2 + eta2 * eta2)
        if rho_ccc <= 4.0:
            u_ccc = math.sqrt(max(0.0, 4.0 - rho_ccc)) / 2.0  # half-chord
            t_ccc = math.atan2(eta2, xi2) - math.atan2(u_ccc, math.sqrt(max(0, 1.0 - u_ccc * u_ccc)))
            v_ccc = _normalize_angle(ph - 2.0 * t_ccc)
            cost_ccc = abs(t_ccc) + abs(math.acos(min(1.0, max(-1.0, 1.0 - rho_ccc / 2.0)))) + abs(v_ccc)
            if not math.isnan(cost_ccc):
                alpha_ccc = math.acos(min(1.0, max(-1.0, 1.0 - rho_ccc / 2.0)))
                candidates.append((cost_ccc,
                                   [('L', t_ccc * r), ('R', -alpha_ccc * r), ('L', v_ccc * r)]))

        if not candidates:
            return None

        candidates.sort(key=lambda c: c[0])
        return candidates[0][1]

    @staticmethod
    def _tau_omega(xi: float, eta: float, phi: float):
        """Helper for CSC-type RS paths."""
        delta = _normalize_angle(phi)
        denom = 2.0 * (1.0 - math.cos(delta))
        if abs(denom) < 1e-12:
            return None, None
        tau = math.atan2(eta - math.sin(delta), xi + math.cos(delta) - 1.0)
        # This is a simplified version; full RS requires more cases
        omega = _normalize_angle(delta - tau)
        return tau, omega

    @staticmethod
    def _polar(x: float, y: float):
        """Return (r, θ) in polar coordinates, or (None, None) on degeneracy."""
        r = math.hypot(x, y)
        if r < 1e-12:
            return 0.0, 0.0
        return r, math.atan2(y, x)

    # ------------------------------------------------------------------
    # Velocity profile (trapezoidal)
    # ------------------------------------------------------------------

    def _evaluate_duration(
        self,
        path: List[Tuple[float, float, float, bool]],
        v_max: float = 1.0,
        a_max: float = 1.0,
    ) -> List[float]:
        """Assign timestamps to path waypoints using a trapezoidal velocity profile.

        Parameters
        ----------
        path : list of (x, y, θ, singular)
        v_max : maximum linear velocity (m/s)
        a_max : maximum acceleration (m/s²)

        Returns
        -------
        times : list of floats, same length as *path*, cumulative seconds.
        """
        if not path:
            return []

        # Compute segment distances
        dists = [0.0]
        for i in range(1, len(path)):
            dx = path[i][0] - path[i - 1][0]
            dy = path[i][1] - path[i - 1][1]
            dists.append(math.hypot(dx, dy))

        total_dist = sum(dists)
        if total_dist < 1e-9:
            return [0.0] * len(path)

        # Trapezoidal profile phases
        t_acc = v_max / a_max
        d_acc = 0.5 * a_max * t_acc * t_acc

        if 2.0 * d_acc >= total_dist:
            # Triangle profile (never reaches v_max)
            t_acc = math.sqrt(total_dist / a_max)
            d_acc = total_dist / 2.0
            t_const = 0.0
            d_const = 0.0
        else:
            d_const = total_dist - 2.0 * d_acc
            t_const = d_const / v_max

        t_total = 2.0 * t_acc + t_const

        # Map cumulative distance → time
        times = [0.0]
        cum = 0.0
        for i in range(1, len(path)):
            cum += dists[i]
            # Piecewise: accel → const → decel
            if cum <= d_acc:
                t = math.sqrt(2.0 * cum / a_max)
            elif cum <= d_acc + d_const:
                t = t_acc + (cum - d_acc) / v_max
            else:
                remaining = total_dist - cum
                t_dec_from_end = math.sqrt(2.0 * remaining / a_max)
                t = t_total - t_dec_from_end
            times.append(max(t, times[-1]))

        return times

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _build_steering_angles(self) -> None:
        """Pre-compute discrete steering angles for motion primitives."""
        step = self.steer_res * self.max_steer
        if step < 1e-9:
            self._steer_angles = [0.0]
            return
        angles = []
        psi = -self.max_steer
        while psi <= self.max_steer + 1e-9:
            angles.append(psi)
            psi += step
        # Ensure exact endpoints
        if angles[-1] < self.max_steer - 1e-9:
            angles.append(self.max_steer)
        self._steer_angles = angles

    @staticmethod
    def _backtrack(
        nodes: List[KinoNode], idx: int,
    ) -> List[Tuple[float, float, float, bool]]:
        """Reconstruct path from start to node at *idx*."""
        path: List[Tuple[float, float, float, bool]] = []
        while idx >= 0:
            n = nodes[idx]
            path.append((n.x, n.y, n.theta, n.singular))
            idx = n.parent_idx
        path.reverse()
        return path

#!/usr/bin/env python3
"""
MPC Controller Node for Ackermann Car-like Robot.

Ported from SEB-Naver paper. Uses CasADi Opti with SQP solver and warm-starting.
Discrete bicycle model with state [x, y, theta] and control [v, delta].
"""

import math
from typing import List, Optional, Tuple

import casadi as ca
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


class MPCController(Node):
    """ROS2 MPC controller using CasADi for Ackermann steering vehicles."""

    def __init__(self) -> None:
        super().__init__('mpc_controller')

        # Declare parameters
        self.declare_parameter('wheel_base', 0.315)
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('min_speed', -0.5)
        self.declare_parameter('max_steer', 0.785)
        self.declare_parameter('N_horizon', 20)
        self.declare_parameter('dt', 0.05)
        self.declare_parameter('Q', [10.0, 10.0, 0.5])
        self.declare_parameter('R', [10.0, 10.0])
        self.declare_parameter('Rd', [10.0, 10.0])

        # Read parameters
        self.wheelbase: float = self.get_parameter('wheel_base').value
        self.max_speed: float = self.get_parameter('max_speed').value
        self.min_speed: float = self.get_parameter('min_speed').value
        self.max_steer: float = self.get_parameter('max_steer').value
        self.N: int = self.get_parameter('N_horizon').value
        self.dt: float = self.get_parameter('dt').value

        q_vals: List[float] = self.get_parameter('Q').value
        r_vals: List[float] = self.get_parameter('R').value
        rd_vals: List[float] = self.get_parameter('Rd').value

        self.Q = np.diag(q_vals)
        self.R = np.diag(r_vals)
        self.Rd = np.diag(rd_vals)

        # State
        self.current_state: Optional[np.ndarray] = None  # [x, y, theta]
        self.ref_trajectory: Optional[List[Tuple[float, float, float]]] = None

        # Warm-start storage: previous optimal controls [N x 2]
        self.prev_u: Optional[np.ndarray] = None

        # 控制平滑: 上一次输出的转向/速度 (限制变化率防横跳)
        self._last_delta: Optional[float] = None
        self._last_v: Optional[float] = None

        # Build CasADi optimization problem once
        self._build_solver()

        # ROS2 interfaces
        self.create_subscription(Odometry, '/Odometry', self._odom_callback, 10)
        self.create_subscription(Path, '/planner/trajectory', self._trajectory_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Control loop at 20 Hz
        self.timer = self.create_timer(0.05, self._control_loop)

        self.get_logger().info(
            f'MPC Controller initialized: N={self.N}, dt={self.dt}s, '
            f'wheelbase={self.wheelbase}m, v=[{self.min_speed}, {self.max_speed}], '
            f'delta=[{-self.max_steer}, {self.max_steer}]'
        )

    def _build_solver(self) -> None:
        """Construct the CasADi Opti problem symbolically (called once)."""
        opti = ca.Opti()

        N = self.N
        dt = self.dt
        L = self.wheelbase

        # Decision variables: states (N+1) x 3, controls N x 2
        X = opti.variable(N + 1, 3)  # [x, y, theta] per step
        U = opti.variable(N, 2)      # [v, delta] per step

        # Parameters: initial state, reference trajectory, previous control
        x0_param = opti.parameter(3)
        X_ref_param = opti.parameter(N, 3)
        U_ref_param = opti.parameter(N, 2)
        U_prev_param = opti.parameter(N, 2)

        # Cost
        cost = 0.0
        Q = self.Q
        R = self.R
        Rd = self.Rd

        for k in range(N):
            # State tracking error
            e_x = X[k, :] - X_ref_param[k, :]
            cost += ca.mtimes([e_x, Q, e_x.T])

            # Control effort relative to reference
            e_u = U[k, :] - U_ref_param[k, :]
            cost += ca.mtimes([e_u, R, e_u.T])

            # Control rate penalty: delta_u_k = u_k - u_{k-1}
            if k == 0:
                du = U[k, :] - U_prev_param[k, :]
            else:
                du = U[k, :] - U[k - 1, :]
            cost += ca.mtimes([du, Rd, du.T])

        # Terminal cost (state tracking at final step)
        e_x_term = X[N, :] - X_ref_param[N - 1, :]
        cost += ca.mtimes([e_x_term, Q, e_x_term.T])

        opti.minimize(cost)

        # Dynamics constraints (discrete bicycle model)
        for k in range(N):
            x_next = X[k, 0] + dt * U[k, 0] * ca.cos(X[k, 2])
            y_next = X[k, 1] + dt * U[k, 0] * ca.sin(X[k, 2])
            th_next = X[k, 2] + dt * U[k, 0] * ca.tan(U[k, 1]) / L

            opti.subject_to(X[k + 1, 0] == x_next)
            opti.subject_to(X[k + 1, 1] == y_next)
            opti.subject_to(X[k + 1, 2] == th_next)

        # Initial condition constraint
        opti.subject_to(X[0, :] == x0_param.T)

        # Box constraints on controls
        for k in range(N):
            opti.subject_to(opti.bounded(self.min_speed, U[k, 0], self.max_speed))
            opti.subject_to(opti.bounded(-self.max_steer, U[k, 1], self.max_steer))

        # Solver settings
        # CasADi sqpmethod 用 qpsol 参数, 默认 qpoases
        # (ipopt.* 选项只适用于 ipopt solver, 不适用 sqpmethod)
        # max_iter 提高: 短程转弯路径 SQP 默认迭代不足会 Maximum_Iterations_Exceeded
        opts = {
            'qpsol': 'qpoases',
            'print_time': False,
            'max_iter': 100,
        }
        try:
            opti.solver('sqpmethod', opts)
        except Exception as e:
            self.get_logger().warn(f"sqpmethod+qpoases failed ({e}), falling back to default options ...")
            opti.solver('sqpmethod')

        # Store for repeated solving
        self.opti = opti
        self.X_sym = X
        self.U_sym = U
        self.x0_param = x0_param
        self.X_ref_param = X_ref_param
        self.U_ref_param = U_ref_param
        self.U_prev_param = U_prev_param

        self.get_logger().info('CasADi SQP solver built successfully.')

    def _odom_callback(self, msg: Odometry) -> None:
        """Extract current pose from odometry."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # Extract yaw from quaternion
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        theta = math.atan2(2.0 * (qw * qz + qx * qy),
                           1.0 - 2.0 * (qy * qy + qz * qz))
        self.current_state = np.array([x, y, theta])

    def _trajectory_callback(self, msg: Path) -> None:
        """Store reference trajectory as list of (x, y, theta) tuples."""
        points: List[Tuple[float, float, float]] = []
        for pose_stamped in msg.poses:
            px = pose_stamped.pose.position.x
            py = pose_stamped.pose.position.y
            qx = pose_stamped.pose.orientation.x
            qy = pose_stamped.pose.orientation.y
            qz = pose_stamped.pose.orientation.z
            qw = pose_stamped.pose.orientation.w
            theta = math.atan2(2.0 * (qw * qz + qx * qy),
                               1.0 - 2.0 * (qy * qy + qz * qz))
            points.append((px, py, theta))
        self.ref_trajectory = points

    def _find_closest_index(self, state: np.ndarray) -> int:
        """Find index of closest point in reference trajectory to current state."""
        if self.ref_trajectory is None or len(self.ref_trajectory) == 0:
            return 0
        min_dist = float('inf')
        closest_idx = 0
        sx, sy = state[0], state[1]
        for i, (rx, ry, _) in enumerate(self.ref_trajectory):
            d = (sx - rx) ** 2 + (sy - ry) ** 2
            if d < min_dist:
                min_dist = d
                closest_idx = i
        return closest_idx

    def _extract_reference(self, state: np.ndarray) -> np.ndarray:
        """Extract N reference points ahead of the robot from the trajectory.

        Returns array of shape (N, 3) with [x, y, theta] per row.
        If insufficient points remain, extrapolate with last known heading.
        """
        N = self.N
        ref = np.zeros((N, 3))

        if self.ref_trajectory is None or len(self.ref_trajectory) == 0:
            # No trajectory: use current position as reference everywhere
            ref[:, 0] = state[0]
            ref[:, 1] = state[1]
            ref[:, 2] = state[2]
            return ref

        start_idx = self._find_closest_index(state)
        traj_len = len(self.ref_trajectory)

        for k in range(N):
            idx = start_idx + k
            if idx < traj_len:
                ref[k, 0] = self.ref_trajectory[idx][0]
                ref[k, 1] = self.ref_trajectory[idx][1]
                ref[k, 2] = self.ref_trajectory[idx][2]
            else:
                # Extrapolate beyond end of trajectory
                last_x, last_y, last_th = self.ref_trajectory[-1]
                ref[k, 0] = last_x + (idx - traj_len + 1) * self.dt * 0.5 * math.cos(last_th)
                ref[k, 1] = last_y + (idx - traj_len + 1) * self.dt * 0.5 * math.sin(last_th)
                ref[k, 2] = last_th

        return ref

    def _solve_mpc(self, state: np.ndarray, X_ref: np.ndarray) -> Optional[np.ndarray]:
        """Solve the MPC optimization problem.

        Args:
            state: Current state [x, y, theta].
            X_ref: Reference trajectory (N, 3).

        Returns:
            Optimal control sequence (N, 2) or None on failure.
        """
        opti = self.opti

        # Set parameter values
        opti.set_value(self.x0_param, state)
        opti.set_value(self.X_ref_param, X_ref)
        opti.set_value(self.U_ref_param, np.zeros((self.N, 2)))

        # Previous control for rate penalty
        if self.prev_u is not None:
            opti.set_value(self.U_prev_param, self.prev_u)
        else:
            opti.set_value(self.U_prev_param, np.zeros((self.N, 2)))

        # Warm-start from shifted previous solution
        if self.prev_u is not None:
            # Shift controls: drop first, repeat last
            u_warm = np.vstack([self.prev_u[1:, :], self.prev_u[-1:, :]])
            try:
                opti.set_initial(self.U_sym, u_warm)
                # Also warm-start states by forward simulation
                x_warm = np.zeros((self.N + 1, 3))
                x_warm[0, :] = state
                for k in range(self.N):
                    v_k, d_k = u_warm[k, 0], u_warm[k, 1]
                    x_warm[k + 1, 0] = x_warm[k, 0] + self.dt * v_k * math.cos(x_warm[k, 2])
                    x_warm[k + 1, 1] = x_warm[k, 1] + self.dt * v_k * math.sin(x_warm[k, 2])
                    x_warm[k + 1, 2] = x_warm[k, 2] + self.dt * v_k * math.tan(d_k) / self.wheelbase
                opti.set_initial(self.X_sym, x_warm)
            except Exception:
                pass  # Fall back to default initialization

        try:
            sol = opti.solve()
            u_opt = sol.value(self.U_sym)
            return np.array(u_opt).reshape(self.N, 2)
        except Exception as e:
            self.get_logger().warn(f'MPC solver failed: {e}')
            return None

    def _publish_zero_velocity(self) -> None:
        """Publish zero velocity command."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def _control_loop(self) -> None:
        """Main control callback at 20 Hz."""
        # Check if we have current state
        if self.current_state is None:
            return  # Wait for odometry

        # Check if we have a reference trajectory
        if self.ref_trajectory is None or len(self.ref_trajectory) == 0:
            self._publish_zero_velocity()
            return

        # Extract reference points
        X_ref = self._extract_reference(self.current_state)

        # Solve MPC
        u_opt = self._solve_mpc(self.current_state, X_ref)

        if u_opt is None:
            # MPC 求解失败: 沿用上一次控制并以 50% 速度衰减平滑延续,
            # 避免突然停车 (短程转弯 SQP 偶尔 Maximum_Iterations_Exceeded)
            if self.prev_u is not None:
                u_prev = self.prev_u[0, :] * 0.5
                v_cmd = float(u_prev[0])
                delta_cmd = float(u_prev[1])
                if self._last_delta is not None:
                    delta_cmd = float(np.clip(delta_cmd, self._last_delta - 0.15, self._last_delta + 0.15))
                self._last_delta = delta_cmd
                if self._last_v is not None:
                    v_cmd = float(np.clip(v_cmd, self._last_v - 0.15, self._last_v + 0.15))
                self._last_v = v_cmd
                omega_cmd = v_cmd * math.tan(delta_cmd) / self.wheelbase
                twist = Twist()
                twist.linear.x = v_cmd
                twist.angular.z = omega_cmd
                self.cmd_pub.publish(twist)
                return
            self._publish_zero_velocity()
            return

        # Apply first control input
        v_cmd = float(u_opt[0, 0])
        delta_cmd = float(u_opt[0, 1])

        # --- 控制平滑: 限制转向角变化率, 防止路径重规划导致 ±δmax 横跳 ---
        # 每次更新 (20Hz) 转向变化 ≤ 0.15 rad → 最大转向速率 3 rad/s
        # 否则极限转向 (±1.59 rad/s ω) 交替会让小车物理甩动, IMU 冲击炸 SLAM
        if self._last_delta is not None:
            d_delta_max = 0.15
            delta_cmd = float(np.clip(
                delta_cmd, self._last_delta - d_delta_max, self._last_delta + d_delta_max))
        self._last_delta = delta_cmd

        # 限制速度变化率: 每次 ±0.15 m/s (20Hz → 加速度 ≤ 3 m/s²)
        if self._last_v is not None:
            d_v_max = 0.15
            v_cmd = float(np.clip(v_cmd, self._last_v - d_v_max, self._last_v + d_v_max))
        self._last_v = v_cmd

        # Compute angular velocity: omega = v * tan(delta) / wheelbase
        omega_cmd = v_cmd * math.tan(delta_cmd) / self.wheelbase

        # Publish command
        twist = Twist()
        twist.linear.x = v_cmd
        twist.angular.z = omega_cmd
        self.cmd_pub.publish(twist)

        # Store solution for warm-starting next iteration
        self.prev_u = u_opt.copy()


def main(args=None) -> None:
    """Entry point for the MPC controller node."""
    rclpy.init(args=args)
    node = MPCController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

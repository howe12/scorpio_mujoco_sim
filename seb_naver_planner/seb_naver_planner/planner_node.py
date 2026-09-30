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
import time
from enum import Enum, auto
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String, Header, Float32MultiArray, MultiArrayDimension
from se2_grid_msgs.msg import SE2Grid

from seb_naver_planner.kino_astar import KinoAStarPlanner, SearchStatus


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
# PlanManager
# ---------------------------------------------------------------------------
class PlanManager:
    """Manages the planning pipeline: A* search → (future: PHR-ALM) → path output."""

    def __init__(self, node: Node, sdf_parser: SDFGridParser) -> None:
        self._node = node
        self._sdf = sdf_parser

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
        status, path = self._kino_astar.search(
            start_state=(start_x, start_y, start_yaw),
            goal_state=(goal_x, goal_y, goal_yaw),
            sdf_grid_data=self._sdf.sdf_map,
            sdf_info=sdf_info,
        )

        if status != SearchStatus.SUCCESS or not path:
            self._node.get_logger().error(f"PlanManager: A* search failed (status={status.name})")
            return None

        # Convert path format: list of (x, y, theta, singular) → list of (x, y, yaw)
        waypoints = [(p[0], p[1], p[2]) for p in path]

        # Step 4: Skip PHR-ALM optimization for now — use rough path directly
        # TODO: Implement PHR-ALM trajectory optimization
        return waypoints


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

        # Plan manager
        self._plan_manager = PlanManager(self, self._sdf_parser)

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

        # Publish rough path (same as optimized for now)
        self._publish_path(path, self._rough_path_pub, "map")

        # Publish trajectory for MPC (currently identical to rough path)
        self._publish_path(path, self._traj_pub, "map")

        self.get_logger().info(f"Planning pipeline succeeded: {len(path)} waypoints")
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

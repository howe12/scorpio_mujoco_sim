#!/usr/bin/env python3
"""
SEB-Naver Simulation Runner

Run car-like robot navigation on paper-replicated terrains:
  - Mountain, Forest, Snowy Mountain, Pump Track

Usage:
    python3 run_seb_naver_sim.py                    # Interactive viewer, mountain
    python3 run_seb_naver_sim.py --terrain forest   # Forest scene
    python3 run_seb_naver_sim.py --terrain all      # Cycle through all terrains
    python3 run_seb_naver_sim.py --headless         # Headless benchmark
"""

import mujoco
import mujoco.viewer
import numpy as np
import argparse
import os
import time


TERRAINS = ['mountain', 'forest', 'snowy_mountain', 'pump_track', 'plaza']

# Paper parameters (Table I / Section VII-B)
DELTA_MAX = 0.785    # max steering angle (rad)
V_MAX = 1.0          # max longitudinal velocity (m/s)
A_MAX_LON = 5.0      # max longitudinal acceleration (m/s²)
A_MAX_LAT = 10.0     # max lateral acceleration (m/s²)
PHI_MAX = 0.52       # max pitch/roll (rad, ~30°)
WHEELBASE = 0.5      # m



def load_world(terrain_name):
    """Load a terrain world file and inject heightfield data if needed."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    base_dir = os.path.dirname(script_dir)
    world_path = os.path.join(base_dir, 'worlds', f'{terrain_name}.xml')
    
    if not os.path.exists(world_path):
        print(f"World file not found. Generating terrains...")
        import sys
        sys.path.insert(0, script_dir)
        from generate_terrain import generate_all_terrains
        generate_all_terrains(base_dir)
    
    model = mujoco.MjModel.from_xml_path(world_path)
    data = mujoco.MjData(model)
    
    # Inject heightfield data from binary file if hfield exists
    if model.nhfield > 0:
        hfield_path = os.path.join(base_dir, 'models', 'terrains', f'{terrain_name}_hfield.bin')
        if os.path.exists(hfield_path):
            hdata = np.fromfile(hfield_path, dtype=np.float32)
            expected = model.hfield_nrow[0] * model.hfield_ncol[0]
            if len(hdata) == expected:
                model.hfield_data[:] = hdata
                print(f"  Loaded heightfield: {model.hfield_nrow[0]}x{model.hfield_ncol[0]}")
            else:
                print(f"  WARNING: hfield size mismatch {len(hdata)} vs {expected}")
        else:
            print(f"  WARNING: hfield file not found: {hfield_path}")
    
    return model, data


def reset_robot_facing_goal(model, data):
    """Reset robot orientation to face the goal."""
    goal_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'goal_site')
    goal = data.site_xpos[goal_id]
    pos = data.qpos[:3]
    
    dx = goal[0] - pos[0]
    dy = goal[1] - pos[1]
    yaw = np.arctan2(dy, dx)
    
    # Set quaternion for this yaw (w, x, y, z)
    data.qpos[3] = np.cos(yaw / 2)  # w
    data.qpos[4] = 0                  # x
    data.qpos[5] = 0                  # y
    data.qpos[6] = np.sin(yaw / 2)  # z
    
    mujoco.mj_forward(model, data)


def snap_to_ground(model, data):
    """把机器人垂直投影到地面 (raycast 找地形表面), 避免初始悬空导致:
    - 自由落体 (IMU 初始化被零加速度污染 → FAST-LIO2 重力估计错误)
    - 落地冲击 (IMU 噪声巨大)
    只命中地形/hfield geom (排除 start_marker 等小标记物).
    """
    x, y = data.qpos[0], data.qpos[1]
    # 找出所有地形类 geom (hfield 或名字含 terrain)
    terrain_gids = []
    for gi in range(model.ngeom):
        gname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gi) or ''
        if model.geom_type[gi] == mujoco.mjtGeom.mjGEOM_HFIELD or 'terrain' in gname.lower():
            terrain_gids.append(gi)
    if not terrain_gids:
        return  # 平地无 hfield, 无需处理

    best_dist = -1.0
    best_z = None
    pnt = np.zeros(3); vec = np.zeros(3)
    geomgroup = np.ones(6, dtype=np.uint8)
    gid = np.array([-1], dtype=np.int32)
    for gi in terrain_gids:
        # 从 (x,y,100) 向下射向该 geom
        pnt[:] = [x, y, 100.0]
        vec[:] = [0.0, 0.0, -1.0]
        dist = mujoco.mj_ray(model, data, pnt, vec, geomgroup, 0, -1, gid, None)
        if dist >= 0 and (best_dist < 0 or dist < best_dist):
            best_dist = dist
            best_z = 100.0 - dist
    if best_z is not None:
        data.qpos[2] = best_z
        mujoco.mj_forward(model, data)
        print(f"  [snap_to_ground] 机器人投影到 (x={x:.2f},y={y:.2f}) 表面 z={best_z:.3f}")


def simple_controller(model, data, goal_pos=None):
    """Proportional controller with terrain-aware speed adjustment."""
    # Get current pose directly from qpos (freejoint: x,y,z,w,x,y,z)
    pos = data.qpos[:3]
    quat = data.qpos[3:7]  # (w, x, y, z)
    
    if goal_pos is None:
        goal_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'goal_site')
        goal_pos = data.site_xpos[goal_id]
    
    dx = goal_pos[0] - pos[0]
    dy = goal_pos[1] - pos[1]
    dist = np.sqrt(dx*dx + dy*dy)
    
    # Current yaw from quaternion (w, x, y, z)
    w, x, y, z = quat
    yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    
    desired_yaw = np.arctan2(dy, dx)
    yaw_error = desired_yaw - yaw
    while yaw_error > np.pi: yaw_error -= 2*np.pi
    while yaw_error < -np.pi: yaw_error += 2*np.pi
    
    # Steering: proportional with higher gain
    steer = np.clip(yaw_error * 3.0, -DELTA_MAX, DELTA_MAX)
    
    # Base speed factor
    speed_factor = 1.0
    
    # Simple obstacle avoidance via LiDAR ray casting
    lidar_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'lidar_site')
    if lidar_site_id >= 0:
        lidar_pos = data.site_xpos[lidar_site_id]
        lidar_mat = data.site_xmat[lidar_site_id].reshape(3, 3)
        
        # Check 5 forward directions
        check_angles = [-0.4, -0.2, 0.0, 0.2, 0.4]
        min_front_dist = 999.0
        blocked_side = 0  # -1=left blocked, +1=right blocked
        
        for angle in check_angles:
            direction_local = np.array([np.cos(angle), np.sin(angle), 0.0])
            direction_world = lidar_mat @ direction_local
            geom_id = np.array([-1], dtype=np.int32)
            ray_dist = mujoco.mj_ray(model, data, lidar_pos, direction_world,
                                      None, 1, -1, geom_id)
            if 0 <= ray_dist < min_front_dist:
                min_front_dist = ray_dist
                if angle < -0.1:
                    blocked_side = 1   # right blocked → steer left
                elif angle > 0.1:
                    blocked_side = -1  # left blocked → steer right
        
        # Avoidance steering override
        if min_front_dist < 2.0:
            avoid_steer = blocked_side * 0.6
            if abs(yaw_error) < 0.5:
                steer = avoid_steer
            else:
                steer = np.clip(steer + avoid_steer * 0.5, -DELTA_MAX, DELTA_MAX)
            
            # Slow down near obstacles
            speed_factor *= min(1.0, min_front_dist / 2.0)
    
    # 目标车速 (m/s): 转弯减速 + 接近目标减速
    speed_factor = max(0.1, speed_factor * (1.0 - abs(yaw_error) * 0.6 / np.pi))
    if dist < 3.0:
        speed_factor *= max(0.1, dist / 3.0)
    v_cmd = np.clip(speed_factor * 0.4, 0.0, 0.5)     # m/s (限 0.5, 稳)
    omega = v_cmd / 0.0525                             # rad/s
    throttle = omega
    
    data.ctrl[0] = throttle
    data.ctrl[1] = throttle
    data.ctrl[2] = steer
    
    return dist, yaw_error


class KeyboardController:
    """Keyboard teleop: hold = active, release = return to neutral (gamepad style).

    ctrl[0], ctrl[1] 单位 = rad/s (后轮角速度, velocity actuator)
    ctrl[2] 单位 = rad (转向角, position actuator)

    操控方式:
      W/S 持续按住 → 持续加速/减速 (松开后速度保持, 按空格停车)
      A/D 持续按住 → 持续转向 (松开自动回正!)
      Q/E/Z/C 按住 → 组合动作 (松开转向回正, 速度保持)
    """

    WHEEL_R = 0.0525
    WHEELBASE = 0.315
    MAX_YAW_RATE = 0.9

    def __init__(self, model):
        self.model = model
        self.throttle = 0.0        # m/s (目标车速)
        self.steer = 0.0           # rad
        self.max_throttle = 0.5    # m/s
        self.max_steer = DELTA_MAX # 0.785 rad = 45°
        self.throttle_accel = 0.6  # m/s² (按住时加速率)
        self.steer_rate = 3.0      # rad/s (按住时转向速率)
        self.steer_return = 3.5    # rad/s (松开时回正速率)
        self.effective_throttle = 0.0
        self.mode = "manual"
        self._quit = False
        self._listener = None
        self._held = set()         # 当前按住的键

    def start_listener(self):
        """启动全局键盘监听 (pynput). 若不可用则降级为无键盘模式."""
        try:
            from pynput import keyboard as kb_mod
        except ImportError:
            print("\n  ⚠ 未安装 pynput, 键盘控制不可用")
            print("    安装: /usr/bin/python3 -m pip install --user pynput")
            print("    或使用 Tab 切换到自动导航模式\n")
            self._listener = None
            return

        def on_press(key):
            try:
                k = key.char
            except AttributeError:
                k = None
            self._held.add(k if k else key)

            # 单次触发 (Tab / R / Space / ESC)
            if k in ('r', 'R'):
                self.steer = 0.0
                self.throttle = 0.0
                self._held.discard(k)
            elif key == kb_mod.Key.space:
                self.throttle = 0.0
                self._held.discard(key)
            elif key == kb_mod.Key.tab:
                self.mode = "auto" if self.mode == "manual" else "manual"
                tag = "AUTO (nav to goal)" if self.mode == "auto" else "MANUAL (keyboard)"
                print(f"\n  >>> Mode: {tag}", flush=True)
                self._held.discard(key)
            elif key == kb_mod.Key.esc:
                self._quit = True

        def on_release(key):
            try:
                k = key.char
            except AttributeError:
                k = None
            self._held.discard(k if k else key)

        self._listener = kb_mod.Listener(on_press=on_press, on_release=on_release)
        self._listener.daemon = True
        self._listener.start()

    def _update_controls(self, dt):
        """每帧根据当前按住的键平滑更新 throttle 和 steer."""
        held = self._held

        # --- 转向: 按住 A/D/Q/E/Z/C 时持续转向, 松开自动回正 ---
        steer_cmd = 0.0
        if any(k in held for k in ('a', 'A', 'q', 'Q', 'z', 'Z')):
            steer_cmd += 1.0
        if any(k in held for k in ('d', 'D', 'e', 'E', 'c', 'C')):
            steer_cmd -= 1.0

        if abs(steer_cmd) > 0:
            target = steer_cmd * self.max_steer
            rate = self.steer_rate
        else:
            target = 0.0
            rate = self.steer_return  # 松开时自动回正

        err = target - self.steer
        step = np.clip(err, -rate * dt, rate * dt)
        self.steer = np.clip(self.steer + step, -self.max_steer, self.max_steer)

        # --- 油门: 按住 W/S/Q/E/Z/C 时加速/减速, 松开保持 ---
        accel = 0.0
        if any(k in held for k in ('w', 'W', 'q', 'Q', 'e', 'E')):
            accel += self.throttle_accel
        if any(k in held for k in ('s', 'S', 'z', 'Z', 'c', 'C')):
            accel -= self.throttle_accel
        self.throttle = np.clip(self.throttle + accel * dt,
                                -self.max_throttle, self.max_throttle)

    def apply(self, data, dt=None):
        """每帧调用: 更新按键状态 → 计算控制 → 写入执行器."""
        if dt is None:
            dt = self.model.opt.timestep
        self._update_controls(dt)

        v_cmd = self.throttle
        # 转弯限速
        if abs(self.steer) > 1e-3:
            R_turn = self.WHEELBASE / np.tan(abs(self.steer))
            v_limit = self.MAX_YAW_RATE * R_turn
            if abs(v_cmd) > v_limit:
                v_cmd = np.sign(v_cmd) * v_limit

        self.effective_throttle = v_cmd
        omega = v_cmd / self.WHEEL_R
        data.ctrl[0] = omega
        data.ctrl[1] = omega
        data.ctrl[2] = self.steer


def run_interactive(terrain_name, enable_ros2=False, publish_images=False, headless=False, no_tf_odom=False):
    """Run with keyboard control (default) or auto-navigation.

    Args:
        terrain_name: 场景名称
        enable_ros2: 是否发布 ROS2 话题 (供 SLAM/导航使用)
        publish_images: 是否同时发布相机图像 (较慢)
        headless: 无 GUI 模式 (需外部 cmd_vel 驱动)
    """
    model, data = load_world(terrain_name)
    mujoco.mj_forward(model, data)
    reset_robot_facing_goal(model, data)
    snap_to_ground(model, data)

    kb = None
    viewer = None
    if not headless:
        kb = KeyboardController(model)
        kb.start_listener()

        print(f"\n{'='*60}")
        print(f"  Scorpio Simulation: {terrain_name.upper()}")
        print(f"{'='*60}")
        print(f"  Keyboard Controls (hold = active, release = auto-center):")
        print(f"    W / ↑     Accelerate  (hold)")
        print(f"    S / ↓     Reverse     (hold)")
        print(f"    A / ←     Steer Left  (hold, release auto-centers)")
        print(f"    D / →     Steer Right (hold, release auto-centers)")
        print(f"    Q         Forward + Left  (hold)")
        print(f"    E         Forward + Right (hold)")
        print(f"    Z         Reverse + Left  (hold)")
        print(f"    C         Reverse + Right (hold)")
        print(f"    Space     Brake (speed=0)")
        print(f"    R         Reset (speed=0, steering=0)")
        print(f"    Tab       Toggle Manual ↔ Auto-nav")
        print(f"    ESC       Quit")
        print(f"  Viewer: Drag=rotate, Scroll=zoom")
        print(f"{'='*60}\n")

        viewer = mujoco.viewer.launch_passive(model, data)
    else:
        print(f"\n  [HEADLESS] Scorpio Simulation: {terrain_name.upper()}")
        print(f"  Waiting for /cmd_vel commands...")

    # 物理稳定期: 让不平地形上悬空的机器人落到地面并静止.
    # forest/mountain 等 hfield 场景初始 qpos[2] 可能悬空 (轮底未贴地),
    # 若直接发布 IMU, FAST-LIO2 会在自由落体 + 落地冲击的瞬态上初始化,
    # 重力方向估计错误 → 定位发散. 这里先 step 到完全静止再开 ROS2.
    if model.nhfield > 0:
        settle_steps = int(2.0 / model.opt.timestep)  # 2 s 物理时间
        for _ in range(settle_steps):
            mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
        print(f"  [settle] 物理稳定 {settle_steps*model.opt.timestep:.1f}s, "
              f"机器人 z={data.qpos[2]:.3f}, v={np.linalg.norm(data.qvel[:2]):.3f}")

    # ROS2 发布器 (可选)
    ros2_pub = None
    if enable_ros2:
        try:
            from scorpio_ros2_publisher import ScorpioROS2Publisher
            ros2_pub = ScorpioROS2Publisher(model, data, publish_images=publish_images,
                                            publish_odom_tf=not no_tf_odom)
        except ImportError as exc:
            print(f"\n  ⚠ 无法启动 ROS2 发布器: {exc}")
            print(f"    请先 source ROS2 环境: source /opt/ros/humble/setup.bash")
        except Exception as exc:
            print(f"\n  ⚠ ROS2 发布器初始化失败: {exc}")

    # 所有 ROS2 模式都订阅 cmd_vel (供 MPC/导航驱动), 无论 GUI 还是 headless
    cmd_vel = [0.0, 0.0]  # [linear.x, angular.z]
    if enable_ros2 and ros2_pub is not None:
        from geometry_msgs.msg import Twist
        def cmd_vel_cb(msg):
            cmd_vel[0] = msg.linear.x
            cmd_vel[1] = msg.angular.z
        ros2_pub.node.create_subscription(Twist, '/cmd_vel', cmd_vel_cb, 10)

    running = True
    while running:
        if headless:
            # 用 cmd_vel 驱动 (Ackermann 转换)
            v_cmd = cmd_vel[0]
            omega_cmd = cmd_vel[1]
            WHEEL_R = 0.0525
            WHEELBASE = 0.315
            if abs(v_cmd) > 0.001:
                steer_angle = np.arctan(omega_cmd * WHEELBASE / max(abs(v_cmd), 0.01))
                steer_angle = np.clip(steer_angle, -0.785, 0.785)
            else:
                steer_angle = 0.0
            wheel_speed = v_cmd / WHEEL_R
            data.ctrl[0] = wheel_speed
            data.ctrl[1] = wheel_speed
            data.ctrl[2] = steer_angle

            mujoco.mj_step(model, data)

            # 发布 ROS2
            if ros2_pub is not None:
                ros2_pub.publish()

            # 每 2 秒打印状态
            if int(data.time * 0.5) != int((data.time - model.opt.timestep) * 0.5):
                pos = data.qpos[:3]
                v_act = float(np.linalg.norm(data.qvel[:2]))
                print(f"\r  [HEADLESS] t={data.time:6.1f}s v={v_act:.2f}m/s "
                      f"pos=({pos[0]:.2f},{pos[1]:.2f}) cmd=({cmd_vel[0]:.2f},{cmd_vel[1]:.2f})   ",
                      end='', flush=True)
        else:
            if not viewer.is_running() or (kb and kb._quit):
                running = False
                continue

            # MPC/导航指令优先: 有非零 cmd_vel 时用它驱动, 否则用键盘
            if abs(cmd_vel[0]) > 0.001 or abs(cmd_vel[1]) > 0.001:
                v_cmd = cmd_vel[0]
                omega_cmd = cmd_vel[1]
                WHEEL_R = 0.0525
                WHEELBASE = 0.315
                if abs(v_cmd) > 0.001:
                    steer_angle = np.arctan(omega_cmd * WHEELBASE / max(abs(v_cmd), 0.01))
                    steer_angle = np.clip(steer_angle, -0.785, 0.785)
                else:
                    steer_angle = 0.0
                wheel_speed = v_cmd / WHEEL_R
                data.ctrl[0] = wheel_speed
                data.ctrl[1] = wheel_speed
                data.ctrl[2] = steer_angle
            elif kb.mode == "manual":
                kb.apply(data)
            else:
                simple_controller(model, data)

            mujoco.mj_step(model, data)
            viewer.sync()

            if ros2_pub is not None:
                ros2_pub.publish()

            if int(data.time * 2) != int((data.time - model.opt.timestep) * 2):
                pos = data.qpos[:3]
                q = data.qpos[3:7]
                yaw = np.degrees(np.arctan2(
                    2*(q[0]*q[3]+q[1]*q[2]), 1-2*(q[2]**2+q[3]**2)))
                goal_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'goal_site')
                if goal_id >= 0:
                    g = data.site_xpos[goal_id]
                    dist = np.sqrt((g[0]-pos[0])**2 + (g[1]-pos[1])**2)
                else:
                    dist = -1
                mode_tag = "MANUAL" if kb.mode == "manual" else "AUTO"
                v_act = float(np.linalg.norm(data.qvel[:2]))
                v_show = kb.effective_throttle
                print(f"\r  [{mode_tag}] t={data.time:5.1f}s | "
                      f"v={v_act:4.2f}m/s(令{v_show:+.2f}) str={np.degrees(kb.steer):+5.1f}° | "
                      f"pos=({pos[0]:.2f},{pos[1]:.2f}) yaw={yaw:+.0f}° | "
                      f"goal={dist:.1f}m   ", end='', flush=True)

    if viewer is not None:
        viewer.close()

    if ros2_pub is not None:
        ros2_pub.shutdown()

    import os as _os
    if kb is not None and kb._listener is not None:
        kb._listener.stop()
    _os._exit(0)


def run_headless_benchmark():
    """Run headless benchmark on all terrains."""
    print(f"\n{'='*60}")
    print(f"  SEB-Naver Headless Benchmark")
    print(f"{'='*60}\n")
    
    results = {}
    for terrain in TERRAINS:
        model, data = load_world(terrain)
        mujoco.mj_forward(model, data)  # Initialize positions
        reset_robot_facing_goal(model, data)
        
        start_time = time.time()
        reached = False
        max_time = 120.0  # 2 min timeout
        
        while data.time < max_time:
            dist, _ = simple_controller(model, data)
            mujoco.mj_step(model, data)
            
            if dist < 0.5:
                reached = True
                break
        
        wall_time = time.time() - start_time
        final_pos = data.qpos[:3]
        
        status = "REACHED" if reached else "TIMEOUT"
        results[terrain] = {
            'status': status,
            'time': data.time if reached else max_time,
            'wall_time': wall_time,
            'final_pos': final_pos.copy(),
            'steps': int(data.time / model.opt.timestep),
        }
        
        print(f"  [{status:7s}] {terrain:20s}: "
              f"sim_time={data.time:6.1f}s, "
              f"wall={wall_time:.1f}s, "
              f"speed={data.time/wall_time:.1f}x RT")
    
    print(f"\n{'='*60}")
    print(f"  Benchmark complete!")
    print(f"{'='*60}")
    return results


def main():
    parser = argparse.ArgumentParser(description='SEB-Naver MuJoCo Simulation')
    parser.add_argument('--terrain', type=str, default='mountain',
                        choices=TERRAINS + ['all'],
                        help='Terrain scenario to simulate')
    parser.add_argument('--headless', action='store_true',
                        help='Run headless benchmark on all terrains')
    parser.add_argument('--headless-ros2', action='store_true',
                        help='Headless mode with ROS2 (no GUI, subscribe /cmd_vel)')
    parser.add_argument('--ros2', action='store_true',
                        help='Enable ROS2 publishing (/scan, /odom, /tf, /imu)')
    parser.add_argument('--ros2-images', action='store_true',
                        help='Also publish camera images (slow)')
    parser.add_argument('--no-tf-odom', action='store_true',
                        help='不发布 odom→base_footprint TF (与 FAST-LIO2 联用时避免 TF 冲突)')
    args = parser.parse_args()
    
    if args.headless_ros2:
        run_interactive(args.terrain, enable_ros2=True, headless=True, no_tf_odom=args.no_tf_odom)
    elif args.headless:
        run_headless_benchmark()
    elif args.terrain == 'all':
        for t in TERRAINS:
            run_interactive(t, args.ros2, args.ros2_images)
    else:
        run_interactive(args.terrain, args.ros2, args.ros2_images)


if __name__ == '__main__':
    main()
    # Avoid GLFW/GLX cleanup segfault on NVIDIA drivers
    import os; os._exit(0)

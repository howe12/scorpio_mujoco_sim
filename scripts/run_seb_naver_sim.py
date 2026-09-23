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
    v_cmd = np.clip(speed_factor * 0.6, 0.0, 1.0)     # m/s
    omega = v_cmd / 0.0525                             # rad/s
    throttle = omega
    
    data.ctrl[0] = throttle
    data.ctrl[1] = throttle
    data.ctrl[2] = steer
    
    return dist, yaw_error


class KeyboardController:
    """Keyboard teleop using pynput global listener.
    
    ctrl[0], ctrl[1] 单位 = Nm (后轮力矩, torque motor)
    ctrl[2] 单位 = rad (转向角, position actuator)
    
    Scorpio: ~5kg, r=0.0525m, vmax=0.26m/s
    ctrl=0.5 Nm → v≈0.22 m/s (接近 vmax)
    ctrl=0.8 Nm → v≈0.33 m/s (超速，留余量)
    """

    # 轮半径: 车速 v = omega * WHEEL_R,  omega = v / WHEEL_R
    WHEEL_R = 0.0525

    def __init__(self, model):
        self.model = model
        self.throttle = 0.0      # m/s (目标车速)
        self.steer = 0.0         # rad
        self.max_throttle = 1.0   # m/s (模型上限 1.575 m/s)
        self.max_steer = DELTA_MAX
        self.throttle_step = 0.1  # m/s 每按一次 (10次到 1.0 m/s)
        self.steer_step = 0.08    # rad 每按一次 (~4.6°)
        self.mode = "manual"
        self._quit = False
        self._listener = None

    def start_listener(self):
        """Start global keyboard listener in background thread."""
        from pynput import keyboard as kb_mod

        def on_press(key):
            try:
                k = key.char
            except AttributeError:
                k = None

            if k in ('w', 'W'):
                self.throttle = min(self.throttle + self.throttle_step, self.max_throttle)
            elif k in ('s', 'S'):
                self.throttle = max(self.throttle - self.throttle_step, -self.max_throttle)
            elif k in ('a', 'A'):
                self.steer = min(self.steer + self.steer_step, self.max_steer)
            elif k in ('d', 'D'):
                self.steer = max(self.steer - self.steer_step, -self.max_steer)
            elif k in ('r', 'R'):
                self.steer = 0.0
                self.throttle = 0.0
            elif key == kb_mod.Key.space:
                self.throttle = 0.0
            elif key == kb_mod.Key.tab:
                self.mode = "auto" if self.mode == "manual" else "manual"
                mode_str = "AUTO (nav to goal)" if self.mode == "auto" else "MANUAL (keyboard)"
                print(f"\n  >>> Mode: {mode_str}", flush=True)
            elif key == kb_mod.Key.esc:
                self._quit = True

        self._listener = kb_mod.Listener(on_press=on_press)
        self._listener.daemon = True
        self._listener.start()

    def apply(self, data):
        """将目标车速 (m/s) 转为轮角速度 (rad/s) 写入执行器"""
        omega = self.throttle / self.WHEEL_R
        data.ctrl[0] = omega
        data.ctrl[1] = omega
        data.ctrl[2] = self.steer


def run_interactive(terrain_name):
    """Run with keyboard control (default) or auto-navigation."""
    model, data = load_world(terrain_name)
    mujoco.mj_forward(model, data)
    reset_robot_facing_goal(model, data)

    kb = KeyboardController(model)
    kb.start_listener()  # Global keyboard listener (bypasses window manager)

    print(f"\n{'='*60}")
    print(f"  Scorpio Simulation: {terrain_name.upper()}")
    print(f"{'='*60}")
    print(f"  Keyboard Controls (any window, no focus required):")
    print(f"    W / ↑     Accelerate")
    print(f"    S / ↓     Reverse / Brake")
    print(f"    A / ←     Steer Left")
    print(f"    D / →     Steer Right")
    print(f"    Space     Stop")
    print(f"    R         Center Steering + Stop")
    print(f"    Tab       Toggle Manual ↔ Auto-nav")
    print(f"    ESC       Quit")
    print(f"  Viewer: Drag=rotate, Scroll=zoom")
    print(f"{'='*60}\n")

    viewer = mujoco.viewer.launch_passive(model, data)

    while viewer.is_running() and not kb._quit:
        if kb.mode == "manual":
            kb.apply(data)
        else:
            simple_controller(model, data)

        mujoco.mj_step(model, data)
        viewer.sync()

        # Print status every 0.5s
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
            print(f"\r  [{mode_tag}] t={data.time:5.1f}s | "
                  f"v_cmd={kb.throttle:+.2f}m/s str={np.degrees(kb.steer):+5.1f}° | "
                  f"pos=({pos[0]:.2f},{pos[1]:.2f}) yaw={yaw:+.0f}° | "
                  f"goal={dist:.1f}m   ", end='', flush=True)

    viewer.close()


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
    args = parser.parse_args()
    
    if args.headless:
        run_headless_benchmark()
    elif args.terrain == 'all':
        for t in TERRAINS:
            run_interactive(t)
    else:
        run_interactive(args.terrain)


if __name__ == '__main__':
    main()
    # Avoid GLFW/GLX cleanup segfault on NVIDIA drivers
    import os; os._exit(0)

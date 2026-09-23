#!/usr/bin/env python3
"""
MuJoCo Simulation Web Viewer

Renders MuJoCo frames via EGL and streams them as MJPEG over HTTP.
Works on headless servers or when GLFW/GLX is broken.

Usage:
    python3 view_sim_web.py                          # Default: car-like on mountain
    python3 view_sim_web.py --terrain pump_track     # Specific terrain
    python3 view_sim_web.py --port 8080              # Custom port
    
Then open http://localhost:8080 in your browser.
"""

import mujoco
import numpy as np
import argparse
import os
import io
import time
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler

# Force EGL backend before importing mujoco
os.environ['MUJOCO_GL'] = 'egl'


class SimState:
    """Shared simulation state between physics and rendering threads."""
    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.lock = threading.Lock()
        self.running = True
        self.latest_frame = None
        self.frame_ready = threading.Event()


def physics_loop(state, controller_fn=None, rate_hz=500):
    """Run physics simulation in background thread."""
    dt = 1.0 / rate_hz
    while state.running:
        with state.lock:
            if controller_fn:
                controller_fn(state.model, state.data)
            mujoco.mj_step(state.model, state.data)
        time.sleep(dt * 0.9)  # Slight undersleep to maintain rate


def render_loop(state, width=960, height=720, fps=30):
    """Render frames at target FPS using EGL."""
    renderer = mujoco.Renderer(state.model, height, width)
    frame_interval = 1.0 / fps
    
    while state.running:
        t0 = time.time()
        
        with state.lock:
            renderer.update_scene(state.data)
        
        frame = renderer.render()
        
        # Encode as JPEG
        try:
            from PIL import Image
            img = Image.fromarray(frame)
            buf = io.BytesIO()
            img.save(buf, format='JPEG', quality=85)
            state.latest_frame = buf.getvalue()
            state.frame_ready.set()
        except ImportError:
            # Fallback: raw PPM
            header = f"P6\n{width} {height}\n255\n".encode()
            state.latest_frame = header + frame.tobytes()
            state.frame_ready.set()
        
        elapsed = time.time() - t0
        sleep_time = max(0, frame_interval - elapsed)
        if sleep_time > 0:
            time.sleep(sleep_time)
    
    renderer.close()


class MJPEGHandler(BaseHTTPRequestHandler):
    """HTTP handler that serves MJPEG stream."""
    
    sim_state = None  # Set by server
    
    def do_GET(self):
        if self.path == '/' or self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            
            while self.sim_state.running:
                self.sim_state.frame_ready.wait(timeout=1.0)
                self.sim_state.frame_ready.clear()
                
                frame_data = self.sim_state.latest_frame
                if frame_data is None:
                    continue
                
                try:
                    self.wfile.write(b'--frame\r\n')
                    self.wfile.write(b'Content-Type: image/jpeg\r\n')
                    self.wfile.write(f'Content-Length: {len(frame_data)}\r\n'.encode())
                    self.wfile.write(b'\r\n')
                    self.wfile.write(frame_data)
                    self.wfile.write(b'\r\n')
                    self.flush()
                except (BrokenPipeError, ConnectionResetError):
                    break
        elif self.path == '/status':
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain')
            self.end_headers()
            d = self.sim_state.data
            pos = d.qpos[:3]
            info = f"time={d.time:.1f}s pos=({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})"
            self.wfile.write(info.encode())
        else:
            # Serve simple HTML page
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            html = """<!DOCTYPE html>
<html><head><title>Scorpio MuJoCo Sim</title>
<style>
body { margin:0; background:#1a1a2e; color:#eee; font-family:sans-serif; display:flex; flex-direction:column; align-items:center; }
h2 { margin:10px; }
img { max-width:95vw; max-height:80vh; border:2px solid #444; border-radius:8px; }
#status { margin:8px; font-size:14px; color:#aaa; }
</style></head>
<body>
<h2>SEB-Naver Car-like Robot Simulation</h2>
<img src="/stream" alt="Simulation"/>
<div id="status">Loading...</div>
<script>
setInterval(async()=>{try{const r=await fetch('/status');document.getElementById('status').textContent=await r.text()}catch(e){}},500);
</script>
</body></html>"""
            self.wfile.write(html.encode())
    
    def log_message(self, format, *args):
        pass  # Suppress request logs


def simple_nav_controller(model, data):
    """Simple goal-seeking controller with obstacle avoidance."""
    DELTA_MAX = 0.785
    
    pos = data.qpos[:3]
    quat = data.qpos[3:7]
    w, x, y, z = quat
    yaw = np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    
    goal_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'goal_site')
    if goal_id < 0:
        return
    goal = data.site_xpos[goal_id]
    
    dx, dy = goal[0] - pos[0], goal[1] - pos[1]
    dist = np.sqrt(dx*dx + dy*dy)
    desired_yaw = np.arctan2(dy, dx)
    yaw_error = desired_yaw - yaw
    while yaw_error > np.pi: yaw_error -= 2*np.pi
    while yaw_error < -np.pi: yaw_error += 2*np.pi
    
    steer = np.clip(yaw_error * 3.0, -DELTA_MAX, DELTA_MAX)
    
    # Obstacle avoidance
    lidar_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'lidar_site')
    speed_factor = 1.0
    if lidar_id >= 0:
        lp = data.site_xpos[lidar_id]
        lm = data.site_xmat[lidar_id].reshape(3, 3)
        min_dist = 999
        blocked = 0
        for a in [-0.3, -0.15, 0, 0.15, 0.3]:
            d_local = np.array([np.cos(a), np.sin(a), 0])
            d_world = lm @ d_local
            gid = np.array([-1], dtype=np.int32)
            rd = mujoco.mj_ray(model, data, lp, d_world, None, 1, -1, gid)
            if 0 <= rd < min_dist:
                min_dist = rd
                blocked = 1 if a < 0 else (-1 if a > 0 else 0)
        if min_dist < 2.0:
            steer = np.clip(steer + blocked * 0.5, -DELTA_MAX, DELTA_MAX)
            speed_factor *= min(1.0, min_dist / 2.0)
    
    speed_factor = max(0.15, speed_factor * (1.0 - abs(yaw_error) * 0.6 / np.pi))
    if dist < 3.0:
        speed_factor *= max(0.1, dist / 3.0)
    throttle = np.clip(speed_factor * 12.0, 0, 15.0)
    
    data.ctrl[0] = throttle
    data.ctrl[1] = throttle
    data.ctrl[2] = steer


def reset_robot_facing_goal(model, data):
    """Orient robot toward goal at start."""
    goal_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'goal_site')
    if goal_id < 0:
        return
    goal = data.site_xpos[goal_id]
    pos = data.qpos[:3]
    yaw = np.arctan2(goal[1] - pos[1], goal[0] - pos[0])
    data.qpos[3] = np.cos(yaw / 2)
    data.qpos[4] = 0
    data.qpos[5] = 0
    data.qpos[6] = np.sin(yaw / 2)
    mujoco.mj_forward(model, data)


def main():
    parser = argparse.ArgumentParser(description='MuJoCo Web Viewer')
    parser.add_argument('--terrain', type=str, default='mountain',
                        choices=['mountain', 'forest', 'snowy_mountain', 'pump_track'])
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--width', type=int, default=960)
    parser.add_argument('--height', type=int, default=720)
    args = parser.parse_args()
    
    # Load world
    script_dir = os.path.dirname(os.path.abspath(__file__))
    base_dir = os.path.dirname(script_dir)
    world_path = os.path.join(base_dir, 'worlds', f'{args.terrain}.xml')
    
    if not os.path.exists(world_path):
        print(f"Generating terrains...")
        import sys; sys.path.insert(0, script_dir)
        from generate_terrain import generate_all_terrains
        generate_all_terrains(base_dir)
    
    print(f"Loading: {world_path}")
    model = mujoco.MjModel.from_xml_path(world_path)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    reset_robot_facing_goal(model, data)
    
    state = SimState(model, data)
    
    # Start threads
    phys_thread = threading.Thread(target=physics_loop, 
                                    args=(state, simple_nav_controller, 500), daemon=True)
    render_thread = threading.Thread(target=render_loop,
                                      args=(state, args.width, args.height, 30), daemon=True)
    
    phys_thread.start()
    render_thread.start()
    
    # Start HTTP server
    MJPEGHandler.sim_state = state
    server = HTTPServer(('0.0.0.0', args.port), MJPEGHandler)
    
    print(f"\n{'='*60}")
    print(f"  SEB-Naver Web Viewer")
    print(f"  Terrain: {args.terrain}")
    print(f"  Open in browser: http://localhost:{args.port}")
    print(f"  Press Ctrl+C to stop")
    print(f"{'='*60}\n")
    
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down...")
        state.running = False
        server.shutdown()


if __name__ == '__main__':
    main()

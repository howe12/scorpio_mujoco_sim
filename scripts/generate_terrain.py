#!/usr/bin/env python3
"""
Procedural terrain generator for SEB-Naver simulation scenarios.

Generates MuJoCo heightfield meshes for:
  - Mountain: rugged elevation with steep slopes
  - Forest: moderate terrain with tree obstacles
  - Snowy Mountain: steep slopes requiring reversing
  - Pump Track: continuous wave bumps for attitude control testing

Inspired by EPFL procedural terrain generator used in the paper.
"""

import numpy as np
import os


def perlin_like_noise(x, y, octaves=4, persistence=0.5, scale=1.0):
    """Simple multi-octave value noise (no external dependency)."""
    total = 0.0
    amplitude = 1.0
    frequency = scale
    max_val = 0.0
    
    for _ in range(octaves):
        # Hash-based pseudo-random gradient
        ix = np.floor(x * frequency).astype(int)
        iy = np.floor(y * frequency).astype(int)
        fx = x * frequency - ix
        fy = y * frequency - iy
        
        # Smoothstep interpolation
        sx = fx * fx * (3 - 2 * fx)
        sy = fy * fy * (3 - 2 * fy)
        
        # Pseudo-random values at corners
        def hash_val(xi, yi):
            n = xi * 374761393 + yi * 668265263
            n = (n ^ (n >> 13)) * 1274126177
            return ((n ^ (n >> 16)) & 0x7fffffff) / 0x7fffffff
        
        n00 = hash_val(ix, iy)
        n10 = hash_val(ix + 1, iy)
        n01 = hash_val(ix, iy + 1)
        n11 = hash_val(ix + 1, iy + 1)
        
        nx0 = n00 * (1 - sx) + n10 * sx
        nx1 = n01 * (1 - sx) + n11 * sx
        val = nx0 * (1 - sy) + nx1 * sy
        
        total += val * amplitude
        max_val += amplitude
        amplitude *= persistence
        frequency *= 2.0
    
    return total / max_val


def generate_heightmap(size_m, resolution, terrain_type, seed=42):
    """Generate heightmap array for a given terrain type."""
    np.random.seed(seed)
    n = int(size_m / resolution)
    x = np.linspace(0, size_m, n)
    y = np.linspace(0, size_m, n)
    X, Y = np.meshgrid(x, y)
    
    if terrain_type == "mountain":
        # Rugged mountainous terrain with multiple peaks
        h = np.zeros_like(X)
        # Base terrain
        h += perlin_like_noise(X, Y, octaves=5, persistence=0.6, scale=0.15) * 1.5
        # Add peaks
        for cx, cy, height, width in [(8, 8, 2.0, 3.0), (15, 5, 1.5, 2.5), 
                                        (5, 15, 1.8, 2.0), (18, 18, 1.2, 3.5)]:
            h += height * np.exp(-((X - cx)**2 + (Y - cy)**2) / (2 * width**2))
        # Rocky detail
        h += perlin_like_noise(X, Y, octaves=3, persistence=0.4, scale=0.8) * 0.3
        h = np.clip(h, -0.5, 3.0)
        
    elif terrain_type == "forest":
        # Gentle rolling hills with moderate variation
        h = perlin_like_noise(X, Y, octaves=4, persistence=0.5, scale=0.1) * 1.0
        h += perlin_like_noise(X, Y, octaves=3, persistence=0.3, scale=0.4) * 0.3
        h = np.clip(h, -0.3, 1.5)
        
    elif terrain_type == "snowy_mountain":
        # Steep slopes with high elevation change
        h = perlin_like_noise(X, Y, octaves=5, persistence=0.65, scale=0.12) * 2.5
        # Large ridge
        h += 1.5 * np.exp(-(Y - 10)**2 / 20) * (1 + 0.3 * np.sin(X * 0.5))
        # Steep sections
        h += perlin_like_noise(X, Y, octaves=3, persistence=0.5, scale=0.5) * 0.5
        h = np.clip(h, -0.5, 4.0)
        
    elif terrain_type == "pump_track":
        # Continuous sinusoidal bumps (wave pattern)
        wavelength_x = 3.0
        wavelength_y = 4.0
        amplitude = 0.25
        h = amplitude * np.sin(2 * np.pi * X / wavelength_x)
        h += amplitude * 0.5 * np.sin(2 * np.pi * Y / wavelength_y)
        # Add banked turns
        r = np.sqrt((X - 10)**2 + (Y - 10)**2)
        bank = 0.15 * np.exp(-((r - 5)**2) / 2)
        h += bank
        h = np.clip(h, -0.5, 1.0)
        
    else:
        raise ValueError(f"Unknown terrain type: {terrain_type}")
    
    return h


def heightmap_to_obj(heightmap, resolution, output_path, name="terrain"):
    """Convert heightmap to OBJ mesh file."""
    ny, nx = heightmap.shape
    
    vertices = []
    faces = []
    
    # Generate vertices
    for j in range(ny):
        for i in range(nx):
            x = i * resolution
            y = j * resolution
            z = heightmap[j, i]
            vertices.append((x, y, z))
    
    # Generate faces (two triangles per quad)
    # Counter-clockwise winding → normals pointing UP (+Z)
    for j in range(ny - 1):
        for i in range(nx - 1):
            v0 = j * nx + i + 1          # 1-indexed for OBJ
            v1 = j * nx + i + 2
            v2 = (j + 1) * nx + i + 1
            v3 = (j + 1) * nx + i + 2
            faces.append((v0, v1, v2))
            faces.append((v1, v3, v2))
    
    with open(output_path, 'w') as f:
        f.write(f"# Terrain mesh: {name}\n")
        f.write(f"# Size: {nx}x{ny} vertices, resolution: {resolution}m\n")
        for v in vertices:
            f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
        for face in faces:
            f.write(f"f {face[0]} {face[1]} {face[2]}\n")
    
    print(f"  Written {output_path}: {len(vertices)} verts, {len(faces)} faces")


def heightmap_to_hfield_data(heightmap, h_max):
    """Convert heightmap to MuJoCo heightfield data array [0, 1].
    
    MuJoCo hfield_data is a flat array of floats in [0, 1].
    0 = elevation_bottom, 1 = elevation_top.
    """
    h = heightmap.copy()
    h = np.clip(h, 0, h_max)
    # Normalize to [0.001, 1] to avoid exact zero (which can cause issues)
    h = 0.001 + 0.999 * h / h_max
    return h


def generate_terrain_mjcf(terrain_type, mesh_dir, world_dir, size_m=20.0, resolution=0.2):
    """Generate complete MJCF world file with terrain and car-like robot.
    
    Uses MuJoCo native heightfield (hfield) for reliable collision and rendering.
    """
    hf_size = int(size_m / resolution)  # 100x100 for 20m @ 0.2m
    
    print(f"\nGenerating '{terrain_type}' terrain ({size_m}m x {size_m}m, {hf_size}x{hf_size} heightfield)...")
    
    # Generate heightmap at hf_size resolution
    hmap = generate_heightmap(size_m, resolution, terrain_type)
    h_max = max(hmap.max(), 0.1)
    
    # Save heightfield data as binary
    hf_data = heightmap_to_hfield_data(hmap, h_max)
    hf_data_path = os.path.join(mesh_dir, f"{terrain_type}_hfield.bin")
    hf_data.astype(np.float32).tofile(hf_data_path)
    
    # Also save OBJ for external visualization (optional)
    obj_name = f"{terrain_type}_terrain.obj"
    obj_path = os.path.join(mesh_dir, obj_name)
    heightmap_to_obj(hmap, resolution, obj_path, terrain_type)
    
    # 起点/终点平面位置
    START_XY = {
        "mountain": (2.0, 2.0),
        "forest": (2.0, 2.0),
        "snowy_mountain": (2.0, 10.0),
        "pump_track": (1.0, 10.0),
    }
    GOAL_XY = {
        "mountain": (16.0, 16.0),
        "forest": (17.0, 17.0),
        "snowy_mountain": (18.0, 10.0),
        "pump_track": (19.0, 10.0),
    }

    def _ground_z(x, y):
        """从高度图取 (x,y) 处地面高度. 映射: hmap[y/res, x/res] (已实测确认)."""
        i = min(int(x / resolution), hf_size - 1)
        j = min(int(y / resolution), hf_size - 1)
        return float(hmap[j, i])

    sx, sy = START_XY[terrain_type]
    gx, gy = GOAL_XY[terrain_type]
    # 机器人起点放在地面上方 5cm (轻微下落, 避免悬空或穿透)
    start_pos = f"{sx:.2f} {sy:.2f} {_ground_z(sx, sy) + 0.05:.3f}"
    # 目标标记立在地面上方 0.5m
    goal_marker_pos = f"{gx:.2f} {gy:.2f} {_ground_z(gx, gy) + 0.5:.3f}"
    
    # Build world MJCF with native heightfield
    half = size_m / 2.0  # 10m half-width
    snow_mat_line = "<material name='snow_mat' rgba='0.95 0.95 0.98 1'/>" if terrain_type == "snowy_mountain" else ""
    terrain_rgba = "rgba='0.95 0.95 0.98 1'" if terrain_type == "snowy_mountain" else "rgba='0.35 0.4 0.25 1'"
    tree_bodies = _generate_trees(size_m) if terrain_type == "forest" else ""

    # 载入 URDF 校准过的 Scorpio 机器人 (所有场景共用)
    robot_body_tpl, robot_asset, robot_actuator, robot_sensor = _load_scorpio_robot(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    robot_body = robot_body_tpl.replace('{robot_start}', start_pos)
    
    world_xml = f'''<mujoco model="seb_naver_{terrain_type}">
  <compiler angle="radian" autolimits="true" meshdir="../models/meshes"/>
  <visual>
    <global offwidth="1280" offheight="960"/>
  </visual>
  <option timestep="0.002" gravity="0 0 -9.81" integrator="RK4">
    <flag contact="enable"/>
  </option>
  <size nconmax="500" njmax="1000"/>
  <default>
    <joint damping="0.3" frictionloss="0.02"/>
    <geom contype="1" conaffinity="1" friction="0.9 0.02 0.005"/>
  </default>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.4 0.5 0.6" rgb2="0.0 0.0 0.1" width="512" height="512"/>
    {robot_asset}
    <hfield name="terrain_hf" nrow="{hf_size}" ncol="{hf_size}" size="{half} {half} {h_max} 0.001"/>
    {snow_mat_line}
  </asset>
  <worldbody>
    <light name="sun" pos="10 10 20" dir="-0.5 -0.5 -1" diffuse="0.9 0.9 0.9" castshadow="true"/>
    <light name="fill" pos="-5 -5 10" dir="0.5 0.5 -1" diffuse="0.3 0.3 0.3"/>
    <geom name="terrain" type="hfield" hfield="terrain_hf" pos="{half} {half} 0" {terrain_rgba} contype="1" conaffinity="1" friction="0.8 0.02 0.01"/>
    <body name="goal_marker" pos="{goal_marker_pos}">
      <geom name="goal_visual" type="cylinder" size="0.3 0.5" rgba="0.1 0.8 0.1 0.6" contype="0" conaffinity="0" group="3"/>
      <site name="goal_site" pos="0 0 0"/>
    </body>
    <body name="start_marker" pos="{start_pos}">
      <geom name="start_visual" type="cylinder" size="0.3 0.5" rgba="0.1 0.3 0.9 0.6" contype="0" conaffinity="0" group="3"/>
    </body>
    <!-- Scorpio Robot (阿克曼转向, 与 models/scorpio.xml 同一模型) -->
    {robot_body}
    {tree_bodies}
  </worldbody>
  <actuator>
    {robot_actuator}
  </actuator>
  <sensor>
    {robot_sensor}
    <framepos name="goal_pos" objtype="site" objname="goal_site"/>
  </sensor>
</mujoco>'''
    
    world_path = os.path.join(world_dir, f"{terrain_type}.xml")
    with open(world_path, 'w') as f:
        f.write(world_xml)
    
    print(f"  World file: {world_path}")
    return world_path


def _load_scorpio_robot(base_dir):
    """从 models/scorpio.xml 提取 Scorpio 机器人定义, 复用到地形场景中.
    
    返回 (body_xml, asset_xml, actuator_xml, sensor_xml).
    这样所有场景共用同一套 URDF 校准过的机器人模型。
    """
    import re as _re
    src = os.path.join(base_dir, 'models', 'scorpio.xml')
    with open(src) as f:
        xml = f.read()

    # 提取 base_footprint 整个 body 块 (配对 </body>)
    start = xml.index('<body name="base_footprint"')
    depth, i = 0, start
    while True:
        nxt_open = xml.find('<body', i + 1)
        nxt_close = xml.find('</body>', i + 1)
        if nxt_close < 0:
            raise ValueError('unbalanced <body> in scorpio.xml')
        if 0 <= nxt_open < nxt_close:
            depth += 1; i = nxt_open
        else:
            if depth == 0:
                end = nxt_close + len('</body>')
                break
            depth -= 1; i = nxt_close
    body_xml = xml[start:end]

    asset_xml = _re.search(r'<asset>(.*?)</asset>', xml, _re.S).group(1)
    # 去掉 skybox/ground 纹理 (地形场景用 hfield 材质)
    asset_xml = _re.sub(r'<texture type="skybox".*?/>', '', asset_xml, flags=_re.S)
    asset_xml = _re.sub(r'<texture name="grid".*?/>', '', asset_xml, flags=_re.S)
    asset_xml = _re.sub(r'<material name="grid_mat".*?/>', '', asset_xml, flags=_re.S)
    actuator_xml = _re.search(r'<actuator>(.*?)</actuator>', xml, _re.S).group(1)
    sensor_xml = _re.search(r'<sensor>(.*?)</sensor>', xml, _re.S).group(1)

    # 机器人放到场景起始点: 把 base_footprint 的 pos 参数化
    body_xml = body_xml.replace('<body name="base_footprint" pos="0 0 0">',
                                '<body name="base_footprint" pos="{robot_start}">', 1)
    return body_xml, asset_xml, actuator_xml, sensor_xml


def _generate_trees(size_m, n_trees=30, seed=123):
    """Generate tree obstacle bodies for forest scene."""
    np.random.seed(seed)
    lines = []
    for i in range(n_trees):
        x = np.random.uniform(1, size_m - 1)
        y = np.random.uniform(1, size_m - 1)
        # Avoid start/goal areas
        if (x < 4 and y < 4) or (x > size_m - 4 and y > size_m - 4):
            continue
        height = np.random.uniform(1.5, 3.0)
        radius = np.random.uniform(0.1, 0.2)
        lines.append(f'''    <body name="tree_{i}" pos="{x:.1f} {y:.1f} {height/2:.1f}">
      <geom name="trunk_{i}" type="cylinder" size="{radius:.2f} {height/2:.1f}" 
            rgba="0.4 0.25 0.1 1" contype="1" conaffinity="1"/>
      <geom name="canopy_{i}" type="sphere" size="{radius*3:.2f}" pos="0 0 {height/2:.1f}"
            rgba="0.1 0.5 0.1 0.8" contype="0" conaffinity="0"/>
    </body>''')
    return '\n'.join(lines)


def generate_all_terrains(base_dir=None):
    """Generate all four SEB-Naver terrain scenarios."""
    if base_dir is None:
        base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    
    mesh_dir = os.path.join(base_dir, 'models', 'terrains')
    world_dir = os.path.join(base_dir, 'worlds')
    os.makedirs(mesh_dir, exist_ok=True)
    os.makedirs(world_dir, exist_ok=True)
    
    terrains = ['mountain', 'forest', 'snowy_mountain', 'pump_track']
    paths = {}
    
    for t in terrains:
        paths[t] = generate_terrain_mjcf(t, mesh_dir, world_dir)
    
    print(f"\n{'='*50}")
    print(f"All terrains generated in {world_dir}/")
    print(f"{'='*50}")
    return paths


if __name__ == '__main__':
    generate_all_terrains()

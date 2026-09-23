#!/usr/bin/env python3
"""
Scorpio LiDAR 仿真模块

支持 2D 激光雷达仿真，输出标准 ROS LaserScan 格式。
可用于:
1. 独立测试 (无需 ROS)
2. ROS2 桥接 (发布到 /scan 话题)
3. 强化学习观测空间

特性:
- 360° 扫描，720 点 (0.5° 分辨率)
- 扫描范围: 0.12m ~ 12.0m
- 扫描频率: 10Hz (可配置)
- 支持自定义扫描参数

使用示例:
    lidar = LiDARSimulator(model, data)
    scan_data = lidar.scan()  # 返回距离数组
"""

import numpy as np
import mujoco


class LiDARSimulator:
    """2D 激光雷达仿真器"""
    
    # YDLidar G6 官方规格
    G6_SPEC = {
        'num_beams': 720,        # 0.5° 角度分辨率
        'min_range': 0.12,       # 最小测距 (m)
        'max_range': 16.0,       # 最大测距 (m) — NXROBO 官方规格
        'scan_frequency': 12.0,  # 扫描频率 (Hz)
        'scan_angle': 360.0,     # 扫描角度 (°)
    }
    
    def __init__(self, model, data, 
                 num_beams=720,
                 min_range=0.12,
                 max_range=16.0,
                 min_angle=-np.pi,
                 max_angle=np.pi,
                 height_offset=0.0,
                 ignore_array="",
                 exclude_robot=False):
        """
        初始化 LiDAR 仿真器
        
        Args:
            model: MuJoCo model
            data: MuJoCo data
            num_beams: 扫描点数 (默认 720)
            min_range: 最小测距 (m)
            max_range: 最大测距 (m)
            min_angle: 最小角度 (rad, 默认 -π)
            max_angle: 最大角度 (rad, 默认 π)
            height_offset: 高度偏移 (相对于 lidar_site)
            exclude_robot: 是否排除机器人本体 (group 1)。
                           False (默认) = 保留本体遮挡, 用 ignore_array 过滤 (贴近真实)
                           True = 完全排除本体, 得到纯净环境扫描
            ignore_array: 屏蔽角度区间, 格式同 ROS 驱动: "deg1,deg2,deg3,deg4"
                          成对出现, 每对屏蔽 [deg1,deg2] 区间。
                          用于屏蔽机器人自身结构造成的遮挡。
                          例如 "-150,-145,145,150" 屏蔽后部两个扇区。
        """
        self.model = model
        self.data = data
        self.num_beams = num_beams
        self.min_range = min_range
        self.max_range = max_range
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.height_offset = height_offset
        self.ignore_array = ignore_array
        self.exclude_robot = exclude_robot
        # geomgroup: 组0=环境, 组1=机器人本体, 组3=纯视觉标记(永不参与雷达)
        # MuJoCo geomgroup 为 6 元素数组, 1=包含
        self.geomgroup = np.array([1, 0 if exclude_robot else 1, 1, 0, 1, 1], dtype=np.int32)
        
        # 获取 lidar_site
        self.site_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, 'lidar_site'
        )
        if self.site_id < 0:
            raise ValueError("lidar_site not found in model")
        
        # 获取 lidar_link body id (用于排除雷达自身外壳, 避免自遮挡)
        # 真实雷达的扫描平面在外壳内部, 但外壳有 360° 透光窗口,
        # 仿真中需排除雷达本体几何体, 否则每束射线都会立即命中外壳。
        self.lidar_body_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, 'lidar_link'
        )
        if self.lidar_body_id < 0:
            self.lidar_body_id = -1  # 未找到则不排除
        
        # 预计算角度数组
        self.angles = np.linspace(min_angle, max_angle, num_beams)
        
        # 解析 ignore_array (需在 self.angles 之后)
        self.ignore_mask = self._parse_ignore_array(ignore_array)
        
        # 存储最近一次扫描结果
        self.last_scan = None
        self.scan_count = 0
    
    def _parse_ignore_array(self, ignore_array):
        """解析 ROS 驱动的 ignore_array 格式字符串."""
        if not ignore_array or not ignore_array.strip():
            return None
        try:
            vals = [float(v) for v in ignore_array.split(',') if v.strip()]
        except ValueError:
            return None
        if len(vals) < 2:
            return None
        mask = np.zeros(self.num_beams, dtype=bool)
        for k in range(0, len(vals) - 1, 2):
            a1, a2 = np.radians(vals[k]), np.radians(vals[k + 1])
            lo, hi = min(a1, a2), max(a1, a2)
            mask |= (self.angles >= lo) & (self.angles <= hi)
        return mask

    def auto_ignore_array(self, model=None, data=None, threshold=0.35):
        """
        自动检测机器人自身造成的遮挡角度, 生成 ignore_array 字符串。
        
        原理: 找出所有 < threshold 米的射线角度, 按连续区间分组,
              扩展少量余量后输出为 ignore_array 格式。
        
        Args:
            threshold: 近距阈值 (m)
        
        Returns:
            str: ignore_array 字符串
        """
        ranges = self.scan()   # 不应用屏蔽, 得到原始扫描
        near = ranges < threshold
        if not near.any():
            return ""
        segs = []
        idx = np.where(near)[0]
        start = idx[0]; prev = idx[0]
        for i in idx[1:]:
            if i != prev + 1:
                segs.append((start, prev)); start = i
            prev = i
        segs.append((start, prev))
        # 处理跨 ±180° 的环绕
        if len(segs) > 1 and segs[0][0] == 0 and segs[-1][1] == self.num_beams - 1:
            segs[0] = (segs[-1][0], segs[0][1] + self.num_beams)
            segs.pop()
        margin = np.radians(1.0)
        parts = []
        for a, b in segs:
            ang_a = self.angles[a % self.num_beams] - margin
            ang_b = self.angles[b % self.num_beams] + margin
            parts.append(f"{np.degrees(ang_a):.1f}")
            parts.append(f"{np.degrees(ang_b):.1f}")
        return ",".join(parts)

    def scan(self):
        """
        执行一次 360° 扫描
        
        Returns:
            numpy.ndarray: 距离数组 (num_beams,)
                - 有效范围: [min_range, max_range]
                - 无命中: inf
        """
        # 获取 LiDAR 位置和姿态 (世界坐标系)
        site_pos = self.data.site_xpos[self.site_id].copy()
        site_mat = self.data.site_xmat[self.site_id].reshape(3, 3)
        
        # 初始化结果数组
        ranges = np.full(self.num_beams, np.inf)
        
        # 逐射线检测
        for i, angle in enumerate(self.angles):
            # 射线方向 (局部坐标系 - 水平扫描)
            # LiDAR 水平扫描: angle=0 是前方 (+X), angle=π/2 是左方 (+Y)
            direction_local = np.array([
                np.cos(angle),
                np.sin(angle),
                0.0
            ])
            
            # 转换到世界坐标系
            direction_world = site_mat @ direction_local
            
            # MuJoCo 射线检测
            geom_id = np.array([-1], dtype=np.int32)
            dist = mujoco.mj_ray(
                self.model, self.data,
                site_pos, direction_world,
                self.geomgroup,     # geomgroup (排除纯视觉标记 group 3)
                1,                  # flg_static (包含静态几何体)
                self.lidar_body_id, # bodyexclude (排除雷达自身外壳)
                geom_id
            )
            
            # 处理结果
            if dist >= 0:
                ranges[i] = np.clip(dist, self.min_range, self.max_range)
        
        # 应用 ignore_array 屏蔽 (模拟真实驱动的 ignore_array 参数)
        if self.ignore_mask is not None:
            ranges[self.ignore_mask] = np.inf
        
        # 更新统计
        self.last_scan = ranges
        self.scan_count += 1
        
        return ranges
    
    def get_lidar_state(self):
        """
        获取完整的 LiDAR 状态
        
        Returns:
            dict: 包含扫描数据和元信息
                - ranges: 距离数组
                - angles: 角度数组 (rad)
                - min_range: 最小有效距离
                - max_range: 最大有效距离
                - num_beams: 扫描点数
                - scan_count: 扫描次数
        """
        if self.last_scan is None:
            self.scan()
        
        return {
            'ranges': self.last_scan.copy(),
            'angles': self.angles.copy(),
            'min_range': self.min_range,
            'max_range': self.max_range,
            'num_beams': self.num_beams,
            'scan_count': self.scan_count,
            'site_id': self.site_id,
            'site_pos': self.data.site_xpos[self.site_id].copy(),
            'site_mat': self.data.site_xmat[self.site_id].reshape(3, 3).copy(),
        }
    
    def to_ros_laserscan(self, ranges=None, timestamp=None):
        """
        转换为 ROS LaserScan 消息格式 (字典表示)
        
        Args:
            ranges: 距离数组 (如果为 None，使用 last_scan)
            timestamp: 时间戳 (如果为 None，使用 data.time)
        
        Returns:
            dict: ROS LaserScan 消息结构
        """
        if ranges is None:
            ranges = self.scan()
        
        if timestamp is None:
            timestamp = self.data.time
        
        return {
            'header': {
                'stamp': timestamp,
                'frame_id': 'lidar_link'
            },
            'angle_min': self.min_angle,
            'angle_max': self.max_angle,
            'angle_increment': (self.max_angle - self.min_angle) / self.num_beams,
            'time_increment': 0.0,
            'scan_time': 1.0/12.0,  # YDLidar G6: 12Hz
            'range_min': self.min_range,
            'range_max': self.max_range,
            'ranges': ranges.tolist(),
            'intensities': [0.0] * self.num_beams,
        }
    
    def visualize_ascii(self, ranges=None, width=60, height=30):
        """
        ASCII 可视化 LiDAR 扫描
        
        Args:
            ranges: 距离数组
            width: 输出宽度
            height: 输出高度
        
        Returns:
            str: ASCII 可视化字符串
        """
        if ranges is None:
            ranges = self.scan()
        
        # 创建地图
        grid = [[' ' for _ in range(width)] for _ in range(height)]
        
        # 中心点
        cx, cy = width // 2, height // 2
        grid[cy][cx] = 'O'  # 机器人位置
        
        # 根据距离绘制点
        for i, r in enumerate(ranges):
            if r >= self.max_range:
                continue
            
            # 角度
            angle = self.angles[i]
            
            # 计算屏幕坐标 (极坐标 -> 直角坐标)
            scale = min(width, height) / (2 * self.max_range)
            dx = r * np.cos(angle) * scale
            dy = r * np.sin(angle) * scale
            
            x = int(cx + dx)
            y = int(cy - dy)  # y 轴翻转
            
            if 0 <= x < width and 0 <= y < height:
                grid[y][x] = '#'
        
        # 转换为字符串
        lines = [''.join(row) for row in grid]
        return '\n'.join(lines)


def create_lidar_from_config(model, data, config=None):
    """
    从配置文件创建 LiDAR 仿真器
    
    Args:
        model: MuJoCo model
        data: MuJoCo data
        config: 配置字典 (可选)
    
    Returns:
        LiDARSimulator: LiDAR 仿真器实例
    """
    if config is None:
        config = {}
    
    return LiDARSimulator(
        model, data,
        num_beams=config.get('num_beams', LiDARSimulator.G6_SPEC['num_beams']),
        min_range=config.get('min_range', LiDARSimulator.G6_SPEC['min_range']),
        max_range=config.get('max_range', LiDARSimulator.G6_SPEC['max_range']),
        min_angle=config.get('min_angle', -np.pi),
        max_angle=config.get('max_angle', np.pi),
        height_offset=config.get('height_offset', 0.0),
    )


# ============================================================
# 测试代码
# ============================================================
if __name__ == '__main__':
    import sys
    import os
    
    # 添加父目录到路径
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    
    # 加载模型
    model_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'models', 'scorpio.xml'
    )
    
    print(f"加载模型: {model_path}")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    
    # 创建 LiDAR 仿真器
    lidar = LiDARSimulator(model, data)
    print(f"LiDAR 仿真器初始化完成")
    print(f"  site_id: {lidar.site_id}")
    print(f"  位置: {lidar.data.site_xpos[lidar.site_id].round(3)}")
    print(f"  扫描点数: {lidar.num_beams}")
    print(f"  范围: {lidar.min_range} ~ {lidar.max_range} m")
    
    # 执行扫描
    print("\n执行扫描...")
    ranges = lidar.scan()
    
    # 统计
    valid = ranges[ranges < lidar.max_range]
    print(f"\n扫描结果:")
    print(f"  总点数: {len(ranges)}")
    print(f"  有效点数: {len(valid)}")
    print(f"  最近距离: {valid.min():.3f} m")
    print(f"  最远距离: {valid.max():.3f} m")
    print(f"  平均距离: {valid.mean():.3f} m")
    
    # ASCII 可视化
    print("\nLiDAR 扫描可视化:")
    print(lidar.visualize_ascii(ranges, width=60, height=25))
    
    # ROS 格式
    ros_msg = lidar.to_ros_laserscan(ranges)
    print(f"\nROS LaserScan 格式:")
    print(f"  angle_min: {ros_msg['angle_min']:.3f} rad")
    print(f"  angle_max: {ros_msg['angle_max']:.3f} rad")
    print(f"  angle_increment: {ros_msg['angle_increment']:.6f} rad")
    print(f"  range_min: {ros_msg['range_min']} m")
    print(f"  range_max: {ros_msg['range_max']} m")
    
    print("\n✅ LiDAR 仿真模块测试完成")

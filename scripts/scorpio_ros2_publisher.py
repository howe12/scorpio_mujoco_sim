#!/usr/bin/env python3
"""
Scorpio ROS2 发布模块

为 MuJoCo 仿真脚本提供 ROS2 话题发布能力（可选）。
与 run_seb_naver_sim.py 配合使用，实现「GUI 窗口 + 键盘控制 + ROS2 话题」一体化。

发布的话题:
  /scan                 sensor_msgs/LaserScan   LiDAR 扫描 (720点, 0.12~16m, 10Hz)
  /odom                 nav_msgs/Odometry       里程计
  /imu/data             sensor_msgs/Imu         IMU 数据
  /camera/rgb/image_raw sensor_msgs/Image       RGB 图像 (可选，较慢)
  /tf, /tf_static       TF                      坐标变换

订阅的话题:
  /cmd_vel              geometry_msgs/Twist     速度指令 (可选)

使用方法:
    from scorpio_ros2_publisher import ScorpioROS2Publisher
    pub = ScorpioROS2Publisher(model, data)
    # 在仿真循环中每帧调用
    pub.publish()
    # 退出时
    pub.shutdown()
"""

import numpy as np


class ScorpioROS2Publisher:
    """将 MuJoCo 仿真数据发布到 ROS2 话题."""

    # 2D LiDAR 参数 (YDLidar G6)
    LIDAR_NUM_POINTS = 720
    LIDAR_MIN_RANGE = 0.12
    LIDAR_MAX_RANGE = 16.0
    LIDAR_SCAN_RATE = 10.0     # Hz

    # 3D LiDAR 参数 (Livox Mid-360 仿真)
    # Mid-360: 360°×70° FOV, 非重复扫描, ~200k pts/s → 10Hz ≈ 20k pts/frame
    LIDAR3D_H_FOV = 360.0          # 度
    LIDAR3D_V_FOV_MIN = -7.0       # 度 (下倾)
    LIDAR3D_V_FOV_MAX = 63.0       # 度 (上仰)
    LIDAR3D_H_RES = 360            # 水平线数
    LIDAR3D_V_LINES = 32           # 垂直线束数
    LIDAR3D_SCAN_RATE = 10.0       # Hz
    LIDAR3D_MIN_RANGE = 0.1        # m
    LIDAR3D_MAX_RANGE = 40.0       # m (Mid-360 标称 70m, 仿真缩短)

    # 轮子/底盘参数
    WHEEL_RADIUS = 0.0525

    def __init__(self, model, data, publish_images=False, verbose=True):
        """初始化 ROS2 发布器.

        Args:
            model: MuJoCo model
            data: MuJoCo data
            publish_images: 是否发布相机图像 (较慢，默认 False)
            verbose: 是否打印初始化信息
        """
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

        self.model = model
        self.data = data
        self.publish_images = publish_images
        self._rclpy = rclpy

        if not rclpy.ok():
            rclpy.init()

        self.node = rclpy.create_node('scorpio_mujoco_publisher')
        self._last_scan_time = -999.0
        self._frame_count = 0

        # QoS: 传感器数据用 BEST_EFFORT
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # 延迟导入消息类型
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu, LaserScan, Image, CameraInfo
        from geometry_msgs.msg import Twist
        from tf2_ros import TransformBroadcaster

        # 发布器
        self.odom_pub = self.node.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.node.create_publisher(Imu, '/imu/data', sensor_qos)
        self.scan_pub = self.node.create_publisher(LaserScan, '/scan', sensor_qos)

        if publish_images:
            self.rgb_pub = self.node.create_publisher(Image, '/camera/rgb/image_raw', sensor_qos)
            self.depth_pub = self.node.create_publisher(Image, '/camera/depth/image_raw', sensor_qos)
            self.cam_info_pub = self.node.create_publisher(CameraInfo, '/camera/rgb/camera_info', sensor_qos)
            self._renderer = None

        # TF 广播
        self.tf_broadcaster = TransformBroadcaster(self.node)

        # 缓存传感器 ID
        self._sensor_ids = {}
        for name in ['base_pos', 'base_quat', 'base_linvel', 'base_angvel',
                     'imu_accel', 'imu_gyro']:
            sid = self._find_sensor(name)
            self._sensor_ids[name] = sid

        self._lidar_site_id = self._find_site('lidar_site')
        self._base_body_id = self._find_body('base_link')
        # 雷达本体 body id: 射线检测时排除, 否则会立即打到自己的外壳
        # (扫描中心在雷达外壳内部, 这是真实雷达的物理结构)
        self._lidar_body_id = self._find_body('lidar_link')

        # geomgroup 过滤: 排除纯视觉标记 (group 3)
        #   group 0 = 环境(地面/墙体/障碍物)
        #   group 1 = 机器人本体
        #   group 3 = 起点/终点标记圆柱 (不可参与雷达扫描)
        # 起点标记是半径 0.3m 的圆柱且套在机器人身上, 不过滤会导致
        # 所有射线在 0.30m 处命中, SLAM 完全无法建图。
        self._geomgroup = np.array([1, 1, 1, 0, 1, 1], dtype=np.int32)

        # 3D LiDAR 非重复扫描相位 (模拟 Livox Lissajous 模式)
        self._lidar3d_phase = 0.0

        # PointCloud2 发布器
        from sensor_msgs.msg import PointCloud2
        self.cloud_pub = self.node.create_publisher(PointCloud2, '/livox/lidar', sensor_qos)
        self._last_cloud_time = -999.0

        if verbose:
            topics = "/odom, /imu/data, /scan, /livox/lidar"
            if publish_images:
                topics += ", /camera/*"
            print(f"\n  [ROS2] 发布器已启动")
            print(f"    {topics}")
            print(f"    3D LiDAR: Livox Mid-360 仿真 ({self.LIDAR3D_H_RES}×{self.LIDAR3D_V_LINES}, {self.LIDAR3D_SCAN_RATE}Hz)")
            if self._lidar_site_id < 0:
                print(f"    ⚠ 未找到 lidar_site，/scan 和 /livox/lidar 将无数据")
            if self._base_body_id < 0:
                print(f"    ⚠ 未找到 base_link，/odom 将无数据")

    # ---- MuJoCo 查找辅助 ----
    def _find_sensor(self, name):
        import mujoco
        sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)
        return sid

    def _find_site(self, name):
        import mujoco
        return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, name)

    def _find_body(self, name):
        import mujoco
        return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)

    def _read_sensor(self, name, dim):
        import mujoco
        sid = self._sensor_ids.get(name, -1)
        if sid is None or sid < 0:
            return np.zeros(dim)
        addr = self.model.sensor_adr[sid]
        return np.asarray(self.data.sensordata[addr:addr + dim])

    # ---- 扫描 ----
    def _render_lidar(self):
        """用射线投射生成 LiDAR 扫描."""
        import mujoco
        if self._lidar_site_id < 0:
            return np.full(self.LIDAR_NUM_POINTS, np.inf)

        site_pos = self.data.site_xpos[self._lidar_site_id].copy()
        site_mat = self.data.site_xmat[self._lidar_site_id].reshape(3, 3)

        angles = np.linspace(-np.pi, np.pi, self.LIDAR_NUM_POINTS, endpoint=False)
        ranges = np.full(self.LIDAR_NUM_POINTS, np.inf, dtype=np.float32)

        for i, ang in enumerate(angles):
            direction = site_mat @ np.array([np.cos(ang), np.sin(ang), 0.0])
            geom_id = np.array([-1], dtype=np.int32)
            dist = mujoco.mj_ray(self.model, self.data, site_pos, direction,
                                 self._geomgroup, 1, self._lidar_body_id, geom_id)
            if dist >= 0:
                ranges[i] = np.clip(dist, self.LIDAR_MIN_RANGE, self.LIDAR_MAX_RANGE)

        return ranges

    # ---- 3D LiDAR ----
    def _render_3d_lidar(self):
        """用射线投射生成 3D 点云 (模拟 Livox Mid-360).

        非重复扫描: 每帧的垂直角度有 Lissajous 偏移, 多帧累积覆盖完整 FOV.
        返回 Nx3 numpy array (xyz in lidar frame) 和强度数组.
        """
        import mujoco
        if self._lidar_site_id < 0:
            return np.zeros((0, 3), dtype=np.float32), np.zeros(0, dtype=np.float32)

        site_pos = self.data.site_xpos[self._lidar_site_id].copy()
        site_mat = self.data.site_xmat[self._lidar_site_id].reshape(3, 3)

        h_angles = np.linspace(-np.pi, np.pi, self.LIDAR3D_H_RES, endpoint=False)
        v_min = np.radians(self.LIDAR3D_V_FOV_MIN)
        v_max = np.radians(self.LIDAR3D_V_FOV_MAX)

        # 非重复扫描: 用黄金比例偏移模拟 Lissajous 填充
        golden = 0.618033988749895
        self._lidar3d_phase = (self._lidar3d_phase + golden * 2 * np.pi) % (2 * np.pi)

        points = []
        intensities = []

        for vi in range(self.LIDAR3D_V_LINES):
            # 基础垂直角均匀分布
            base_v = v_min + (v_max - v_min) * vi / max(self.LIDAR3D_V_LINES - 1, 1)
            # Lissajous 偏移: 不同水平位置有不同的垂直抖动
            for hi, h_ang in enumerate(h_angles):
                # 非重复偏移: sin(phase + h_idx * golden_ratio)
                jitter = 0.02 * np.sin(self._lidar3d_phase + hi * golden)
                v_ang = base_v + jitter
                v_ang = np.clip(v_ang, v_min, v_max)

                # 球坐标 → 方向向量 (lidar frame: +X forward, +Y left, +Z up)
                dx = np.cos(v_ang) * np.cos(h_ang)
                dy = np.cos(v_ang) * np.sin(h_ang)
                dz = np.sin(v_ang)
                direction = site_mat @ np.array([dx, dy, dz])

                geom_id = np.array([-1], dtype=np.int32)
                dist = mujoco.mj_ray(self.model, self.data, site_pos, direction,
                                     self._geomgroup, 1, self._lidar_body_id, geom_id)
                if dist >= 0 and self.LIDAR3D_MIN_RANGE <= dist <= self.LIDAR3D_MAX_RANGE:
                    # 转换到 lidar_link 坐标系
                    local_pt = site_mat.T @ (direction * dist)
                    points.append(local_pt)
                    # 简单强度模型: 近距离更强
                    intensity = max(0.0, min(255.0, 200.0 * (1.0 - dist / self.LIDAR3D_MAX_RANGE)))
                    intensities.append(intensity)

        if not points:
            return np.zeros((0, 3), dtype=np.float32), np.zeros(0, dtype=np.float32)

        return np.array(points, dtype=np.float32), np.array(intensities, dtype=np.float32)

    def _publish_pointcloud(self, stamp):
        """发布 3D PointCloud2 (/livox/lidar)."""
        from sensor_msgs.msg import PointCloud2, PointField
        import struct

        pts_xyz, intensities = self._render_3d_lidar()
        n = len(pts_xyz)
        if n == 0:
            return

        # PointCloud2 消息
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = 'lidar_link'
        msg.height = 1
        msg.width = n
        msg.is_bigendian = False
        msg.is_dense = True

        # 字段定义: x,y,z (float32) + intensity (float32) + timestamp (uint64)
        # FAST-LIO2 需要 x,y,z,intensity,timestamp 或至少 x,y,z,intensity
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        point_step = 16  # 4 floats × 4 bytes
        msg.fields = fields
        msg.point_step = point_step
        msg.row_step = point_step * n

        # 打包数据
        buf = bytearray(n * point_step)
        ts_ns = int(stamp.sec * 1e9 + stamp.nanosec)
        for i in range(n):
            offset = i * point_step
            struct.pack_into('<fffI', buf, offset,
                             float(pts_xyz[i, 0]),
                             float(pts_xyz[i, 1]),
                             float(pts_xyz[i, 2]),
                             int(intensities[i]))
        msg.data = bytes(buf)

        self.cloud_pub.publish(msg)

    # ---- 发布 ----
    def publish(self):
        """发布一帧所有话题 (在仿真循环中每帧调用)."""
        from geometry_msgs.msg import Quaternion, TransformStamped
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu, LaserScan

        stamp = self.node.get_clock().now().to_msg()

        # --- 位姿 (直接读 qpos 更可靠) ---
        pos = np.asarray(self.data.qpos[:3])
        quat_wxyz = np.asarray(self.data.qpos[3:7])  # w,x,y,z

        # --- /odom ---
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = float(pos[0])
        odom.pose.pose.position.y = float(pos[1])
        odom.pose.pose.position.z = float(pos[2])
        odom.pose.pose.orientation = Quaternion(
            x=float(quat_wxyz[1]), y=float(quat_wxyz[2]),
            z=float(quat_wxyz[3]), w=float(quat_wxyz[0]))

        linvel = self._read_sensor('base_linvel', 3)
        angvel = self._read_sensor('base_angvel', 3)
        odom.twist.twist.linear.x = float(linvel[0])
        odom.twist.twist.linear.y = float(linvel[1])
        odom.twist.twist.linear.z = float(linvel[2])
        odom.twist.twist.angular.x = float(angvel[0])
        odom.twist.twist.angular.y = float(angvel[1])
        odom.twist.twist.angular.z = float(angvel[2])
        self.odom_pub.publish(odom)

        # --- /tf: odom -> base_footprint ---
        tf_odom = TransformStamped()
        tf_odom.header.stamp = stamp
        tf_odom.header.frame_id = 'odom'
        tf_odom.child_frame_id = 'base_footprint'
        tf_odom.transform.translation.x = float(pos[0])
        tf_odom.transform.translation.y = float(pos[1])
        tf_odom.transform.translation.z = float(pos[2])
        tf_odom.transform.rotation = Quaternion(
            x=float(quat_wxyz[1]), y=float(quat_wxyz[2]),
            z=float(quat_wxyz[3]), w=float(quat_wxyz[0]))
        self.tf_broadcaster.sendTransform(tf_odom)

        # --- /tf: base_footprint -> base_link (重合) ---
        tf_base = TransformStamped()
        tf_base.header.stamp = stamp
        tf_base.header.frame_id = 'base_footprint'
        tf_base.child_frame_id = 'base_link'
        tf_base.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf_base)

        # --- /tf: base_link -> lidar_link (静态, z=0.282) ---
        tf_lidar = TransformStamped()
        tf_lidar.header.stamp = stamp
        tf_lidar.header.frame_id = 'base_link'
        tf_lidar.child_frame_id = 'lidar_link'
        tf_lidar.transform.translation.z = 0.28172
        tf_lidar.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf_lidar)

        # --- /imu/data ---
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'IMU_link'
        accel = self._read_sensor('imu_accel', 3)
        gyro = self._read_sensor('imu_gyro', 3)
        imu.linear_acceleration.x = float(accel[0])
        imu.linear_acceleration.y = float(accel[1])
        imu.linear_acceleration.z = float(accel[2])
        imu.angular_velocity.x = float(gyro[0])
        imu.angular_velocity.y = float(gyro[1])
        imu.angular_velocity.z = float(gyro[2])
        imu.orientation = Quaternion(
            x=float(quat_wxyz[1]), y=float(quat_wxyz[2]),
            z=float(quat_wxyz[3]), w=float(quat_wxyz[0]))
        # 协方差 (SLAM Toolbox 需要有效的协方差)
        imu.orientation_covariance[0] = 0.01
        imu.angular_velocity_covariance[0] = 0.01
        imu.linear_acceleration_covariance[0] = 0.01
        self.imu_pub.publish(imu)

        # --- /scan (限频 10Hz) ---
        t = float(self.data.time)
        if t - self._last_scan_time >= 1.0 / self.LIDAR_SCAN_RATE:
            ranges = self._render_lidar()
            scan = LaserScan()
            scan.header.stamp = stamp
            scan.header.frame_id = 'lidar_link'
            scan.angle_min = float(-np.pi)
            scan.angle_max = float(np.pi)
            scan.angle_increment = float(2 * np.pi / self.LIDAR_NUM_POINTS)
            scan.time_increment = 0.0
            scan.scan_time = float(1.0 / self.LIDAR_SCAN_RATE)
            scan.range_min = float(self.LIDAR_MIN_RANGE)
            scan.range_max = float(self.LIDAR_MAX_RANGE)
            scan.ranges = [float(r) if np.isfinite(r) else float('inf') for r in ranges]
            scan.intensities = []
            self.scan_pub.publish(scan)
            self._last_scan_time = t

        # --- /livox/lidar 3D PointCloud2 (限频 10Hz) ---
        if t - self._last_cloud_time >= 1.0 / self.LIDAR3D_SCAN_RATE:
            self._publish_pointcloud(stamp)
            self._last_cloud_time = t

        # --- 相机图像 (可选) ---
        if self.publish_images:
            self._publish_cameras(stamp)

        # 处理 ROS2 事件 (非阻塞)
        self._rclpy.spin_once(self.node, timeout_sec=0.0)
        self._frame_count += 1

    def _publish_cameras(self, stamp):
        """发布 RGB 和深度图像 (较慢，默认关闭)."""
        import mujoco
        from sensor_msgs.msg import Image, CameraInfo
        import io

        try:
            if self._renderer is None:
                self._renderer = mujoco.Renderer(self.model, 480, 640)

            cam_rgb = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, 'd435_rgb')
            cam_depth = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, 'd435_depth')
            if cam_rgb < 0 or cam_depth < 0:
                return

            # RGB
            self._renderer.update_scene(self.data, camera=cam_rgb)
            rgb = self._renderer.render()
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = 'd435_rgb_optical_frame'
            msg.height, msg.width = rgb.shape[0], rgb.shape[1]
            msg.encoding = 'rgb8'
            msg.is_bigendian = False
            msg.step = rgb.shape[1] * 3
            msg.data = rgb.tobytes()
            self.rgb_pub.publish(msg)

            # Depth
            self._renderer.enable_depth_rendering()
            self._renderer.update_scene(self.data, camera=cam_depth)
            depth = self._renderer.render()
            self._renderer.disable_depth_rendering()
            dmsg = Image()
            dmsg.header.stamp = stamp
            dmsg.header.frame_id = 'd435_depth_optical_frame'
            dmsg.height, dmsg.width = depth.shape[0], depth.shape[1]
            dmsg.encoding = '32FC1'
            dmsg.is_bigendian = False
            dmsg.step = depth.shape[1] * 4
            dmsg.data = depth.astype(np.float32).tobytes()
            self.depth_pub.publish(dmsg)

            # CameraInfo
            info = CameraInfo()
            info.header.stamp = stamp
            info.header.frame_id = 'd435_rgb_optical_frame'
            info.height, info.width = 480, 640
            info.distortion_model = 'plumb_bob'
            info.d = [0.0] * 5
            fovy = np.radians(42.0)
            fy = 480 / (2.0 * np.tan(fovy / 2.0))
            fx = fy
            info.k = [fx, 0.0, 320.0, 0.0, fy, 240.0, 0.0, 0.0, 1.0]
            info.p = [fx, 0.0, 320.0, 0.0, 0.0, fy, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            self.cam_info_pub.publish(info)
        except Exception as exc:
            # 相机发布失败不影响主流程 (例如 EGL 不可用)
            if self._frame_count % 300 == 0:
                print(f"  [ROS2] 相机发布失败: {exc}")

    def shutdown(self):
        """关闭 ROS2 节点."""
        try:
            if self._renderer is not None:
                self._renderer.close()
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass

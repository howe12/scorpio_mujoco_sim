#!/usr/bin/env python3
"""
Scorpio MuJoCo-ROS2 Bridge Node

Bridges MuJoCo physics simulation with ROS2 topics/services:
- Subscribes to cmd_vel for differential drive control
- Publishes odom, tf, imu, scan (LiDAR), and camera topics
- Provides ground truth pose for evaluation

Supports both navigation/SLAM testing and RL training workflows.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import mujoco
import numpy as np
import os
import time
from threading import Thread, Lock

# ROS2 message types
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan, Image, CameraInfo
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster


class ScorpioMuJoCoBridge(Node):
    """ROS2 node bridging MuJoCo simulation for the Scorpio robot."""

    # Robot parameters (from URDF analysis)
    WHEEL_RADIUS = 0.0525  # meters
    WHEEL_SEPARATION = 0.1856  # 2 * 0.0928
    WHEELBASE = 0.315  # front-rear axle distance (m), Ackermann
    MAX_LINEAR_VEL = 0.26  # m/s
    MAX_ANGULAR_VEL = 1.0  # rad/s
    
    # LiDAR parameters (YDLidar G6 specs)
    LIDAR_NUM_POINTS = 720
    LIDAR_MIN_RANGE = 0.12  # meters
    LIDAR_MAX_RANGE = 12.0  # meters
    LIDAR_MIN_ANGLE = -np.pi  # radians
    LIDAR_MAX_ANGLE = np.pi   # radians
    LIDAR_UPDATE_RATE = 10.0  # Hz

    def __init__(self):
        super().__init__('scorpio_mujoco_bridge')
        
        # Declare parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('sim_rate', 500.0)  # MuJoCo physics rate
        self.declare_parameter('publish_rate', 50.0)  # ROS publish rate
        self.declare_parameter('use_headless', False)
        self.headless = self.get_parameter('use_headless').value
        
        model_path = self.get_parameter('model_path').value
        if not model_path:
            # Default path relative to package
            pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            model_path = os.path.join(pkg_dir, 'models', 'scorpio.xml')
        
        self.get_logger().info(f'Loading MuJoCo model from: {model_path}')
        
        # Initialize MuJoCo
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.mj_lock = Lock()
        
        # Get joint/actuator indices
        self.joint_names = [
            'left_front_wheel_joint', 'left_rear_wheel_joint',
            'right_front_wheel_joint', 'right_rear_wheel_joint'
        ]
        self.joint_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in self.joint_names
        ]
        
        # Sensor indices
        self.imu_accel_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, 'imu_accel')
        self.imu_gyro_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, 'imu_gyro')
        self.base_pos_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, 'base_pos')
        self.base_quat_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, 'base_quat')
        self.base_linvel_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, 'base_linvel')
        self.base_angvel_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, 'base_angvel')
        
        # Commanded velocity
        self.cmd_vel = Twist()
        self.cmd_vel_lock = Lock()
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', sensor_qos)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', sensor_qos)
        self.rgb_pub = self.create_publisher(Image, '/camera/rgb/image_raw', sensor_qos)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', sensor_qos)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/rgb/camera_info', sensor_qos)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Simulation state
        self.sim_time = 0.0
        self.last_scan_time = 0.0
        self.physics_rate = self.get_parameter('sim_rate').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Start physics thread
        self.physics_thread = Thread(target=self.physics_loop, daemon=True)
        self.physics_thread.start()
        
        # Timer for ROS publishing
        self.publish_timer = self.create_timer(1.0 / self.publish_rate, self.publish_loop)
        
        self.get_logger().info(
            f'Scorpio MuJoCo Bridge initialized. '
            f'Physics: {self.physics_rate}Hz, Publish: {self.publish_rate}Hz'
        )

    def cmd_vel_callback(self, msg: Twist):
        """Store commanded velocity (thread-safe)."""
        with self.cmd_vel_lock:
            # Clamp velocities
            linear = max(-self.MAX_LINEAR_VEL, min(self.MAX_LINEAR_VEL, msg.linear.x))
            angular = max(-self.MAX_ANGULAR_VEL, min(self.MAX_ANGULAR_VEL, msg.angular.z))
            self.cmd_vel.linear.x = linear
            self.cmd_vel.angular.z = angular

    def diff_drive_to_wheel_velocities(self, linear: float, angular: float):
        """Convert differential drive commands to individual wheel velocities."""
        # Skid-steer: left wheels = v + omega*L/2, right wheels = v - omega*L/2
        left_vel = (linear + angular * self.WHEEL_SEPARATION / 2.0) / self.WHEEL_RADIUS
        right_vel = (linear - angular * self.WHEEL_SEPARATION / 2.0) / self.WHEEL_RADIUS
        return left_vel, right_vel

    def physics_loop(self):
        """Run MuJoCo physics simulation in a separate thread."""
        dt = 1.0 / self.physics_rate
        
        while rclpy.ok():
            with self.mj_lock:
                # Read commanded velocity
                with self.cmd_vel_lock:
                    linear = self.cmd_vel.linear.x
                    angular = self.cmd_vel.angular.z
                
                # Ackermann drive: ctrl[0]=left_rear_motor, ctrl[1]=right_rear_motor,
                # ctrl[2]=steering_actuator (see models/scorpio.xml)
                steer_cmd = float(np.arctan2(self.WHEELBASE * angular, linear)) if abs(linear) > 1e-6 else 0.0
                steer_cmd = max(-0.785, min(0.785, steer_cmd))  # steering limit ~= +-45 deg
                rear_vel = linear / self.WHEEL_RADIUS           # rad/s
                self.data.ctrl[0] = rear_vel   # left rear
                self.data.ctrl[1] = rear_vel   # right rear
                self.data.ctrl[2] = steer_cmd  # steering angle (rad)
                
                # Step physics
                mujoco.mj_step(self.model, self.data)
                self.sim_time += dt

    def get_sensor_data(self):
        """Read all sensor data from MuJoCo (thread-safe)."""
        with self.mj_lock:
            def _sensor(id_):
                """Read sensor slice using MuJoCo address/dim mapping."""
                adr = self.model.sensor_adr[id_]
                dim = self.model.sensor_dim[id_]
                return self.data.sensordata[adr : adr + dim].copy()

            # Base pose (ground truth)
            base_pos = _sensor(self.base_pos_id)
            base_quat = _sensor(self.base_quat_id)
            base_linvel = _sensor(self.base_linvel_id)
            base_angvel = _sensor(self.base_angvel_id)

            # IMU
            imu_accel = _sensor(self.imu_accel_id)
            imu_gyro = _sensor(self.imu_gyro_id)
            
            # Wheel encoder velocities
            wheel_vels = []
            for jid in self.joint_ids:
                qpos_addr = self.model.jnt_qposadr[jid]
                wheel_vels.append(self.data.qpos[qpos_addr])
            
            # Render LiDAR
            lidar_ranges = self.render_lidar()
            
            # Render cameras
            rgb_img, depth_img = (None, None) if self.headless else self.render_cameras()
            
            return {
                'base_pos': base_pos,
                'base_quat': base_quat,
                'base_linvel': base_linvel,
                'base_angvel': base_angvel,
                'imu_accel': imu_accel,
                'imu_gyro': imu_gyro,
                'wheel_positions': wheel_vels,
                'lidar_ranges': lidar_ranges,
                'rgb_image': rgb_img,
                'depth_image': depth_img,
            }

    def render_lidar(self):
        """Simulate 2D LiDAR using MuJoCo ray casting."""
        lidar_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'lidar_site')
        
        # Get lidar position and orientation
        lidar_pos = self.data.site_xpos[lidar_site_id].copy()
        lidar_mat = self.data.site_xmat[lidar_site_id].reshape(3, 3)
        
        ranges = np.full(self.LIDAR_NUM_POINTS, self.LIDAR_MAX_RANGE, dtype=np.float32)
        angles = np.linspace(self.LIDAR_MIN_ANGLE, self.LIDAR_MAX_ANGLE, self.LIDAR_NUM_POINTS)
        
        for i, angle in enumerate(angles):
            # Ray direction in lidar frame
            direction_local = np.array([np.cos(angle), np.sin(angle), 0.0])
            # Transform to world frame
            direction_world = lidar_mat @ direction_local
            
            # Ray cast
            geom_id = np.array([-1], dtype=np.int32)
            dist = mujoco.mj_ray(
                self.model, self.data,
                lidar_pos, direction_world,
                None,  # geomgroup
                1,     # flg_static
                -1,    # bodyexclude
                geom_id
            )
            
            if dist >= 0 and dist <= self.LIDAR_MAX_RANGE:
                ranges[i] = max(dist, self.LIDAR_MIN_RANGE)
            else:
                ranges[i] = float('inf')
        
        return ranges

    def render_cameras(self):
        """Render RGB and depth images from MuJoCo cameras."""
        width, height = 640, 480
        
        # RGB camera
        rgb_renderer = mujoco.Renderer(self.model, height, width)
        rgb_cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, 'rgb_camera')
        rgb_renderer.update_scene(self.data, camera=rgb_cam_id)
        rgb_img = rgb_renderer.render()
        
        # Depth camera
        depth_renderer = mujoco.Renderer(self.model, height, width)
        depth_cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, 'depth_camera')
        depth_renderer.update_scene(self.data, camera=depth_cam_id)
        depth_renderer.enable_depth_rendering()
        depth_img = depth_renderer.render()
        depth_renderer.disable_depth_rendering()
        
        return rgb_img, depth_img

    def publish_loop(self):
        """Publish all sensor data to ROS2 topics."""
        now = self.get_clock().now()
        stamp = now.to_msg()
        
        try:
            sensors = self.get_sensor_data()
        except Exception as e:
            self.get_logger().warn(f'Sensor read error: {e}', throttle_duration_sec=1.0)
            return
        
        # === Odometry ===
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        pos = sensors['base_pos']
        quat = sensors['base_quat']
        linvel = sensors['base_linvel']
        angvel = sensors['base_angvel']
        
        odom.pose.pose.position.x = pos[0]
        odom.pose.pose.position.y = pos[1]
        odom.pose.pose.position.z = pos[2]
        odom.pose.pose.orientation = Quaternion(
            x=quat[1], y=quat[2], z=quat[3], w=quat[0]
        )
        
        # Transform linear velocity to body frame
        odom.twist.twist.linear.x = linvel[0]
        odom.twist.twist.linear.y = linvel[1]
        odom.twist.twist.linear.z = linvel[2]
        odom.twist.twist.angular.x = angvel[0]
        odom.twist.twist.angular.y = angvel[1]
        odom.twist.twist.angular.z = angvel[2]
        
        self.odom_pub.publish(odom)
        
        # === TF: odom -> base_footprint ===
        tf_odom = TransformStamped()
        tf_odom.header.stamp = stamp
        tf_odom.header.frame_id = 'odom'
        tf_odom.child_frame_id = 'base_footprint'
        tf_odom.transform.translation.x = pos[0]
        tf_odom.transform.translation.y = pos[1]
        tf_odom.transform.translation.z = pos[2]
        tf_odom.transform.rotation = Quaternion(
            x=quat[1], y=quat[2], z=quat[3], w=quat[0]
        )
        self.tf_broadcaster.sendTransform(tf_odom)
        
        # === TF: base_footprint -> base_link (static) ===
        tf_base = TransformStamped()
        tf_base.header.stamp = stamp
        tf_base.header.frame_id = 'base_footprint'
        tf_base.child_frame_id = 'base_link'
        tf_base.transform.translation.z = 0.0
        tf_base.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf_base)
        
        # === IMU ===
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'IMU_link'
        accel = sensors['imu_accel']
        gyro = sensors['imu_gyro']
        imu.linear_acceleration.x = accel[0]
        imu.linear_acceleration.y = accel[1]
        imu.linear_acceleration.z = accel[2]
        imu.angular_velocity.x = gyro[0]
        imu.angular_velocity.y = gyro[1]
        imu.angular_velocity.z = gyro[2]
        imu.orientation = Quaternion(x=quat[1], y=quat[2], z=quat[3], w=quat[0])
        self.imu_pub.publish(imu)
        
        # === LiDAR Scan ===
        current_time = time.time()
        if current_time - self.last_scan_time >= 1.0 / self.LIDAR_UPDATE_RATE:
            scan = LaserScan()
            scan.header.stamp = stamp
            scan.header.frame_id = 'lidar_link'
            scan.angle_min = self.LIDAR_MIN_ANGLE
            scan.angle_max = self.LIDAR_MAX_ANGLE
            scan.angle_increment = (self.LIDAR_MAX_ANGLE - self.LIDAR_MIN_ANGLE) / self.LIDAR_NUM_POINTS
            scan.range_min = self.LIDAR_MIN_RANGE
            scan.range_max = self.LIDAR_MAX_RANGE
            scan.ranges = sensors['lidar_ranges'].tolist()
            scan.intensities = [0.0] * self.LIDAR_NUM_POINTS
            self.scan_pub.publish(scan)
            self.last_scan_time = current_time
        
        # === Camera Images ===
        rgb_img = sensors['rgb_image']
        if rgb_img is not None:
            rgb_msg = Image()
            rgb_msg.header.stamp = stamp
            rgb_msg.header.frame_id = 'camera_rgb_optical_frame'
            rgb_msg.height = rgb_img.shape[0]
            rgb_msg.width = rgb_img.shape[1]
            rgb_msg.encoding = 'rgb8'
            rgb_msg.is_bigendian = False
            rgb_msg.step = rgb_img.shape[1] * 3
            rgb_msg.data = rgb_img.tobytes()
            self.rgb_pub.publish(rgb_msg)
            
            # Camera info
            cam_info = CameraInfo()
            cam_info.header.stamp = stamp
            cam_info.header.frame_id = 'camera_rgb_optical_frame'
            cam_info.height = rgb_img.shape[0]
            cam_info.width = rgb_img.shape[1]
            cam_info.distortion_model = 'plumb_bob'
            cam_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            fovy_rad = 60.0 * np.pi / 180.0
            fy = rgb_img.shape[0] / (2.0 * np.tan(fovy_rad / 2.0))
            fx = fy
            cx = rgb_img.shape[1] / 2.0
            cy = rgb_img.shape[0] / 2.0
            cam_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
            cam_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
            cam_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            self.camera_info_pub.publish(cam_info)
        
        depth_img = sensors['depth_image']
        if depth_img is not None:
            depth_msg = Image()
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = 'camera_depth_optical_frame'
            depth_msg.height = depth_img.shape[0]
            depth_msg.width = depth_img.shape[1]
            depth_msg.encoding = '32FC1'
            depth_msg.is_bigendian = False
            depth_msg.step = depth_img.shape[1] * 4
            depth_msg.data = depth_img.astype(np.float32).tobytes()
            self.depth_pub.publish(depth_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScorpioMuJoCoBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

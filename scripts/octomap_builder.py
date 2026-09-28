#!/usr/bin/env python3
"""
轻量 3D→2D 栅格建图节点

订阅 FAST-LIO2 的 /cloud_registered (PointCloud2, camera_init frame)
构建 occupancy grid 并发布标准 /map (OccupancyGrid).

TF 架构说明:
  FAST-LIO2 中 camera_init 是 SLAM 世界固定坐标系 (不是相机!),
  等价于 ROS 标准的 map frame. body 是机体坐标系, 随机器人移动.

  完整 TF 链:
    camera_init (= map, 固定) → body (动态, SLAM 位姿)
    body → lidar_link (静态外参, 由 robot_state_publisher 发布)
    body → base_link (identity, 因为 URDF 中 body = base_link)

  本节点发布:
    - /map (OccupancyGrid, frame=camera_init)
    - body → base_link 静态 TF (identity, 桥接 URDF 模型)
"""

import sys
import numpy as np


def main():
    if sys.executable != '/usr/bin/python3':
        import os
        os.execv('/usr/bin/python3', ['/usr/bin/python3'] + sys.argv)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import OccupancyGrid, MapMetaData
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import StaticTransformBroadcaster

    class OctoMapBuilder(Node):
        def __init__(self):
            super().__init__('octomap_builder')

            # 参数
            self.declare_parameter('resolution', 0.1)
            self.declare_parameter('min_z', -0.5)
            self.declare_parameter('max_z', 3.0)
            self.declare_parameter('ground_z_max', 0.3)
            self.declare_parameter('obstacle_z_min', 0.3)
            self.declare_parameter('map_size', 200.0)
            self.declare_parameter('publish_rate', 1.0)

            self.resolution = self.get_parameter('resolution').value
            self.min_z = self.get_parameter('min_z').value
            self.max_z = self.get_parameter('max_z').value
            self.ground_z_max = self.get_parameter('ground_z_max').value
            self.obstacle_z_min = self.get_parameter('obstacle_z_min').value
            map_size = self.get_parameter('map_size').value
            publish_rate = self.get_parameter('publish_rate').value

            # 内部栅格
            self._grid = {}  # (ix, iy) → [free_count, occupied_count]
            self._offset_x = int(map_size / self.resolution) // 2
            self._offset_y = int(map_size / self.resolution) // 2

            # QoS
            sensor_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT, depth=5)
            latched_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)

            # 订阅 FAST-LIO2 输出的配准点云
            self.create_subscription(
                PointCloud2, '/cloud_registered', self._cloud_cb, sensor_qos)

            # 发布标准 /map
            self.map_pub = self.create_publisher(OccupancyGrid, '/map', latched_qos)
            self.meta_pub = self.create_publisher(MapMetaData, '/map_metadata', latched_qos)

            # TF: body → base_link (identity)
            # URDF 的 root link 是 base_footprint → base_link,
            # 但 FAST-LIO2 发布的是 camera_init → body.
            # 需要 body → base_link 让 URDF 模型挂在 body 下显示.
            # 同时 base_footprint → base_link 已由 URDF 定义.
            # 所以只需补一个 body → base_footprint (identity).
            self._static_tf = StaticTransformBroadcaster(self)
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'body'
            t.child_frame_id = 'base_footprint'
            t.transform.rotation.w = 1.0
            self._static_tf.sendTransform(t)

            # 定时发布地图
            self._timer = self.create_timer(1.0 / publish_rate, self._publish_map)

            self._frame_count = 0
            self.get_logger().info(
                f'OctoMap Builder: res={self.resolution}m, '
                f'z=[{self.min_z},{self.max_z}], '
                f'ground<={self.ground_z_max}m, obstacle>={self.obstacle_z_min}m')

        def _cloud_cb(self, msg):
            """处理一帧 /cloud_registered 点云."""
            n = msg.width * msg.height
            if n == 0:
                return

            # 解析 xyz
            pts = np.frombuffer(msg.data, dtype=np.float32,
                                count=n * (msg.point_step // 4)).reshape(n, -1)
            x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]

            # 高度过滤
            mask = (z >= self.min_z) & (z <= self.max_z)
            x, y, z = x[mask], y[mask], z[mask]
            if len(x) == 0:
                return

            # 转栅格坐标
            ix = (x / self.resolution).astype(np.int32) + self._offset_x
            iy = (y / self.resolution).astype(np.int32) + self._offset_y

            # 更新计数
            for i in range(len(ix)):
                key = (int(ix[i]), int(iy[i]))
                if key not in self._grid:
                    self._grid[key] = [0, 0]
                if z[i] < self.ground_z_max:
                    self._grid[key][0] += 1   # free (地面层)
                elif z[i] >= self.obstacle_z_min:
                    self._grid[key][1] += 1   # occupied (障碍层)

            self._frame_count += 1

        def _publish_map(self):
            """发布 OccupancyGrid /map."""
            if not self._grid:
                return

            keys = list(self._grid.keys())
            min_ix = min(k[0] for k in keys)
            max_ix = max(k[0] for k in keys)
            min_iy = min(k[1] for k in keys)
            max_iy = max(k[1] for k in keys)

            width = max_ix - min_ix + 1
            height = max_iy - min_iy + 1

            data = np.full(width * height, -1, dtype=np.int8)
            for (ix, iy), (free_cnt, occ_cnt) in self._grid.items():
                idx = (iy - min_iy) * width + (ix - min_ix)
                total = free_cnt + occ_cnt
                if total > 0:
                    prob = occ_cnt / total
                    if prob > 0.5:
                        data[idx] = 100
                    elif prob < 0.2:
                        data[idx] = 0
                    else:
                        data[idx] = int(prob * 100)

            stamp = self.get_clock().now().to_msg()

            meta = MapMetaData()
            meta.resolution = float(self.resolution)
            meta.width = int(width)
            meta.height = int(height)
            meta.origin.position.x = float((min_ix - self._offset_x) * self.resolution)
            meta.origin.position.y = float((min_iy - self._offset_y) * self.resolution)
            meta.origin.position.z = 0.0
            meta.origin.orientation.w = 1.0
            self.meta_pub.publish(meta)

            grid_msg = OccupancyGrid()
            grid_msg.header.stamp = stamp
            grid_msg.header.frame_id = 'camera_init'  # FAST-LIO2 世界系 = map
            grid_msg.info = meta
            grid_msg.data = data.tolist()
            self.map_pub.publish(grid_msg)

            if self._frame_count % 10 == 0:
                occ = int(np.sum(data == 100))
                free = int(np.sum(data == 0))
                self.get_logger().info(
                    f'Map: {width}×{height}, {occ} occ, {free} free, '
                    f'{len(self._grid)} cells, {self._frame_count} frames')

    rclpy.init()
    node = OctoMapBuilder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

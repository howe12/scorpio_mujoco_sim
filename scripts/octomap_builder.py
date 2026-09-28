#!/usr/bin/env python3
"""
轻量 OctoMap / 3D→2D 栅格建图节点

订阅 FAST-LIO2 的 /cloud_registered (PointCloud2, camera_init frame)
构建 3D occupancy grid 并发布:
  - /map           nav_msgs/OccupancyGrid  (2D 投影, 用于导航/RViz)
  - /map_metadata  nav_msgs/MapMetaData
  - /octomap_full  octomap_msgs/OctoMap   (完整 3D 地图, 可选)

用法:
    ros2 run scorpio_mujoco_sim octomap_builder.py
"""

import sys
import numpy as np
import struct


def main():
    if sys.executable != '/usr/bin/python3':
        import os
        os.execv('/usr/bin/python3', ['/usr/bin/python3'] + sys.argv)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import OccupancyGrid, MapMetaData
    from geometry_msgs.msg import Pose
    from std_msgs.msg import Header

    class OctoMapBuilder(Node):
        def __init__(self):
            super().__init__('octomap_builder')

            # 参数
            self.declare_parameter('resolution', 0.1)       # m/cell
            self.declare_parameter('min_z', -0.5)           # 最低高度 (过滤地面以下)
            self.declare_parameter('max_z', 3.0)            # 最高高度
            self.declare_parameter('ground_z_min', -0.5)    # 地面层下限
            self.declare_parameter('ground_z_max', 0.3)     # 地面层上限 (此范围内视为地面)
            self.declare_parameter('obstacle_z_min', 0.3)   # 障碍物下限
            self.declare_parameter('map_size', 200.0)       # 初始地图大小 (m)
            self.declare_parameter('publish_rate', 1.0)     # Hz

            self.resolution = self.get_parameter('resolution').value
            self.min_z = self.get_parameter('min_z').value
            self.max_z = self.get_parameter('max_z').value
            self.ground_z_min = self.get_parameter('ground_z_min').value
            self.ground_z_max = self.get_parameter('ground_z_max').value
            self.obstacle_z_min = self.get_parameter('obstacle_z_min').value
            map_size = self.get_parameter('map_size').value
            publish_rate = self.get_parameter('publish_rate').value

            # 内部栅格 (动态扩展)
            self._grid = {}  # (ix, iy) → {'free': count, 'occupied': count}
            self._origin_x = 0.0
            self._origin_y = 0.0
            self._cells_x = int(map_size / self.resolution)
            self._cells_y = int(map_size / self.resolution)
            self._offset_x = self._cells_x // 2  # 原点在中心
            self._offset_y = self._cells_y // 2

            # QoS
            sensor_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                depth=5
            )
            latched_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=1
            )

            # 订阅
            self.create_subscription(PointCloud2, '/cloud_registered',
                                     self._cloud_cb, sensor_qos)

            # 发布
            self.map_pub = self.create_publisher(OccupancyGrid, '/map', latched_qos)
            self.meta_pub = self.create_publisher(MapMetaData, '/map_metadata', latched_qos)

            # 定时发布
            self._timer = self.create_timer(1.0 / publish_rate, self._publish_map)

            self._frame_count = 0
            self.get_logger().info(
                f'OctoMap Builder started: res={self.resolution}m, '
                f'z=[{self.min_z},{self.max_z}], ground=[{self.ground_z_min},{self.ground_z_max}]')

        def _cloud_cb(self, msg):
            """处理一帧点云."""
            n = msg.width * msg.height
            if n == 0:
                return

            # 解析 xyz (只取前 12 bytes per point)
            pts = np.frombuffer(msg.data, dtype=np.float32,
                                count=n * (msg.point_step // 4)).reshape(n, -1)
            x = pts[:, 0]
            y = pts[:, 1]
            z = pts[:, 2]

            # 高度过滤
            mask = (z >= self.min_z) & (z <= self.max_z)
            x, y, z = x[mask], y[mask], z[mask]

            if len(x) == 0:
                return

            # 转栅格坐标
            ix = (x / self.resolution).astype(np.int32) + self._offset_x
            iy = (y / self.resolution).astype(np.int32) + self._offset_y

            # 更新栅格
            for i in range(len(ix)):
                key = (int(ix[i]), int(iy[i]))
                if key not in self._grid:
                    self._grid[key] = [0, 0]  # [free, occupied]

                if z[i] < self.ground_z_max:
                    self._grid[key][0] += 1  # free (ground level)
                elif z[i] >= self.obstacle_z_min:
                    self._grid[key][1] += 1  # occupied (above ground)

            self._frame_count += 1

        def _publish_map(self):
            """发布 OccupancyGrid."""
            if not self._grid:
                return

            # 找到有数据的边界
            keys = list(self._grid.keys())
            min_ix = min(k[0] for k in keys)
            max_ix = max(k[0] for k in keys)
            min_iy = min(k[1] for k in keys)
            max_iy = max(k[1] for k in keys)

            width = max_ix - min_ix + 1
            height = max_iy - min_iy + 1

            # 构建 occupancy data (-1=unknown, 0=free, 100=occupied)
            data = np.full(width * height, -1, dtype=np.int8)

            for (ix, iy), (free_cnt, occ_cnt) in self._grid.items():
                px = ix - min_ix
                py = iy - min_iy
                idx = py * width + px
                total = free_cnt + occ_cnt
                if total > 0:
                    occ_prob = occ_cnt / total
                    if occ_prob > 0.5:
                        data[idx] = 100  # occupied
                    elif occ_prob < 0.2:
                        data[idx] = 0    # free
                    else:
                        data[idx] = int(occ_prob * 100)

            # 构建消息
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
            grid_msg.header.frame_id = 'camera_init'
            grid_msg.info = meta
            grid_msg.data = data.tolist()
            self.map_pub.publish(grid_msg)

            if self._frame_count % 10 == 0:
                occ_cells = np.sum(data == 100)
                free_cells = np.sum(data == 0)
                self.get_logger().info(
                    f'Map: {width}×{height} cells, '
                    f'{occ_cells} occupied, {free_cells} free, '
                    f'{len(self._grid)} total, {self._frame_count} frames')

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

#!/usr/bin/env python3
"""
FAST-LIO2 端到端测试: 启动仿真 + FAST-LIO2, 验证 3D 建图输出.
无需 GUI, 自动驱动机器人移动.
"""
import sys
import os
import time
import subprocess
import signal

def main():
    # 确保用系统 python
    if sys.executable != '/usr/bin/python3':
        os.execv('/usr/bin/python3', ['/usr/bin/python3'] + sys.argv)

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2, Imu
    from nav_msgs.msg import Path, Odometry
    from geometry_msgs.msg import Twist

    rclpy.init()
    node = rclpy.create_node('fastlio_test')

    # 统计数据
    stats = {
        'lidar_count': 0,
        'imu_count': 0,
        'cloud_registered_count': 0,
        'laser_map_count': 0,
        'path_count': 0,
        'odom_count': 0,
        'lidar_points': 0,
        'map_points': 0,
    }

    def lidar_cb(msg):
        stats['lidar_count'] += 1
        stats['lidar_points'] = msg.width * msg.height

    def imu_cb(msg):
        stats['imu_count'] += 1

    def cloud_reg_cb(msg):
        stats['cloud_registered_count'] += 1

    def map_cb(msg):
        stats['laser_map_count'] += 1
        stats['map_points'] = msg.width * msg.height

    def path_cb(msg):
        stats['path_count'] += 1

    def odom_cb(msg):
        stats['odom_count'] += 1

    node.create_subscription(PointCloud2, '/livox/lidar', lidar_cb, 10)
    node.create_subscription(Imu, '/imu/data', imu_cb, 10)
    node.create_subscription(PointCloud2, '/cloud_registered', cloud_reg_cb, 10)
    node.create_subscription(PointCloud2, '/Laser_map', map_cb, 10)
    node.create_subscription(Path, '/path', path_cb, 10)
    node.create_subscription(Odometry, '/Odometry', odom_cb, 10)

    # 发布 cmd_vel 驱动机器人移动 (FAST-LIO2 需要运动激励)
    vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)

    print("=" * 60)
    print("  FAST-LIO2 端到端测试")
    print("=" * 60)
    print(f"  等待数据...")

    start_time = time.time()
    test_duration = 30  # 秒
    move_pattern = [
        (0.2, 0.0, 5),    # 前进 5s
        (0.2, 0.3, 3),    # 左转 3s
        (0.2, -0.3, 3),   # 右转 3s
        (0.2, 0.0, 5),    # 前进 5s
        (0.0, 0.5, 3),    # 原地旋转 3s
        (0.2, 0.0, 5),    # 前进 5s
        (-0.1, 0.0, 3),   # 后退 3s
        (0.2, 0.4, 3),    # 左前 3s
    ]

    pattern_idx = 0
    pattern_start = time.time()

    while time.time() - start_time < test_duration:
        elapsed = time.time() - start_time

        # 按模式发布速度
        if pattern_idx < len(move_pattern):
            v, w, dur = move_pattern[pattern_idx]
            if time.time() - pattern_start > dur:
                pattern_idx += 1
                pattern_start = time.time()
            else:
                twist = Twist()
                twist.linear.x = v
                twist.angular.z = w
                vel_pub.publish(twist)

        rclpy.spin_once(node, timeout_sec=0.1)

        # 每 5 秒打印一次状态
        if int(elapsed) % 5 == 0 and int(elapsed) > 0:
            print(f"\n  [{elapsed:.0f}s] LiDAR:{stats['lidar_count']}({stats['lidar_points']}pts) "
                  f"IMU:{stats['imu_count']} "
                  f"Registered:{stats['cloud_registered_count']} "
                  f"Map:{stats['laser_map_count']}({stats['map_points']}pts) "
                  f"Path:{stats['path_count']} "
                  f"Odom:{stats['odom_count']}")

    # 停止机器人
    vel_pub.publish(Twist())
    time.sleep(1)

    print("\n" + "=" * 60)
    print("  测试结果")
    print("=" * 60)
    print(f"  /livox/lidar:       {stats['lidar_count']} msgs, {stats['lidar_points']} pts/msg")
    print(f"  /imu/data:          {stats['imu_count']} msgs")
    print(f"  /cloud_registered:  {stats['cloud_registered_count']} msgs")
    print(f"  /Laser_map:         {stats['laser_map_count']} msgs, {stats['map_points']} pts")
    print(f"  /path:              {stats['path_count']} msgs")
    print(f"  /Odometry:          {stats['odom_count']} msgs")

    # 判定
    ok = True
    if stats['lidar_count'] == 0:
        print("\n  ❌ FAIL: 没有收到 LiDAR 数据")
        ok = False
    if stats['imu_count'] == 0:
        print("\n  ❌ FAIL: 没有收到 IMU 数据")
        ok = False
    if stats['cloud_registered_count'] == 0:
        print("\n  ❌ FAIL: FAST-LIO2 没有输出 /cloud_registered")
        ok = False
    if stats['laser_map_count'] == 0:
        print("\n  ❌ FAIL: FAST-LIO2 没有输出 /Laser_map")
        ok = False
    if stats['path_count'] == 0:
        print("\n  ⚠ WARNING: 没有 /path 输出")

    if ok:
        print("\n  ✅ PASS: FAST-LIO2 3D 建图正常!")
    else:
        print("\n  ❌ FAIL: 请检查上方错误")

    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1

if __name__ == '__main__':
    sys.exit(main())

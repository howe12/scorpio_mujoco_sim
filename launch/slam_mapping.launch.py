#!/usr/bin/env python3
"""
Launch file: MuJoCo 仿真 + SLAM Toolbox + RViz2 一体化启动

一键启动完整的建图系统:
  1. MuJoCo 仿真 (带 GUI 窗口 + 键盘控制 + ROS2 话题发布)
  2. SLAM Toolbox (2D 建图)
  3. RViz2 (可视化)

使用方法:
  ros2 launch scorpio_mujoco_sim slam_mapping.launch.py terrain:=plaza
  ros2 launch scorpio_mujoco_sim slam_mapping.launch.py terrain:=plaza sim:=false
  ros2 launch scorpio_mujoco_sim slam_mapping.launch.py terrain:=plaza rviz:=false

注意:
  仿真节点必须使用系统 Python (/usr/bin/python3), 因为 conda 的
  libstdc++.so.6 版本过旧, 与 ROS2 rclpy 不兼容。
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('scorpio_mujoco_sim')

    # ---- 参数 ----
    terrain_arg = DeclareLaunchArgument(
        'terrain', default_value='plaza',
        description='场景: plaza, mountain, forest, snowy_mountain, pump_track')

    sim_arg = DeclareLaunchArgument(
        'sim', default_value='true',
        description='是否启动 MuJoCo 仿真 (false = 只启动 SLAM + RViz)')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='是否启动 RViz2')

    images_arg = DeclareLaunchArgument(
        'publish_images', default_value='false',
        description='是否发布相机图像 (较慢)')

    # ---- 1. MuJoCo 仿真 (GUI + 键盘控制 + ROS2 发布) ----
    # 使用系统 python3, 避免 conda 的 libstdc++ 冲突
    sim_node = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('sim')),
        cmd=[
            '/usr/bin/python3',
            PathJoinSubstitution([pkg_share, 'scripts', 'run_seb_naver_sim.py']),
            '--terrain', LaunchConfiguration('terrain'),
            '--ros2',
        ],
        output='screen',
        name='mujoco_sim',
    )

    # ---- 2. SLAM Toolbox ----
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': False,          # MuJoCo 用真实时钟
            'solver_plugin': 'solver_plugins::CeresSolver',
            'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
            'ceres_preconditioner': 'SCHUR_JACOBI',
            'ceres_trust_strategy': 'LEVENBERG_MARQUARDT',
            # 里程计/建图触发阈值 (越小越灵敏)
            'minimum_travel_distance': 0.2,
            'minimum_travel_heading': 0.2,
            'scan_buffer_size': 20,
            'scan_buffer_maximum_scan_distance': 16.0,
            'do_loop_closing': True,
            'loop_search_maximum_distance': 3.0,
            # 地图
            'resolution': 0.05,
            'map_update_interval': 0.5,
            # 激光范围 (与 YDLidar G6 一致, 消除启动警告)
            'min_laser_range': 0.12,
            'max_laser_range': 16.0,
            'use_scan_matching': True,
            'use_scan_barycenter': True,
        }],
        remappings=[('/scan', '/scan'), ('/odom', '/odom')],
    )

    # ---- 3. RViz2 ----
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'config', 'slam.rviz'])],
    )

    return LaunchDescription([
        terrain_arg,
        sim_arg,
        rviz_arg,
        images_arg,
        sim_node,
        slam_node,
        rviz_node,
    ])

"""
FAST-LIO2 3D 建图启动文件

集成: MuJoCo 仿真 + FAST-LIO2 (Livox Mid-360) + RViz2

用法:
    ros2 launch scorpio_mujoco_sim fastlio_mapping.launch.py terrain:=plaza
    ros2 launch scorpio_mujoco_sim fastlio_mapping.launch.py terrain:=mountain sim:=false  # 仅 SLAM

前置条件:
    1. FAST-LIO2 已编译安装到 ROS2 workspace:
       cd ~/fastlio_ws/src
       git clone https://github.com/hku-mars/FAST_LIO.git -b ros2
       git clone https://github.com/Livox-SDK/livox_ros_driver2.git
       cd .. && colcon build --symlink-install
    2. source ~/fastlio_ws/install/setup.bash
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('scorpio_mujoco_sim')

    terrain = LaunchConfiguration('terrain')
    use_sim = LaunchConfiguration('sim')
    use_rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument('terrain', default_value='plaza',
                              description='地形场景'),
        DeclareLaunchArgument('sim', default_value='true',
                              description='是否启动 MuJoCo 仿真'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='是否启动 RViz2'),

        # --- MuJoCo 仿真 (带 ROS2 + 3D 点云) ---
        ExecuteProcess(
            cmd=[
                '/usr/bin/python3',
                PathJoinSubstitution([pkg_share, 'scripts', 'run_seb_naver_sim.py']),
                '--terrain', terrain,
                '--ros2',
            ],
            name='scorpio_mujoco_sim',
            output='screen',
            condition=IfCondition(use_sim),
        ),

        # --- FAST-LIO2 ---
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='fastlio_mapping',
            parameters=[
                PathJoinSubstitution([pkg_share, 'config', 'fastlio_mid360.yaml']),
            ],
            output='screen',
            remappings=[],
        ),

        # --- 静态 TF: base_link → IMU_link ---
        # FAST-LIO2 需要 IMU_link frame; 我们的 IMU 在 base_link 上
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu_tf',
            arguments=['0.16', '0.04', '0.175', '0', '0', '0',
                       'base_link', 'IMU_link'],
        ),

        # --- RViz2 ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_fastlio',
            arguments=['-d', PathJoinSubstitution([pkg_share, 'config', 'fastlio.rviz'])],
            condition=IfCondition(use_rviz),
        ),
    ])

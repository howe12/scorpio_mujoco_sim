"""
FAST-LIO2 3D 建图启动文件

集成: MuJoCo 仿真 + FAST-LIO2 + RViz2

用法:
    ros2 launch scorpio_mujoco_sim fastlio_mapping.launch.py terrain:=plaza
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('scorpio_mujoco_sim')

    terrain = LaunchConfiguration('terrain')
    use_sim = LaunchConfiguration('sim')
    use_rviz = LaunchConfiguration('rviz')

    urdf_path = PathJoinSubstitution([pkg_share, 'models', 'scorpio_rviz.urdf'])

    return LaunchDescription([
        DeclareLaunchArgument('terrain', default_value='plaza'),
        DeclareLaunchArgument('sim', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),

        # --- MuJoCo 仿真 (GUI + ROS2) ---
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

        # --- robot_state_publisher (URDF → /robot_description + TF) ---
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(Command(['cat ', urdf_path]), value_type=str),
            }],
            output='screen',
        ),

        # --- FAST-LIO2 ---
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='laser_mapping',
            parameters=[{
                'common.lid_topic': '/livox/lidar',
                'common.imu_topic': '/imu/data',
                'common.time_sync_en': False,
                'common.time_offset_lidar_to_imu': 0.0,
                'preprocess.lidar_type': 2,
                'preprocess.scan_line': 32,
                'preprocess.timestamp_unit': 0,
                'preprocess.blind': 0.5,
                # IMU 协方差: 仿真 IMU 有数值噪声, 适当提高让算法更信任 LiDAR
                'mapping.acc_cov': 3.5,
                'mapping.gyr_cov': 0.7,
                'mapping.b_acc_cov': 0.001,
                'mapping.b_gyr_cov': 0.001,
                'mapping.fov_degree': 360.0,
                'mapping.det_range': 40.0,
                # 外参在线估计关闭: 仿真中 LiDAR-IMU 外参已知且固定,
                # 开启会导致 forest 等复杂场景中估计漂移→重影
                'mapping.extrinsic_est_en': False,
                'mapping.extrinsic_T': [0.0, 0.0, 0.0],
                'mapping.extrinsic_R': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                'publish.path_en': True,
                'publish.map_en': True,
                'publish.scan_publish_en': True,
                'publish.dense_publish_en': True,
                'publish.scan_bodyframe_pub_en': False,
                # 轮式里程计加速度修正: 打滑/陡坡时弊大于利, 默认关闭
                # (纯 LiDAR-IMU; /wheel_odom 仍发布, 仅作观测)
                'wheel_odom.enable': False,
                # point_filter_num=1: 保留更多点, 提高匹配精度减少重影
                'point_filter_num': 1,
                # filter_size 减小: 更精细的地图, 减少下采样导致的信息丢失
                'filter_size_surf': 0.3,
                'filter_size_map': 0.3,
                'cube_side_length': 200.0,
                # max_iteration 增加: 更多 ICP 迭代提高配准精度
                'max_iteration': 8,
                'feature_extract_enable': False,
                # 保存: Ctrl+C 退出时自动保存 PCD 到此路径
                'map_file_path': '/develop/scorpio_mujoco_sim_ws/map.pcd',
                'pcd_save.pcd_save_en': True,
                'pcd_save.interval': -1,
            }],
            output='screen',
        ),

        # --- body → base_footprint TF (桥接 URDF 到 FAST-LIO2 body frame) ---
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='body_to_base_tf',
            arguments=['0', '0', '0', '0', '0', '0',
                       'body', 'base_footprint'],
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

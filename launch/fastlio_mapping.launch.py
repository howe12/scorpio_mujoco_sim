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
                'mapping.acc_cov': 0.1,
                'mapping.gyr_cov': 0.1,
                'mapping.b_acc_cov': 0.0001,
                'mapping.b_gyr_cov': 0.0001,
                'mapping.fov_degree': 360.0,
                'mapping.det_range': 40.0,
                'mapping.extrinsic_est_en': True,
                'mapping.extrinsic_T': [0.0, 0.0, 0.0],
                'mapping.extrinsic_R': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                'publish.path_en': True,
                'publish.map_en': True,
                'publish.scan_publish_en': True,
                'publish.dense_publish_en': True,
                'publish.scan_bodyframe_pub_en': False,
                'point_filter_num': 2,
                'filter_size_surf': 0.5,
                'filter_size_map': 0.5,
                'cube_side_length': 200.0,
                'max_iteration': 4,
                'feature_extract_enable': False,
                'pcd_save.pcd_save_en': True,
                'pcd_save.interval': -1,
            }],
            output='screen',
        ),

        # --- OctoMap Builder (3D→2D 栅格地图) ---
        Node(
            package='scorpio_mujoco_sim',
            executable='octomap_builder.py',
            name='octomap_builder',
            parameters=[{
                'resolution': 0.1,
                'min_z': -0.5,
                'max_z': 3.0,
                'ground_z_max': 0.3,
                'obstacle_z_min': 0.3,
                'publish_rate': 1.0,
            }],
            output='screen',
        ),

        # --- 静态 TF: base_link → IMU_link ---
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

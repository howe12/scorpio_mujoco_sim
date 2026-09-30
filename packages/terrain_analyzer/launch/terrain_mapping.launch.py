"""
SEB-Naver Terrain Mapping Launch File (ROS2)

Integrates: MuJoCo sim + FAST-LIO2 + TerrainAnalyzer + RViz2

Usage:
    ros2 launch terrain_analyzer terrain_mapping.launch.py terrain:=plaza
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    scorpio_share = FindPackageShare('scorpio_mujoco_sim')
    terrain_share = FindPackageShare('terrain_analyzer')

    terrain = LaunchConfiguration('terrain')
    use_sim = LaunchConfiguration('sim')
    use_rviz = LaunchConfiguration('rviz')
    sim_gui = LaunchConfiguration('sim_gui')

    urdf_path = PathJoinSubstitution([scorpio_share, 'models', 'scorpio_rviz.urdf'])

    # sim_gui=true → GUI 模式 (MuJoCo 窗口), 否则 headless (纯 RViz 控制)
    sim_mode = PythonExpression(["'--ros2' if '", sim_gui, "' == 'true' else '--headless-ros2'"])

    return LaunchDescription([
        DeclareLaunchArgument('terrain', default_value='plaza'),
        DeclareLaunchArgument('sim', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('sim_gui', default_value='false',
                              description='true → 弹出 MuJoCo 窗口; false → headless 仅 RViz'),

        # --- MuJoCo Simulation ---
        ExecuteProcess(
            cmd=[
                '/usr/bin/python3',
                PathJoinSubstitution([scorpio_share, 'scripts', 'run_seb_naver_sim.py']),
                '--terrain', terrain,
                sim_mode,
                '--no-tf-odom',
            ],
            name='scorpio_mujoco_sim',
            output='log',
            condition=IfCondition(use_sim),
        ),

        # --- robot_state_publisher ---
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
                'mapping.acc_cov': 3.5,
                'mapping.gyr_cov': 0.7,
                'mapping.b_acc_cov': 0.001,
                'mapping.b_gyr_cov': 0.001,
                'mapping.fov_degree': 360.0,
                'mapping.det_range': 40.0,
                'mapping.extrinsic_est_en': False,
                'mapping.extrinsic_T': [0.0, 0.0, 0.0],
                'mapping.extrinsic_R': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                'publish.path_en': True,
                'publish.map_en': True,
                'publish.scan_publish_en': True,
                'publish.dense_publish_en': True,
                'publish.scan_bodyframe_pub_en': False,
                'wheel_odom.enable': True,
                'point_filter_num': 1,
                'filter_size_surf': 0.3,
                'filter_size_map': 0.3,
                'cube_side_length': 200.0,
                'max_iteration': 8,
                'feature_extract_enable': False,
                'map_file_path': '/develop/scorpio_mujoco_sim_ws/map.pcd',
                'pcd_save.pcd_save_en': True,
                'pcd_save.interval': -1,
            }],
            output='screen',
        ),

        # --- Terrain Analyzer (SEB-Naver core) ---
        Node(
            package='terrain_analyzer',
            executable='terrain_analyzer_node',
            name='terrain_analyzer',
            parameters=[PathJoinSubstitution([terrain_share, 'terrain_analyzer.yaml'])],
            output='screen',
        ),

        # --- body → base_footprint TF ---
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='body_to_base_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'body', 'base_footprint'],
        ),

        # --- RViz2 ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_terrain',
            arguments=['-d', PathJoinSubstitution([scorpio_share, 'config', 'fastlio.rviz'])],
            condition=IfCondition(use_rviz),
        ),
    ])

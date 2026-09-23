"""
Launch file for Scorpio MuJoCo simulation with ROS2 bridge.

Usage:
    ros2 launch scorpio_mujoco_sim scorpio_mujoco.launch.py
    ros2 launch scorpio_mujoco_sim scorpio_mujoco.launch.py headless:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('scorpio_mujoco_sim')
    
    model_path = os.path.join(pkg_dir, 'models', 'scorpio.xml')
    
    # Launch arguments
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run MuJoCo in headless mode (no GUI)'
    )
    
    sim_rate_arg = DeclareLaunchArgument(
        'sim_rate', default_value='500.0',
        description='MuJoCo physics simulation rate (Hz)'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate', default_value='50.0',
        description='ROS2 topic publish rate (Hz)'
    )
    
    # MuJoCo bridge node
    mujoco_bridge = Node(
        package='scorpio_mujoco_sim',
        executable='scorpio_mujoco_bridge.py',
        name='scorpio_mujoco_bridge',
        output='screen',
        parameters=[{
            'model_path': model_path,
            'sim_rate': LaunchConfiguration('sim_rate'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'use_headless': LaunchConfiguration('headless'),
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/odom', '/odom'),
            ('/scan', '/scan'),
            ('/imu/data', '/imu/data'),
        ]
    )
    
    # Robot state publisher (for static transforms from URDF)
    # Optional: can be used alongside MuJoCo TF for sensor frame transforms
    
    return LaunchDescription([
        headless_arg,
        sim_rate_arg,
        publish_rate_arg,
        mujoco_bridge,
    ])

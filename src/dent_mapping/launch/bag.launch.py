"""
launch/bag.launch.py
--------------------
Starts dent_mapping_node in "bag" (offline) mode using image paths
from a config YAML — no RealSense hardware required.

Usage:
    ros2 launch dent_mapping bag.launch.py config:=/path/to/bag_config.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    config_arg = DeclareLaunchArgument(
        'config',
        default_value='',
        description='Path to YAML config file with bag_rgb_paths / bag_depth_paths')

    dent_dir_arg = DeclareLaunchArgument(
        'dent_dir',
        default_value='~/dent',
        description='Directory containing dent reference images')

    voxel_size_arg = DeclareLaunchArgument(
        'voxel_size',
        default_value='0.01')

    dent_mapping_node = Node(
        package='dent_mapping',
        executable='dent_mapping_node',
        name='dent_mapping_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config'),   # YAML overrides
            {
                'source':     'bag',
                'dent_dir':   LaunchConfiguration('dent_dir'),
                'voxel_size': LaunchConfiguration('voxel_size'),
                'map_frame':  'map',
            }
        ]
    )

    return LaunchDescription([
        config_arg,
        dent_dir_arg,
        voxel_size_arg,
        dent_mapping_node,
    ])

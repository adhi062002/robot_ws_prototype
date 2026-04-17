from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ------------------- CAMERA -------------------
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ascamera'),
                'launch',
                'hp60c.launch.py'
            )
        )
    )

    # ------------------- IMU -------------------
    imu_node = Node(
        package='ugv_hardware',
        executable='hardware_interface',
        name='imu_node',
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    # ------------------- VINS -------------------
    vins_node = Node(
        package='vins',
        executable='vins_node',
        name='vins_node',
        output='screen',
        arguments=[
            '/home/nvidia/robot_ws/src/VINS-Fusion-ROS2-humble-arm/config/hp60c/hp60c_mono_imu_kaliber.yaml'
        ],
        respawn=True,
        respawn_delay=2.0
    )

    # ------------------- COMPRESS -------------------
    compress_node = Node(
        package='ugv_teleop',
        executable='compress',
        name='compress_node',
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    # ------------------- SEQUENCE -------------------
    return LaunchDescription([

        # 1. Start camera first
        camera_launch,

        # 2. Start IMU after 3 sec
        TimerAction(
            period=3.0,
            actions=[imu_node]
        ),

        # 3. Start VINS after 6 sec
        TimerAction(
            period=6.0,
            actions=[vins_node]
        ),

        # 4. Start compress after 8 sec
        TimerAction(
            period=8.0,
            actions=[compress_node]
        ),
    ])

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # --- Get Package Directories ---
    ugv_description_pkg = get_package_share_directory('ugv_description')

    # --- Process URDF ---
    urdf_file_path = os.path.join(ugv_description_pkg, 'urdf', 'ugv_bot.urdf.xacro')
    robot_description_raw = xacro.process_file(urdf_file_path).toxml()

    pkg_share = get_package_share_directory('ugv_teleop')

    vio_params = os.path.join(pkg_share, 'config', 'vio_params.yaml')
    slam_params = os.path.join(pkg_share, 'config', 'rtabmap_params.yaml')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
    )

    vio_node_A = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[vio_params],
        remappings=[
            ('rgb/image', '/ascamera_hp60c/camera_publisher/rgb0/image'),
            ('depth/image', '/ascamera_hp60c/camera_publisher/depth0/image_raw'),
            ('rgb/camera_info', '/ascamera_hp60c/camera_publisher/rgb0/camera_info'),
            ('imu/data_raw', '/imu/data_raw'),
            ('odom', '/vio/odom')
        ]
    )

    slam_node_A = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        arguments=['--delete_db_on_start'],
        parameters=[slam_params],
        remappings=[
            ('rgb/image', '/ascamera_hp60c/camera_publisher/rgb0/image'),
            ('depth/image', '/ascamera_hp60c/camera_publisher/depth0/image_raw'),
            ('rgb/camera_info', '/ascamera_hp60c/camera_publisher/rgb0/camera_info'),
            ('odom', '/vio/odom')
        ]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        vio_node_A,
        slam_node_A
    ])

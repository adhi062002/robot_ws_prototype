import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import TimerAction
import xacro


def generate_launch_description():

    ugv_description_pkg = get_package_share_directory('ugv_description')
    urdf_file_path = os.path.join(ugv_description_pkg, 'urdf', 'ugv_bot.urdf.xacro')
    robot_description_raw = xacro.process_file(urdf_file_path).toxml()
    ascamera_pkg = get_package_share_directory('ascamera') 

    # --- Nodes ---
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ascamera_pkg, 'launch', 'hp60c.launch.py')
        )
    )

    hardware_interface_node = Node(
        package='ugv_teleop',
        executable='hardware_interface',
        name='hardware_interface_node',
        output='screen'
    )

    forward_kinematics_node = Node(
        package='ugv_teleop',
        executable='forward_kinematics',
        name='forward_kinematics_node'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
    )

    # smooth_teleop_node = Node(
    #     package='ugv_teleop',
    #     executable='smooth_teleop',  # Your improved teleop script entry point
    #     name='smooth_teleop_node',
    #     output='screen'
    # )
     # 2a. Image Compression Nodes

    vio_node_A = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[{'frame_id':'base_footprint', 'odom_frame_id':'odom_vio', 'publish_tf':True, 'approx_sync':True, 'use_sim_time':False, 'Vis/ImuAsInput':True, 'Odom/Strategy':'0', 'Vis/MaxFeatures':'1500', 'Vis/MinInliers':'20', 'OdomF2M/MaxSize':'3000', 'Odom/ResetCountdown':'5', 'Odom/Holonomic':'True', 'Reg/Force3DoF': 'True'}],
        remappings=[('rgb/image','/ascamera_hp60c/camera_publisher/rgb0/image'), ('depth/image','/ascamera_hp60c/camera_publisher/depth0/image_raw'), ('rgb/camera_info','/ascamera_hp60c/camera_publisher/rgb0/camera_info'), ('imu/data','/imu/data_raw'), ('odom','/vio/odom')]
    )

    slam_node_A = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        #arguments=['--delete_db_on_start'],
        parameters=[{'frame_id':'base_footprint', 'subscribe_depth':True, 'approx_sync':True, 'use_sim_time':False, 'Reg/Strategy':'0', 'Vis/MinInliers':'20', 'RGBD/ProximityBySpace':'False', 'RGBD/ProximityByTime':'True', 'wait_for_transform': 0.5}],
        remappings=[('rgb/image','/ascamera_hp60c/camera_publisher/rgb0/image'), ('depth/image','/ascamera_hp60c/camera_publisher/depth0/image_raw'), ('rgb/camera_info','/ascamera_hp60c/camera_publisher/rgb0/camera_info'), ('odom','/vio/odom')]
    )

    delayed_rtabmap_and_vio = TimerAction(
        period=5.0,  # Delay in seconds
        actions=[
            vio_node_A,
            slam_node_A
        ]
    )


    # --- Launch Description ---
    return LaunchDescription([
        camera_launch,
        hardware_interface_node,
        # smooth_teleop_node,
        # vio_node_A,
        # slam_node_A
        forward_kinematics_node,
        robot_state_publisher_node,
        delayed_rtabmap_and_vio
    ])

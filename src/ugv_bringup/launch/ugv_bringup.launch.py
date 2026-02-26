#THIS is the bringup file for the end
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # --- Launch arguments ---
    use_ekf_arg = DeclareLaunchArgument(
        'use_ekf', default_value='false',
        description='Set to "true" to run System B (EKF-Fused), "false" for System A (VIO-Only).'
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')


    # --- Get Package Directories --- 
    ugv_description_pkg = get_package_share_directory('ugv_description')
    urdf_file_path = os.path.join(ugv_description_pkg, 'urdf', 'ugv_bot.urdf.xacro')
    robot_description_raw = xacro.process_file(urdf_file_path).toxml()
    ascamera_pkg = get_package_share_directory('ascamera') 

    # --- Load config files ---

    imu_filter_config_path = os.path.join(
        get_package_share_directory('ugv_bringup'),
        'config',
        'imu_filter.yaml'
    )

    ekf_params_file = os.path.join(
        get_package_share_directory('ugv_bringup'),
        'config',
        'ekf.yaml'
    )


    # 1. Camera Node (from your teleop_function.launch.py)
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ascamera_pkg, 'launch', 'hp60c.launch.py')
        )
    )

     # 2a. Image Compression Nodes
    republish_rgb_node = Node(
        package='image_transport',
        executable='republish',
        name='republish_rgb',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in', '/ascamera_hp60c/camera_publisher/rgb0/image'),
            ('out/compressed', '/ascamera_hp60c/camera_publisher/rgb0/image/compressed')
        ]
    )

     # 2b. Image Compression Nodes
    republish_depth_node = Node(
        package='image_transport',
        executable='republish',
        name='republish_depth',
        arguments=['raw', 'compressedDepth'], 
        remappings=[
            ('in', '/ascamera_hp60c/camera_publisher/depth0/image_raw'),
            ('out/compressedDepth', '/ascamera_hp60c/camera_publisher/depth0/image_raw/compressedDepth')
        ]
    )

    # 3. Hardware Interface Node (Teensy connection)
    hardware_interface_node = Node(
        package='ugv_hardware',
        executable='hardware_interface', 
        name='hardware_interface_node'
    )

    # 4. Forward Kinematics Node (Odometry calculation)
    forward_kinematics_node = Node(
        package='ugv_hardware',
        executable='forward_kinematics',
        name='forward_kinematics_node'
    )

    # 5. Robot State Publisher Node (URDF TF tree)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': use_sim_time
        }]
    )

    # 6. The IMU Filter Node
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter', 
        output='screen',
        parameters=[imu_filter_config_path],
        remappings=[
            ('/imu/data_raw', '/imu/data_raw'),
            ('/imu/data', '/imu/data')
        ]
    )

    # 7. EKF Node (Only for System B)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_ekf')),
        parameters=[ekf_params_file, {'use_sim_time': False}]
    )

    # 8a. VIO Node for System A (VIO-Only)
    vio_node_A = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_ekf')),
        parameters=[{'frame_id':'base_footprint', 'odom_frame_id':'odom_vio', 'publish_tf':True, 'approx_sync':True, 'use_sim_time':False, 'Vis/ImuAsInput':True, 'Odom/Strategy':'0', 'Vis/MaxFeatures':'1500', 'Vis/MinInliers':'20', 'OdomF2M/MaxSize':'3000', 'Odom/ResetCountdown':'5', 'Odom/Holonomic':'True', 'Reg/Force3DoF': 'True'}],
        remappings=[('rgb/image','/ascamera_hp60c/camera_publisher/rgb0/image'), ('depth/image','/ascamera_hp60c/camera_publisher/depth0/image_raw'), ('rgb/camera_info','/ascamera_hp60c/camera_publisher/rgb0/camera_info'), ('imu/data','/imu/data'), ('odom','/vio/odom')]
    )


    # 8b. VIO Node for System B (EKF-Fused)
    vio_node_B = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_ekf')),
        parameters=[{'frame_id':'base_footprint', 'odom_frame_id':'odom_vio', 'publish_tf':False, 'approx_sync':True, 'use_sim_time':False, 'Vis/ImuAsInput':True, 'Odom/Strategy':'0', 'Vis/MaxFeatures':'1500', 'Vis/MinInliers':'20', 'OdomF2M/MaxSize':'3000', 'Odom/ResetCountdown':'5', 'Odom/Holonomic':'True', 'Vis/FeatureType': '2'}],
        remappings=[('rgb/image','/ascamera_hp60c/camera_publisher/rgb0/image'), ('depth/image','/ascamera_hp60c/camera_publisher/depth0/image_raw'), ('rgb/camera_info','/ascamera_hp60c/camera_publisher/rgb0/camera_info'), ('imu/data','/imu/data'), ('odom','/vio/odom')]
    )



    # 9a. SLAM Node for System A
    slam_node_A = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_ekf')),
        #arguments=['--delete_db_on_start'],
        parameters=[{'frame_id':'base_footprint', 'subscribe_depth':True, 'approx_sync':True, 'use_sim_time':False, 'Reg/Strategy':'0', 'Vis/MinInliers':'20', 'RGBD/ProximityBySpace':'False', 'RGBD/ProximityByTime':'True', 'wait_for_transform': 0.5}],
        remappings=[('rgb/image','/ascamera_hp60c/camera_publisher/rgb0/image'), ('depth/image','/ascamera_hp60c/camera_publisher/depth0/image_raw'), ('rgb/camera_info','/ascamera_hp60c/camera_publisher/rgb0/camera_info'), ('odom','/vio/odom')]
    )

    # 9b. SLAM Node for System B
    slam_node_B = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_ekf')),
        arguments=['--delete_db_on_start'],
        parameters=[{'frame_id':'base_footprint', 'subscribe_depth':True, 'approx_sync':True, 'use_sim_time':False, 'Reg/Strategy':'2', 'Vis/MinInliers':'20', 'RGBD/ProximityBySpace':'False', 'RGBD/ProximityByTime':'False', 'RGBD/NeighborLinkRefining': 'False', 'Optimizer/GravitySigma': '0.1', 'Reg/Strategy': '2', 'RGBD/LoopClosureReextractFeatures': 'True', 'Optimizer/Strategy': '0'}],
        remappings=[('rgb/image','/ascamera_hp60c/camera_publisher/rgb0/image'), ('depth/image','/ascamera_hp60c/camera_publisher/depth0/image_raw'), ('rgb/camera_info','/ascamera_hp60c/camera_publisher/rgb0/camera_info'), ('odom','/odometry/filtered')]
    )



    # --- Assemble the Launch Description ---
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # Add all nodes to be launched
        camera_launch,
        republish_rgb_node,
        republish_depth_node,
        hardware_interface_node,
        imu_filter_node,
        forward_kinematics_node,
        robot_state_publisher_node,
        use_ekf_arg,
        ekf_node,
        vio_node_A,
        vio_node_B,
        slam_node_A,
        slam_node_B
    ])

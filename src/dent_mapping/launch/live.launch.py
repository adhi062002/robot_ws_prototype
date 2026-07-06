"""
launch/live.launch.py  –  ROS2 Humble
Starts realsense2_camera (D415) + dent_mapping_node in live mode.

Usage:
  ros2 launch dent_mapping live.launch.py \
      yolo_model_path:=/path/to/dent_yolov8.pt \
      yolo_conf:=0.4
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── launch arguments (all strings – ROS2 launch only speaks strings) ──────
    args = [
        DeclareLaunchArgument(
            'yolo_model_path', default_value='yolov8n.pt',
            description='Path to YOLO .pt weights'),
        DeclareLaunchArgument(
            'yolo_conf', default_value='0.4',
            description='YOLO confidence threshold (float as string)'),
        DeclareLaunchArgument(
            'voxel_size', default_value='0.01'),
        DeclareLaunchArgument(
            'map_frame', default_value='map'),
    ]

    # ── RealSense D415 ─────────────────────────────────────────────────────────
    # Humble realsense2_camera uses profile strings, NOT separate w/h/fps args.
    # Format: "width,height,fps"
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch', 'rs_launch.py',
            ])
        ]),
        launch_arguments={
            'enable_color':               'true',
            'enable_depth':               'true',
            'align_depth.enable':         'true',   # depth aligned to colour
            'pointcloud.enable':          'false',  # we build our own
            'colorizer.enable':           'false',
            # profile = "width,height,fps"  (must be a mode the D415 supports)
            'rgb_camera.color_profile':   '640,480,30',
            'depth_module.depth_profile': '640,480,30',
        }.items()
    )

    # ── dent_mapping_node ──────────────────────────────────────────────────────
    # Parameters are passed as a Python dict; numeric types stay numeric here.
    # LaunchConfiguration values are strings at launch time, so float/int params
    # that need to be substituted at runtime are declared separately below.
    dent_node = Node(
        package='dent_mapping',
        executable='dent_mapping_node',
        name='dent_mapping_node',
        output='screen',
        parameters=[
            # Static typed params – safe as literals
            {
                'source':          'live',
                'yolo_class_ids':  [],        # empty = accept all YOLO classes
                'dent_min_depth':  0.1,
                'dent_max_depth':  3.5,
                'queue_size':      5,
                'camera_frame':    'camera_color_optical_frame',
            },
            # String-substitution params (LaunchConfiguration returns strings;
            # rclpy will coerce 'map_frame' as str and the others via declare)
            {
                'yolo_model_path': LaunchConfiguration('yolo_model_path'),
                'yolo_conf':       LaunchConfiguration('yolo_conf'),
                'voxel_size':      LaunchConfiguration('voxel_size'),
                'map_frame':       LaunchConfiguration('map_frame'),
            },
        ],
        remappings=[
            # realsense2_camera Humble publishes under /camera/camera/...
            ('/camera/color/image_raw',
             '/camera/camera/color/image_raw'),
            ('/camera/depth/image_rect_raw',
             '/camera/camera/aligned_depth_to_color/image_raw'),
            ('/camera/color/camera_info',
             '/camera/camera/color/camera_info'),
        ],
    )

    return LaunchDescription(args + [realsense, dent_node])

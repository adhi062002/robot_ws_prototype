# ~/ugv_ws/src/ugv_description/launch/start_calibrated_camera.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    # --- PART 1: Define paths ---
    ugv_description_share_dir = get_package_share_directory('ugv_description')
    # This is the path to YOUR calibration file.
    camera_calibration_file = os.path.join(ugv_description_share_dir, 'config', 'hp60c_orin_nano_rgb.yaml')

    # --- PART 2: The Original (Buggy) Camera Driver Launch ---
    # We will re-use the working launch file for the camera driver.
    # This launch file has the hardcoded path to the vendor's configs.
    camera_driver_node = Node(
        package='ascamera',
        executable='ascamera_node',
        namespace="ascamera_hp60c",
        name='ascamera_hp60c_publisher', # Give it a unique name
        parameters=[
            {"confiPath": '/home/nvidia/ascam_ws/src/ascamera/configurationfiles'},
            # We don't even need camera_info_url here since it's ignored
            {"usb_bus_no": -1}, {"usb_path": "null"}, {"color_pcl": False},
            {"pub_tfTree": True}, {"depth_width": 640}, {"depth_height": 480},
            {"rgb_width": 640}, {"rgb_height": 480}, {"fps": 25},
        ]
    )

    # --- PART 3: The "Fixer" Node ---
    # This node will load YOUR calibration file and publish it on the correct topic,
    # overriding the incorrect data from the main driver.
    calibrated_info_publisher = Node(
        package='image_publisher',
        executable='camerainfo_publisher',
        name='rgb_info_publisher',
        parameters=[
            {'camera_info_url': 'file://' + camera_calibration_file},
        ],
        # We remap its output topic to be exactly the same as the driver's camera_info topic.
        remappings=[
            ('/camera_info', '/ascamera_hp60c/camera_publisher/rgb0/camera_info'),
        ]
    )

    return LaunchDescription([
        camera_driver_node,
        calibrated_info_publisher
    ])

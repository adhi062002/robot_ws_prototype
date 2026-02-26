# --- Copy and paste this one last time ---

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # This part for our calibration file is correct and handled by a standard library.
    ugv_description_share_dir = get_package_share_directory('ugv_description')
    camera_info_url = 'file://' + os.path.join(ugv_description_share_dir, 'config', 'hp60c_orin_nano_rgb.yaml')

    #################################################################
    ### THE LAST RESORT ###
    # We will point confiPath directly to the SRC directory,
    # because the C++ node is buggy and seems to ignore anything else.
    confiPath = '/home/nvidia/test_ws/src/ascamera/configurationfiles'
    #################################################################

    # Define the camera node
    ascamera_node = Node(
        namespace="ascamera_hp60c",
        package='ascamera',
        executable='ascamera_node',
        respawn=True,
        output='both',
        parameters=[
            {"confiPath": confiPath},
            {'camera_info_url': camera_info_url},
            {"usb_bus_no": -1},
            {"usb_path": "null"},
            {"color_pcl": False},
            {"pub_tfTree": False},
            {"depth_width": 640},
            {"depth_height": 480},
            {"rgb_width": 640},
            {"rgb_height": 480},
            {"fps": 25},
        ],
        remappings=[]
    )

    ld = LaunchDescription()
    ld.add_action(ascamera_node)

    return ld

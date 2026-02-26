import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """
    This is the main launch file for Project Orion.
    It brings up the essential hardware drivers and the teleop node.
    """

    # --- 1. Launch the HP60C Camera Driver ---
    # We find the launch file from the 'ascamera' package and include it.
    ascam_pkg_dir = get_package_share_directory('ascamera')
    camera_launch_file = os.path.join(ascam_pkg_dir, 'launch', 'hp60c.launch.py')
    
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_file)
    )

    # --- 2. Launch the Hardware Interface Node ---
    hardware_interface_node = Node(
        package='ugv_hardware',  # The package name from your workspace
        executable='hardware_interface',    # The executable name from your setup.py
        name='hardware_interface_node', # A descriptive name for the running node
        output='screen'             # Show print/log statements in the terminal
    )

    # --- 3. Launch the Teleop Node ---
    teleop_node = Node(
        package='ugv_teleop', # The package containing your teleop script
        executable='teleop',    # The executable name from its setup.py
        name='teleop_node',
        output='screen',
        prefix='xterm -e' # This is a handy trick to open the node in a new terminal window
    )

    return LaunchDescription([
        LogInfo(msg="--- Starting Launch ---"),
        
        # Add the nodes to the launch description
        camera_launch,
        hardware_interface_node,
        teleop_node
    ])
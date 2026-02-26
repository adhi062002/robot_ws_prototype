from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Path to the display.launch.py file
    ugv_description_dir = get_package_share_directory('ugv_description')
    display_launch_path = os.path.join(ugv_description_dir, 'launch', 'display.launch.py')

    return LaunchDescription([
        # 1. Include the launch file to start RViz and robot_state_publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(display_launch_path)
        ),
        
        # 2. Start your forward kinematics node
        Node(
            package='ugv_hardware',
            executable='forward_kinematics',
            name='forward_kinematics_node'
        ),
        
        # 3. Start your teleop node
        Node(
            package='ugv_teleop', 
            executable='teleop', 
            name='teleop_node',
            prefix='xterm -e', 
            output='screen'
        )
    ])
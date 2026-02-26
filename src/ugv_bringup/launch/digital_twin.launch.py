import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # --- Declare Launch Arguments ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # --- Get Package Directories ---
    ugv_description_pkg = get_package_share_directory('ugv_description')
    ascamera_pkg = get_package_share_directory('ascamera') 

    # --- Load URDF and RViz Config ---
    urdf_file_path = os.path.join(ugv_description_pkg, 'urdf', 'ugv_bot.urdf.xacro')
    robot_description_raw = xacro.process_file(urdf_file_path).toxml()
    rviz_config_file = os.path.join(ugv_description_pkg, 'rviz', 'display_twin.rviz')


    # --- Define All Necessary Nodes ---

    # 1. Camera Node (from your teleop_function.launch.py)
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ascamera_pkg, 'launch', 'hp60c.launch.py')
        )
    )
    # 2. Hardware Interface Node (Teensy connection)
    hardware_interface_node = Node(
        package='ugv_hardware',
        executable='hardware_interface', 
        name='hardware_interface_node'
    )

    # 3. Forward Kinematics Node (Odometry calculation)
    forward_kinematics_node = Node(
        package='ugv_hardware',
        executable='forward_kinematics',
        name='forward_kinematics_node'
    )

    # 4. Robot State Publisher Node (URDF TF tree)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': use_sim_time
        }]
    )

    # 5. Teleop Node (Keyboard control)
    teleop_node = Node(
        package='ugv_teleop',
        executable='teleop', 
        name='teleop_node',
        prefix='xterm -e', 
        output='screen'
    )

    # 6. RViz2 Node (Visualization)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # --- Assemble the Launch Description ---
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # Add all nodes to be launched
        camera_launch,
        hardware_interface_node,
        forward_kinematics_node,
        robot_state_publisher_node,
        teleop_node,
        rviz_node
    ])
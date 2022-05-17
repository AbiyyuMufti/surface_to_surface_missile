import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('exocet_mm40b3_description'))
    xacro_file = os.path.join(pkg_path,'urdf','exocet_mm40b3.urdf.xacro')
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'exocet_mm40b3.rviz')
    robot_description_config = xacro.process_file(xacro_file)
   
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        joint_state_publisher_node,
        node_robot_state_publisher,
        rviz_node,
    ])
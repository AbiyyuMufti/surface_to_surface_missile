import os

from yaml import load
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit
import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('exocet_mm40b3_description'))
    xacro_file = os.path.join(pkg_path,'urdf','exocet_mm40b3.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
   
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    # launch_arguments={'paused':'true', 'use_sim_time': 'true'}.items()
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')

    # pause gazebo simulation directly after start
    pause_gazebo = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/pause_physics', 'std_srvs/srv/Empty'],
        output='screen')
    
    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )
    spawn_forward_position_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["forward_position_controller"],
        output="screen",
    )
    spawn_position_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["position_trajectory_controller"],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #     target_action=spawn_entity,
        #     on_exit=[pause_gazebo],
        #     )
        # ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[spawn_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_joint_state_broadcaster,
                on_exit=[spawn_position_trajectory_controller],
            )
        ), 
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_position_trajectory_controller,
                on_exit=[spawn_forward_position_controller],
            )
        ),       
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
    ])
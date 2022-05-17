import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions
from launch_ros.actions import Node


def generate_launch_description():
    joy_config = launch.substitutions.LaunchConfiguration('joy_config')
    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')
    config_filepath = launch.substitutions.LaunchConfiguration('config_filepath')

    node_joystick = Node(
        package='joy_missile_controller',
        executable='joy_missile_controller',
        output='screen',
        # parameters=[params]
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('joy_config', default_value='fins_motion'),
        launch.actions.DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        launch.actions.DeclareLaunchArgument('config_filepath', default_value=[
            launch.substitutions.TextSubstitution(text=os.path.join(
                get_package_share_directory('joy_missile_controller'), 'config', '')),
            joy_config, launch.substitutions.TextSubstitution(text='.config.yaml')]),

        launch_ros.actions.Node(
            package='joy', node_executable='joy_node', name='joy_node',
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }]),
        # launch_ros.actions.Node(
        #     package='joy_missile_controller', node_executable='joy_missile_controller',
        #     name='joy_missile_controller', parameters=[config_filepath]),
        node_joystick
    ])
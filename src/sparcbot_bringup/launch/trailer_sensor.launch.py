"""Start only the trailer angle sensor (AS5600), for testing.

    ros2 launch sparcbot_bringup trailer_sensor.launch.py

Then watch it from a second terminal:

    ros2 topic echo /trailer_angle

Parameters come from config/sparcbot.yaml (trailer_sensor section).
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = str(
        Path(get_package_share_directory('sparcbot_bringup')) / 'config' / 'sparcbot.yaml')

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='SPARCBot parameter file'),

        Node(
            package='sparcbot_nodes',
            executable='trailer_sensor_node',
            name='trailer_sensor',
            parameters=[params_file],
            output='screen',
            emulate_tty=True),
    ])

"""Start trailer_lqr (keeps the trailer under control while you drive).

    ros2 launch sparcbot_bringup robot.launch.py trailer_sensor:=true                    # terminal 1
    ros2 launch sparcbot_bringup trailer_lqr.launch.py                                   # terminal 2
    ros2 run sparcbot_teleop trailer_teleop --ros-args -r cmd_vel:=cmd_vel_teleop        # terminal 3

The teleop must publish /cmd_vel_teleop (the remap above), not /cmd_vel: trailer_lqr sits
between the teleop and the wheel controller. All its settings are at the top of
sparcbot_control/trailer_lqr.py; it does not read sparcbot.yaml.
Do not run another /cmd_vel publisher at the same time.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sparcbot_control',
            executable='trailer_lqr',
            name='trailer_lqr',
            output='screen',
            emulate_tty=True),
    ])
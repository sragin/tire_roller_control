from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg=['Execute tire roller control launcher!']),
        # Node(
        #     package='tire_roller_control',
        #     executable='base_controller',
        # ),
        Node(
            package='tire_roller_control',
            executable='roller_controller',
        ),
        Node(
            package='tire_roller_control',
            executable='navigator',
        ),
    ])

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
            package='gps_rclpy_pkg',
            executable='tcpgps_geoid_pub',
            parameters=[
                {'gps_ip': '192.168.110.163'},
                {'gps_port': 11511},
            ]
        ),
        Node(
            package='tire_roller_control',
            executable='roller_publisher',
        ),
        # Node(
        #     package='tire_roller_control',
        #     executable='navigator',
        # ),
        # Node(
        #     package='tire_roller_control',
        #     executable='roller_controller',
        # ),
    ])

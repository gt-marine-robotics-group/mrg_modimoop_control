from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="mrg_modimoop_control",
            executable="control_bridge",
            name="control_bridge",
            parameters=[{"rate_hz": 50.0}],
            output="screen",
        ),

        Node(
            package="mrg_modimoop_control",
            executable="localization",
            name="localization",
            output="screen",
            parameters=[{
                'declination_rad': 0.0,
                'use_tilt_compensation': True,
                'publish_rate_hz': 20.0,
            }]
        )
    ])
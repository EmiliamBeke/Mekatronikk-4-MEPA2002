from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="mekk4_bringup",
            executable="talker",
            name="talker",
            output="screen",
        ),
    ])

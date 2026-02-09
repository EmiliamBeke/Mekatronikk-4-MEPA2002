from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    frame_duration_us = LaunchConfiguration("frame_duration_us")

    return LaunchDescription([
        DeclareLaunchArgument("width", default_value="2304"),
        DeclareLaunchArgument("height", default_value="1296"),
        DeclareLaunchArgument("frame_duration_us", default_value="66666"),
        Node(
            package="camera_ros",
            executable="camera_node",
            name="camera_node",
            namespace="camera",
            output="screen",
            parameters=[
                {"camera": 0},
                {"frame_id": "camera"},
                {"width": width},
                {"height": height},
                {"FrameDurationLimits": [frame_duration_us, frame_duration_us]},
            ],
        ),
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    frame_duration_us = LaunchConfiguration("frame_duration_us")
    image_topic = LaunchConfiguration("image_topic")

    return LaunchDescription([
        DeclareLaunchArgument("width", default_value="2304"),
        DeclareLaunchArgument("height", default_value="1296"),
        DeclareLaunchArgument("frame_duration_us", default_value="66666"),
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
        SetEnvironmentVariable("MEKK4_CAM_TOPIC", image_topic),
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
        Node(
            package="mekk4_perception",
            executable="teddy_detector",
            name="teddy_detector",
            output="screen",
        ),
    ])

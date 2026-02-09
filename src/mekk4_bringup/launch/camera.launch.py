from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    fps = LaunchConfiguration("fps")

    return LaunchDescription([
        DeclareLaunchArgument("width", default_value="2304"),
        DeclareLaunchArgument("height", default_value="1296"),
        DeclareLaunchArgument("fps", default_value="15.0"),
        Node(
            package="libcamera_ros",
            executable="camera_node",
            name="camera_node",
            namespace="camera",
            output="screen",
            parameters=[
                {"camera_name": "pi_cam"},
                {"frame_id": "camera"},
                {"width": width},
                {"height": height},
                {"frame_rate": fps},
            ],
        ),
    ])

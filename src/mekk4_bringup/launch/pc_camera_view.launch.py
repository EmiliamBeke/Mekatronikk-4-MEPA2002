from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port = LaunchConfiguration("port")
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    topic_name = LaunchConfiguration("topic_name")
    frame_id = LaunchConfiguration("frame_id")

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="5601"),
        DeclareLaunchArgument("width", default_value="1296"),
        DeclareLaunchArgument("height", default_value="972"),
        DeclareLaunchArgument("topic_name", default_value="/camera"),
        DeclareLaunchArgument("frame_id", default_value="camera_link"),
        SetEnvironmentVariable(
            "MEKK4_CAM_SOURCE_GST",
            [
                "udpsrc port=",
                port,
                " caps=application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000 ",
                "! rtph264depay ! h264parse ! decodebin ! videoconvert ! appsink drop=true max-buffers=1 sync=false",
            ],
        ),
        SetEnvironmentVariable("MEKK4_CAM_WIDTH", width),
        SetEnvironmentVariable("MEKK4_CAM_HEIGHT", height),
        Node(
            package="mekk4_perception",
            executable="udp_camera_bridge",
            name="udp_camera_bridge",
            output="screen",
            parameters=[
                {
                    "width": ParameterValue(width, value_type=int),
                    "height": ParameterValue(height, value_type=int),
                    "topic_name": topic_name,
                    "frame_id": frame_id,
                }
            ],
        ),
    ])

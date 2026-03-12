from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gst_pipeline = LaunchConfiguration("gst_pipeline")

    return LaunchDescription([
        DeclareLaunchArgument(
            "gst_pipeline",
            default_value=(
                "udpsrc port=5600 caps=application/x-rtp,media=video,"
                "encoding-name=H264,payload=96,clock-rate=90000 "
                "! rtph264depay ! h264parse ! avdec_h264 "
                "! videoconvert ! appsink drop=true max-buffers=1 sync=false"
            ),
        ),
        SetEnvironmentVariable("MEKK4_CAM_SOURCE_GST", gst_pipeline),
        Node(
            package="mekk4_perception",
            executable="teddy_detector",
            name="teddy_detector",
            output="screen",
        ),
    ])

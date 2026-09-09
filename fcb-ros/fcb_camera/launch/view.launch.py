"""Decode /<ns>/image/ffmpeg locally into a raw image topic for viewers and CV nodes.

    ros2 launch fcb_camera view.launch.py            # republishes to /fcb/image/decoded
    ros2 run rqt_image_view rqt_image_view /fcb/image/decoded

Prefers the hardware decoder when one exists (hevc_cuvid / h264_cuvid on NVIDIA,
nvv4l2 on Jetson via the ffmpeg_image_transport decoders parameter), falling
back to the software decoder.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ns = LaunchConfiguration("namespace")
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="fcb"),
        DeclareLaunchArgument("out_topic", default_value="image/decoded"),
        Node(
            package="image_transport",
            executable="republish",
            name="fcb_republish",
            namespace=ns,
            output="screen",
            arguments=["ffmpeg", "raw"],
            remappings=[
                ("in/ffmpeg", "image/ffmpeg"),
                ("out", LaunchConfiguration("out_topic")),
            ],
            parameters=[{
                "ffmpeg_image_transport.decoders.hevc": "hevc_cuvid,hevc",
                "ffmpeg_image_transport.decoders.h264": "h264_cuvid,h264",
            }],
        ),
    ])

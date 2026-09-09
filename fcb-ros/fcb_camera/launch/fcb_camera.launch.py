"""Launch the FCB-EV9520L driver.

    ros2 launch fcb_camera fcb_camera.launch.py
    ros2 launch fcb_camera fcb_camera.launch.py codec:=h264 bit_rate:=4000000 video_device:=/dev/video2
    ros2 launch fcb_camera fcb_camera.launch.py params_file:=/path/to/my.yaml

Topics land under the `namespace` argument (default "fcb"):
    /fcb/image/ffmpeg   ffmpeg_image_transport_msgs/FFMPEGPacket   (record this)
    /fcb/camera_info    sensor_msgs/CameraInfo
    /fcb/state          fcb_interfaces/CameraState
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("fcb_camera")
    default_params = os.path.join(share, "config", "fcb_camera.yaml")

    args = [
        DeclareLaunchArgument("namespace", default_value="fcb"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("video_device", default_value="",
                              description="V4L2 node, empty = autodetect"),
        DeclareLaunchArgument("visca_port", default_value="",
                              description="serial port, empty = autodetect"),
        DeclareLaunchArgument("backend", default_value="auto",
                              description="auto | libav | gstreamer"),
        DeclareLaunchArgument("codec", default_value="hevc", description="hevc | h264"),
        DeclareLaunchArgument("encoder_name", default_value="",
                              description="libav encoder or GStreamer element, empty = auto"),
        DeclareLaunchArgument("bit_rate", default_value="8000000"),
        DeclareLaunchArgument("fps", default_value="60.0"),
        DeclareLaunchArgument("width", default_value="1920"),
        DeclareLaunchArgument("height", default_value="1080"),
        DeclareLaunchArgument("camera_info_url",
                              default_value="package://fcb_camera/config/fcb_ev9520l_1080p.yaml"),
        DeclareLaunchArgument("log_level", default_value="info"),
    ]

    node = Node(
        package="fcb_camera",
        executable="fcb_camera_node",
        name="fcb_camera",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "video.device": LaunchConfiguration("video_device"),
                "video.fps": LaunchConfiguration("fps"),
                "video.width": LaunchConfiguration("width"),
                "video.height": LaunchConfiguration("height"),
                "visca.port": LaunchConfiguration("visca_port"),
                "encoder.backend": LaunchConfiguration("backend"),
                "encoder.codec": LaunchConfiguration("codec"),
                "encoder.name": LaunchConfiguration("encoder_name"),
                "encoder.bit_rate": LaunchConfiguration("bit_rate"),
                "camera_info_url": LaunchConfiguration("camera_info_url"),
            },
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )
    return LaunchDescription(args + [node])

"""Launch the QR vision node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the launch description."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("device", default_value="/dev/video0"),
            DeclareLaunchArgument("width", default_value="1280"),
            DeclareLaunchArgument("height", default_value="720"),
            DeclareLaunchArgument("fps", default_value="30"),
            DeclareLaunchArgument("mode", default_value="camera"),
            DeclareLaunchArgument("image_dir", default_value=""),
            Node(
                package="qr_vision",
                executable="qr_vision_node.py",
                name="qr_vision_node",
                output="screen",
                parameters=[
                    {
                        "mode": LaunchConfiguration("mode"),
                        "device": LaunchConfiguration("device"),
                        "width": LaunchConfiguration("width"),
                        "height": LaunchConfiguration("height"),
                        "fps": LaunchConfiguration("fps"),
                        "image_dir": LaunchConfiguration("image_dir"),
                    }
                ],
            ),
        ]
    )

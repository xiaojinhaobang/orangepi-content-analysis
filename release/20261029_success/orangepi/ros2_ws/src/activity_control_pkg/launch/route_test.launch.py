from importlib import import_module
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:  # pragma: no cover
    LaunchDescription = Any
    Node = Any


def generate_launch_description():
    launch_module = import_module("launch")
    launch_ros_actions = import_module("launch_ros.actions")
    LaunchDescription = getattr(launch_module, "LaunchDescription")
    Node = getattr(launch_ros_actions, "Node")

    return LaunchDescription([
        Node(
            package="activity_control_pkg",
            executable="route_test_node",
            name="route_test_node",
            output="screen",
            parameters=[
                {
                    "use_two_second_capture_mode": True,
                    "first_drive_ticks": 50,
                    "first_forward_x_cm": 300.0,
                    "first_forward_y_cm": 0.0,
                    "first_forward_z_cm": 4.0,
                    "takeoff_height_cm": 100.0,
                    "capture_settle_ticks": 3,
                    "capture_window_size": 7,
                }
            ],
        )
    ])

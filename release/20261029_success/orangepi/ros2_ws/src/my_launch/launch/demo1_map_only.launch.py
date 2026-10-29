from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    map_viewer_node = Node(
        package='my_launch',
        executable='map_viewer_node',
        name='map_viewer_node',
        output='screen',
        parameters=[
            {'map_image_path': '/home/orangepi/Downloads'},
            {'x_min_cm': 0.0},
            {'x_max_cm': 350.0},
            {'y_min_cm': 0.0},
            {'y_max_cm': 400.0},
            {'origin_is_bottom': True},
            {'origin_is_right': True},
            {'show_hud_always': True},
            {'debug_force_marker': True},
            {'log_overlay_heartbeat': True},
            {'draw_default_when_tf_missing': True},
            {'window_name': 'Flight Map Viewer (Map Only)'}
        ]
    )

    return LaunchDescription([
        map_viewer_node,
    ])

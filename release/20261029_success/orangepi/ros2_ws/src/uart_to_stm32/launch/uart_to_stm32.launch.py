from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    continuous_fc_notify_11_arg = DeclareLaunchArgument(
        'continuous_fc_notify_11',
        default_value='true',
        description='Enable continuous 0x11 debug frames to FC.',
    )
    continuous_fc_notify_period_sec_arg = DeclareLaunchArgument(
        'continuous_fc_notify_period_sec',
        default_value='0.2',
        description='Period (seconds) for continuous 0x11 debug frame sending.',
    )
    continuous_fc_notify_duration_sec_arg = DeclareLaunchArgument(
        'continuous_fc_notify_duration_sec',
        default_value='5.0',
        description='How long (seconds) to keep sending 0x11 in debug mode.',
    )
    simulate_fourth_waypoint_notify_arg = DeclareLaunchArgument(
        'simulate_fourth_waypoint_notify',
        default_value='false',
        description='Simulate reaching 4th waypoint and send 0x11 three times once.',
    )
    simulate_fourth_waypoint_notify_delay_sec_arg = DeclareLaunchArgument(
        'simulate_fourth_waypoint_notify_delay_sec',
        default_value='3.0',
        description='Delay seconds before simulated 4th-waypoint 0x11 sending.',
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyS3',
        description='UART used to talk to the STM32 flight controller '
                    '(ttyS3 = 40pin 31/33, ttyS4 = 40pin 23/19, ttyS6 = touchscreen).',
    )
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='921600',
        description='Baudrate for the flight-controller link.',
    )

    return LaunchDescription([
        continuous_fc_notify_11_arg,
        continuous_fc_notify_period_sec_arg,
        continuous_fc_notify_duration_sec_arg,
        simulate_fourth_waypoint_notify_arg,
        simulate_fourth_waypoint_notify_delay_sec_arg,
        serial_port_arg,
        baud_rate_arg,
        Node(
            package='uart_to_stm32',
            executable='uart_to_stm32_node',
            name='uart_to_stm32',
            parameters=[
                {'update_rate': 100.0},
                {'source_frame': 'map'},
                {'target_frame': 'laser_link'},
                {'serial_port': LaunchConfiguration('serial_port')},
                {'baud_rate': LaunchConfiguration('baud_rate')},
                {'continuous_fc_notify_11': LaunchConfiguration('continuous_fc_notify_11')},
                {'continuous_fc_notify_period_sec': LaunchConfiguration('continuous_fc_notify_period_sec')},
                {'continuous_fc_notify_duration_sec': LaunchConfiguration('continuous_fc_notify_duration_sec')},
                {'simulate_fourth_waypoint_notify': LaunchConfiguration('simulate_fourth_waypoint_notify')},
                {'simulate_fourth_waypoint_notify_delay_sec': LaunchConfiguration('simulate_fourth_waypoint_notify_delay_sec')},
            ],
            output='screen'
        )
    ])

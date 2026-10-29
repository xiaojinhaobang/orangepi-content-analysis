"""阶段三一键启动：雷达建图 + 定点导航链路。

前置：雷达驱动必须已在跑（由自启动 ~/radar_start.sh 独占 UDP 6668），
      本 launch 故意不再拉雷达，避免两个进程抢 6668。
      自检: ~/diag/preflight.sh   （[1] 雷达 必须 PASS）

拉起顺序：
  0s  robot_state_publisher (fly.urdf: base_link -> laser_link)
  1s  cartographer_node     (my_carto_pkg / amphi.lua, 纯雷达扫描匹配)
  2s  cartographer_occupancy_grid_node  (/map)
  3s  uart_to_stm32         (map -> laser_link 的 tf 转 0x31 速度指令)
  5s  position_pid_controller (位置环, 输出 /target_velocity)
  5s  route_target_publisher  (航点表, 可选: use_route:=true)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_route = DeclareLaunchArgument(
        'use_route', default_value='false',
        description='启动航点发布器（默认 false：避免一上电就发目标点）')
    serial_port = DeclareLaunchArgument('serial_port', default_value='/dev/ttyS3')
    baud_rate = DeclareLaunchArgument('baud_rate', default_value='921600')
    resolution = DeclareLaunchArgument('resolution', default_value='0.05')

    carto_share = get_package_share_directory('my_carto_pkg')
    with open(os.path.join(carto_share, 'urdf', 'fly.urdf')) as urdf_file:
        robot_desc = urdf_file.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}, {'use_sim_time': False}],
        output='screen')

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', os.path.join(carto_share, 'configuration_files'),
            '-configuration_basename', 'amphi.lua'],
        remappings=[('scan', 'scan')],
        output='screen')

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        parameters=[{'use_sim_time': False}],
        arguments=['-resolution', LaunchConfiguration('resolution'),
                   '-publish_period_sec', '1.0'],
        output='screen')

    uart_to_stm32_node = Node(
        package='uart_to_stm32',
        executable='uart_to_stm32_node',
        name='uart_to_stm32',
        parameters=[
            {'update_rate': 100.0},
            {'source_frame': 'map'},
            {'target_frame': 'laser_link'},
            {'serial_port': LaunchConfiguration('serial_port')},
            {'baud_rate': LaunchConfiguration('baud_rate')},
        ],
        remappings=[('/velocity_map', '/velocity_map_filtered')],
        output='screen')

    position_pid_controller = Node(
        package='pid_control_pkg',
        executable='position_pid_controller',
        name='position_pid_controller',
        parameters=[{
            'control_frequency': 50.0,
            'map_frame': 'map',
            'laser_link_frame': 'laser_link',
            'position_tolerance': 6.0,
            'yaw_tolerance': 5.0,
            'height_tolerance': 6.0,
            'kp_xy': 0.5, 'ki_xy': 0.0, 'kd_xy': 0.1,
            'kp_yaw': 1.0, 'ki_yaw': 0.0, 'kd_yaw': 0.2,
            'kp_z': 0.9, 'ki_z': 0.0, 'kd_z': 0.15,
            'max_linear_velocity': 33.0,
            'max_angular_velocity': 30.0,
            'max_vertical_velocity': 30.0,
            'max_slow_velocity': 20.0,
            'vertical_takeoff_index': 1,
            'vertical_takeoff_min_z_cm': 20.0,
            'takeoff_max_xy_velocity': 8.0,
        }],
        output='screen')

    route_target_publisher = Node(
        package='activity_control_pkg',
        executable='route_target_publisher_node',
        name='route_target_publisher',
        condition=IfCondition(LaunchConfiguration('use_route')),
        parameters=[{
            'map_frame': 'map',
            'laser_link_frame': 'laser_link',
            'output_topic': '/target_position',
            'position_tolerance_cm': 18.0,
            'yaw_tolerance_deg': 15.0,
            'height_tolerance_cm': 6.0,
        }],
        output='screen')

    # 速度滤波：静止时 SLAM 有 ±2~3cm/s 假速度，死区+低通后喂给飞控
    velocity_filter = ExecuteProcess(
        cmd=['python3', '/home/orangepi/diag/velocity_filter.py'],
        name='velocity_filter',
        output='screen')

    return LaunchDescription([
        use_route, serial_port, baud_rate, resolution,
        TimerAction(period=0.0, actions=[robot_state_publisher]),
        TimerAction(period=1.0, actions=[cartographer_node]),
        TimerAction(period=2.0, actions=[occupancy_grid_node]),
        TimerAction(period=2.5, actions=[velocity_filter]),
        TimerAction(period=3.0, actions=[uart_to_stm32_node]),
        TimerAction(period=5.0, actions=[position_pid_controller]),
        TimerAction(period=5.0, actions=[route_target_publisher]),
    ])

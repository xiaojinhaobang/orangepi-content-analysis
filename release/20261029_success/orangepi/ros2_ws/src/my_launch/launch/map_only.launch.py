"""建图模式：只建图，不发任何速度指令。

组成：robot_state_publisher + cartographer_node + cartographer_occupancy_grid_node
故意不含 uart_to_stm32 / position_pid_controller —— 建图期间飞控收不到速度帧。

前置：雷达驱动必须已在跑（自启动 ~/radar_start.sh 独占 UDP 6668）。
      自检: ~/diag/preflight.sh   （[1] 雷达必须 PASS，[2][3][4] 允许 FAIL）

建图约定（重要）：先把飞机放在起飞点、机头朝约定方向，再启动本 launch。
                 这样地图原点 (0,0,0) 就是起飞点，目标点坐标可直接使用。

存图：走完一圈后执行  ~/diag/save_map.sh <名字>
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    resolution = DeclareLaunchArgument('resolution', default_value='0.05')
    config = DeclareLaunchArgument('config', default_value='amphi.lua',
                                   description='my_carto_pkg 里的 lua 配置名')

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
            '-configuration_basename', LaunchConfiguration('config')],
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

    return LaunchDescription([
        resolution, config,
        TimerAction(period=0.0, actions=[robot_state_publisher]),
        TimerAction(period=1.0, actions=[cartographer_node]),
        TimerAction(period=2.0, actions=[occupancy_grid_node]),
    ])

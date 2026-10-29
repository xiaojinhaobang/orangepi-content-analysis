"""阶段三（简易方案）：航点 + 速度积分定位，不用地图。

链路：
  0.0s  robot_state_publisher     base_link -> laser_link（cartographer 需要）
  0.5s  cartographer_node         只为拿 /velocity_map（飞控定点环的速度测量源）
  1.5s  velocity_filter           /velocity_map -> 死区+低通 -> /velocity_map_filtered
  2.0s  dead_reckon               速度积分 -> tf map -> dr_base + /dead_reckon/pose
                                  （方案1 必须 source=raw，见 reckon_source 参数）
  3.0s  uart_to_stm32             0x31 速度指令 / 0x32 速度注入 / 0x66 任务控制
  5.0s  position_pid_controller   位置环（看 dr_base）-> /target_velocity
  5.0s  route_target_publisher    航点表 -> /target_position（use_route:=true 才起）

为什么 cartographer 还在：飞控 mode 2 定点需要"水平速度测量"，板上唯一的
雷达速度源就是它的 /velocity_map（本地补丁的发布代码已丢失，不能重编译它）。
但**导航坐标不用地图**：位置环看的是 dead_reckon 的 dr_base。

操作顺序：
  1) CH5 中档（模式2）→ 手动内八解锁 → 确认 unlock_sta=1
  2) ros2 topic pub --once /active_controller std_msgs/msg/UInt8 "{data: 2}"
  3) ros2 topic pub --once /mission_cmd std_msgs/msg/Int16MultiArray "{data: [1, 100]}"
     ← 0x66 起飞到 100cm；这一步不做的话飞控会把所有速度指令置 0
  4) 记录：~/diag/flight_trace.py 120
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_route = DeclareLaunchArgument('use_route', default_value='false')
    serial_port = DeclareLaunchArgument('serial_port', default_value='/dev/ttyS3')
    baud_rate = DeclareLaunchArgument('baud_rate', default_value='921600')
    nav_frame = DeclareLaunchArgument(
        'nav_frame', default_value='dr_base',
        description='位置环/航点看的子帧：dr_base=速度积分, laser_link=SLAM')
    open_loop = DeclareLaunchArgument(
        'open_loop', default_value='false',
        description='true=注入恒定有效0，打开飞控位置环，只用香橙派的位置环')
    reckon_source = DeclareLaunchArgument(
        'reckon_source', default_value='raw',
        description='dead_reckon 的积分输入：raw=/velocity_map(默认,方案1必须用), '
                    'filtered=/velocity_map_filtered(open_loop 时恒为0,会冻结位姿), '
                    'fc_echo=/fc_vel_echo')
    axis_swap = DeclareLaunchArgument(
        'axis_swap', default_value='false',
        description='已废弃（固定 90° 旋转只在机头朝向不变时成立）。'
                    '统一改为 false：导航系=地图系，航向旋转交给 body_vel_adapter')
    vel_src = DeclareLaunchArgument(
        'vel_src', default_value='/velocity_map',
        description='velocity_filter 的输入：/velocity_map=雷达原始(基线)；'
                    '/velocity_fused=IMU 延迟补偿后（见 imu_fusion.py）')
    use_reanchor = DeclareLaunchArgument(
        'use_reanchor', default_value='false',
        description='true = 周期重发 0x66[1,当前高度]，让飞控的定点保持点跟着飞机走'
                    '（模式2 下要飞远航点必须开；见 reanchor.py 注释）')
    reanchor_period = DeclareLaunchArgument('reanchor_period', default_value='1.0')
    waypoints_file = DeclareLaunchArgument(
        'waypoints_file', default_value='/home/orangepi/diag/waypoints_demo.yaml',
        description='航点表 + 简易地图(YAML)')

    carto_share = get_package_share_directory('my_carto_pkg')
    with open(os.path.join(carto_share, 'urdf', 'fly.urdf')) as urdf_file:
        robot_desc = urdf_file.read()

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}, {'use_sim_time': False}],
        output='screen')

    # ★ 2026-10-15：**改成"加载已标定地图"的定位模式**，固定地图坐标系朝向。
    #   背景：cartographer 原来每次都是"新建地图"（全 SLAM），而新图的坐标系朝向
    #   是任意的 → `psi0`（地图系里机头朝向）每次不同（-132°/0.1°/56°/100° 都出现过）
    #   → 同一条航线在房间里有时正、有时转过头。现场表现："我一直按平时方向摆，
    #   今天方向却不对"。
    #   改法：`load_state_filename` 指向标定好的 pbstream + 打开定位模式。
    #   配置目录指到 ~/diag/carto_cfg（那个目录我们能写，不用 sudo）；
    #   只有本 launch 文件本身需要一条 sudo cp。
    # ★ 2026-10-15：定位模式（加载已标定地图）—— 用 `CARTO_LOAD_STATE` 控制，
    #   **默认开启**并指向 `~/maps/site.pbstream`（`save_map.sh site` 存的那张）。
    #   关掉（回全 SLAM）：起链路时带 `CARTO_LOAD_STATE=`（空）。
    #   注意：第一次崩（SIGABRT）是 lua 里 `pure_localization_trimmer` 传了空表，
    #   已补 `max_submaps_to_keep = 3` 修好。
    CARTO_CFG = os.path.expanduser('~/diag/carto_cfg')
    _env_ls = os.environ.get('CARTO_LOAD_STATE')
    PBSTREAM = os.path.expanduser(_env_ls if _env_ls is not None
                                  else '~/maps/site.pbstream')
    cartographer_node = Node(
        package='cartographer_ros', executable='cartographer_node',
        name='cartographer_node',
        parameters=[{'use_sim_time': False}]
        + ([{'load_state_filename': PBSTREAM}] if PBSTREAM else []),
        arguments=['-configuration_directory',
                   CARTO_CFG if os.path.isdir(CARTO_CFG)
                   else os.path.join(carto_share, 'configuration_files'),
                   '-configuration_basename', 'amphi.lua']
        # ★ 2026-10-15：cartographer_ros 读的是**命令行参数**形式
        #   （`-load_state_filename`），走 parameters 时实测没生效
        #   （连续三次重启 psi 都在变：56.4°/37.4°/-0.27°）。
        + (['-load_state_filename', PBSTREAM] if PBSTREAM else []),
        remappings=[('scan', 'scan')], output='screen')

    velocity_filter = ExecuteProcess(
        cmd=['python3', '/home/orangepi/diag/velocity_filter.py',
             '--ros-args', '-p', ['open_loop:=', LaunchConfiguration('open_loop')],
             # ★ 2026-10-09：输入话题可切换（输出名不变 → 飞控侧不用动，A/B 也对称）
             # 注意：ExecuteProcess 不支持 remappings= 关键字，必须走 -r 命令行参数
             '-r', ['/velocity_map:=', LaunchConfiguration('vel_src')]],
        name='velocity_filter', output='screen')

    dead_reckon = ExecuteProcess(
        cmd=['python3', '/home/orangepi/diag/dead_reckon.py',
             '--ros-args',
             '-p', ['source:=', LaunchConfiguration('reckon_source')],
             '-p', ['axis_swap:=', LaunchConfiguration('axis_swap')],
             '-p', 'yaw_from_tf:=true',
              # 2026-09-30 回退：实测 τ 0.20s 那几场航向峰峰反而更大（21°/26°/更差），
              # 回到 9/28 那套最好成绩的 τ≈0.67s（见 launch 里 kp_yaw 那段调参记录）
              # ★ 2026-10-07：悬停那套不用动，但**掉头**被这个低通害惨了：
              #   τ=1.7s × 转速 = 反馈滞后多少度就转过头多少度（15deg/s → 25°）。
              #   改成 0.12（τ≈0.42s）：掉头滞后误差降到 ~1/4，悬停侧影响可接受。
              '-p', 'yaw_lpf_alpha:=0.12',
             # ★ 2026-10-06：50Hz → 20Hz。实测本节点吃 ~53% 一个核（每帧要做一次
             #   tf 查询），而位置环现在就是 20Hz，50Hz 纯属浪费；20Hz 足够
             #   给位置环供应位姿。
             '-p', 'rate_hz:=20.0',
             # 2026-10-18 A/B：dr_base 直接镜像 cartographer 的 map->base_link，
             # 不再积分 /velocity_map。reset 由 passthrough_offset 处理。
             '-p', 'passthrough:=true',
             '-p', 'passthrough_frame:=base_link'],
        name='dead_reckon', output='screen')

    uart_to_stm32_node = Node(
        package='uart_to_stm32', executable='uart_to_stm32_node', name='uart_to_stm32',
        parameters=[{'update_rate': 100.0}, {'source_frame': 'map'},
                    {'target_frame': LaunchConfiguration('nav_frame')},
                    {'serial_port': LaunchConfiguration('serial_port')},
                    {'baud_rate': LaunchConfiguration('baud_rate')}],
        remappings=[('/velocity_map', '/velocity_map_filtered')],
        output='screen')

    position_pid_controller = Node(
        package='pid_control_pkg', executable='position_pid_controller',
        name='position_pid_controller',
        parameters=[{
            'control_frequency': 20.0,
            'map_frame': 'map',
            'laser_link_frame': LaunchConfiguration('nav_frame'),
            'position_tolerance': 6.0, 'yaw_tolerance': 5.0, 'height_tolerance': 6.0,
            # 2026-10-18：和参考工程对齐的位姿/高度健康门控。
            'pose_data_timeout_sec': 0.5,
            'height_min_cm': 0.0, 'height_max_cm': 250.0,
            'height_filter_jump_threshold_cm': 35.0,
            'height_filter_required_frames': 5,
            # ki_xy 试过 0.1（9/30 15:17 那场）：静态偏置没消掉，反而出现 ±5cm 的慢速摆动
            # （x 3.9→5.8→0.2→-5.7，指令 ±4~5cm/s），典型的积分+滞后引起的慢振荡。
            # **回退 ki=0**，接受 ~5cm 静差（在 6cm 容差内）。
            # 2026-10-06：kp_xy 0.35 → 0.7。理由：起飞横向漂 12~16cm 时我们的修正只有
            # 0.35×16≈5.6cm/s（上限 15 远没打满）→ 不是拉不住而是拉得慢；提高比例增益让
            # 起飞漂移回落更快、悬停静差更小。积分仍然保持 0（试过 0.1 会慢摆）。
            'kp_xy': 0.7, 'ki_xy': 0.0, 'kd_xy': 0.25,
            # 2026-09-28：飞控自锁航向有 ±6° 低频摆动（疑似磁罗盘受室内磁场污染）。
            # 现在 dead_reckon 已经给航向加了低通（τ≈0.7s），可以安全开一个"慢速外环"。
            # 偏航环调参记录（航向峰峰 / 水平最大偏移，8~15s 稳态窗口）：
            #   0.5  τ0.67 kd0   速度α0.15 → 航向 7.5°  水平 7.3cm（横滚明显）
            #   0.5  τ0.67 kd0   速度α0.35 → 航向12.0°  水平 4.2cm  ★9/28 18:37 最佳综合
            #   0.7  τ0.20 kd0   速度α0.35 → 航向21.1° @0.72Hz（环自激，指令 ±8.8deg/s）
            #   0.25 τ0.20 kd0   速度α0.35 → 航向26.4° @0.29Hz（环太弱，追不上漂移）
            #   0.5  τ0.20 kd0.15 速度α0.35 → 更差（9/30 14:36/38）
            # ★ 2026-09-30 决定：**整体回退到 9/28 18:37 那套**（下面这组），
            #   它是目前综合最好的一次：水平 4.2cm、高度 117.9±0.6、航向 12°、横滚已消失。
            #   后续先调飞控侧（ID65 动力参数），我们这边冻结不动。
            # 2026-10-04 实测（7 场悬停，我们的环参与）：航向前半段稳（2~6°），**10 秒后
            # 开始摆**并越来越大（后半段 11~18°）。相位分析：我们的偏航指令滞后实际角速度
            # 1.0~1.25s（滞后来自飞控响应+SLAM 航向估计，不是我们的滤波）→ 我们只能追、追不住。
            # 对策：加**微分阻尼**（相位超前）。τ 已降到 0.03（航向信号干净），kd 可以放心用。
            'kp_yaw': 0.0, 'ki_yaw': 0.04, 'kd_yaw': 0.0, 'yaw_deadband_deg': 0.5,
            # ★ 2026-10-14：z 环调温和 0.9/0.0/0.15 → 0.3/0.0/0.05。
            #   理由：C++ 直接吃 200Hz 的 /height（只挡 >100cm 毛刺），kd_z 把高度
            #   噪声放大成抖；悬停自测那条链路**完全不碰高度**（vz 恒 0）反而稳。
            #   调温和后高度基本交飞控，只在"换排"（142↔103）时慢慢修。
            # ★ 2026-10-15：0.3 → 1.0。现场"下降太慢了" —— 换排要从 142 降到 103
            #   （39cm），kp_z=0.3 只给 11.7cm/s，还要跟飞控自己的 142 定高环抢
            #   → 降得极慢。1.0 给 39cm/s，快得多。kd_z 保持 0.05（原来是 0.15，
            #   高 D + 带毛刺的 200Hz /height 会抖，这个坑不要再踩）。
            'kp_z': 1.0, 'ki_z': 0.0, 'kd_z': 0.05,
            # 2026-09-30：巡线速度 22 → 15 cm/s。任务飞行中横滚/左右摆动比悬停时明显，
            # 先降速减小姿态扰动（也顺带让扫码更从容）。
            # ★ 2026-10-07：20 → 15（先按 12 试过，现场觉得太保守）。平移速度越小，
            #   推力倾斜越小、对偏航的扰动也越小；15 是"稳一点但别太磨"的折中。
            # ★ 2026-10-14：巡航 15 → 22 cm/s（配合"快滑"把流程做利索）；
            #   max_slow_velocity 保持 15（最后接近段仍慢，保停位精度）。
            'max_linear_velocity': 22.0, 'max_angular_velocity': 30.0,
            'max_vertical_velocity': 30.0, 'max_slow_velocity': 15.0,
            'vertical_takeoff_index': 1, 'vertical_takeoff_min_z_cm': 20.0,
            'takeoff_max_xy_velocity': 8.0,
        }],
        remappings=[('/target_velocity', '/target_velocity_nav')],
        output='screen')

    # 导航系 -> 机体系 旋转（按 tf 里的实时航向）。位置环输出必须过这一层，
    # 否则飞机一转向，修正方向就整体错 90°。
    body_vel_adapter = ExecuteProcess(
        cmd=['python3', '/home/orangepi/diag/body_vel_adapter.py',
             '--ros-args', '-p', ['nav_frame:=', LaunchConfiguration('nav_frame')]],
        name='body_vel_adapter', output='screen')

    waypoint_nav = ExecuteProcess(
        cmd=['python3', '/home/orangepi/diag/waypoint_nav.py',
             '--ros-args',
             '-p', ['file:=', LaunchConfiguration('waypoints_file')],
             '-p', ['body_frame:=', LaunchConfiguration('nav_frame')]],
        name='waypoint_nav',
        condition=IfCondition(LaunchConfiguration('use_route')),
        output='screen')

    reanchor = ExecuteProcess(
        cmd=['python3', '/home/orangepi/diag/reanchor.py',
             '--ros-args', '-p', ['period_s:=', LaunchConfiguration('reanchor_period')]],
        name='reanchor',
        condition=IfCondition(LaunchConfiguration('use_reanchor')),
        output='screen')

    return LaunchDescription([
        use_route, serial_port, baud_rate, nav_frame, waypoints_file, open_loop,
        reckon_source, axis_swap, vel_src, use_reanchor, reanchor_period,
        # 启动间隔原来 0/0.5/1.5/2/3/4.5/5 s，实测过保守；DDS 发现与话题订阅
        # 不依赖顺序（订阅者可以先于发布者存在），这里压到 0.3 s 一档，
        # 冷启动就绪时间从约 7.4 s 降到约 3 s。流程要求"按一下就快速开始"。
        TimerAction(period=0.0, actions=[robot_state_publisher]),
        TimerAction(period=0.3, actions=[cartographer_node]),
        TimerAction(period=0.6, actions=[velocity_filter]),
        TimerAction(period=0.9, actions=[dead_reckon]),
        TimerAction(period=1.2, actions=[uart_to_stm32_node]),
        TimerAction(period=1.5, actions=[body_vel_adapter]),
        TimerAction(period=1.8, actions=[position_pid_controller]),
        TimerAction(period=1.8, actions=[waypoint_nav]),
        TimerAction(period=2.1, actions=[reanchor]),
    ])

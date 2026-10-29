#!/usr/bin/env bash
# 链路启动脚本（给独立的 flight-link.service 用）。
# 为什么独立：实测从 flight-stack 服务上下文里 `setsid nohup ros2 launch` 会
# core dump，而从 SSH 里同样一条命令正常；怀疑是 cgroup/服务生命周期连坐。
# 这个服务把 launch 放在自己的 cgroup 里，前台运行（exec），由 systemd 直接管。
set -e

# ★ 2026-10-19：**禁止第二份 launch 同时跑**。
#   19:46 有人手动起了一份 open_loop:=true 的 launch，把本脚本的 false 顶掉，
#   导致 19:57 / 20:02 两次"飞控定点被关、起飞自由漂"。
#   锁由内核在进程退出时自动释放；-w 10 留给 systemd 重启的旧进程收尾时间。
exec 9>/tmp/flight_link.lock
flock -w 10 9 || { echo "!! 另一份 flight-link 已在运行，本实例退出"; exit 1; }

source /opt/ros/humble/setup.bash
source /home/orangepi/kian_ws/install/setup.bash
source /home/orangepi/cartographer_ws/install/setup.bash
source /home/orangepi/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=6
export ROS_LOCALHOST_ONLY=0
# ★2026-10-17：不再加载 ~/maps/site.pbstream（那张图已 85x68m、被跳变污染，
#   定位会贴到错解 → 实测静止时航向跳 58°）。改为纯现场建图。
#   要恢复定位模式：删掉下面这行即可（备份 flight_link_run.sh.bak-20261017-nomap）。
export CARTO_LOAD_STATE=

WP="${WP_FILE:-/home/orangepi/diag/waypoints_square.yaml}"
echo "=== flight-link 启动: $(date) open_loop=false wp=$WP ==="
exec ros2 launch my_launch stage3_simple.launch.py \
  open_loop:=false vel_src:=/velocity_map use_route:=false use_reanchor:=false \
  waypoints_file:="$WP"

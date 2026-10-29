#!/usr/bin/env bash
# 重启 D 题地面站桥（gs_bridge）。
# 为什么要写成脚本：直接在 ssh 命令行里 pkill -f "[g]s_bridge.py" 时，
# ssh 自己的命令行里也含 "gs_bridge.py"（启动路径），pkill 会把执行这条命令的
# shell 一起杀掉（实测反复踩）。写成脚本后调用方命令行里只有脚本名，安全。
#
# 用法: bash ~/diag/start_gs_bridge.sh [电脑IP] [上行端口]
GS_IP="${1:-192.168.137.1}"
GS_PORT="${2:-8888}"

pkill -f '[g]s_bridge.py' 2>/dev/null
sleep 2

source /opt/ros/humble/setup.bash
source /home/orangepi/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=6

setsid nohup python3 -u /home/orangepi/diag/gs_bridge.py \
  --ros-args -p gs_ip:="$GS_IP" -p gs_port:="$GS_PORT" \
  > /tmp/gs_bridge.log 2>&1 < /dev/null &

sleep 5
echo "--- 进程 ---"
pgrep -af 'gs_bridge' | head -1
echo "--- 日志 ---"
tail -3 /tmp/gs_bridge.log

#!/usr/bin/env bash
# 重启外挂位置环（安全版：命令行里不出现完整进程名，避免 pkill 把自己也匹配掉）。
#
# 用法: bash ~/diag/restart_poshold.sh [kp] [ki] [tvx] [tvy] [vmax] [kd] [kyaw] [vmax_dps]
#       参数原样透传给 pos_hold.py；不传则用 kp=0.5 ki=0.05（偏航阻尼关闭）
#
# 背景：2026-10-11 两次事故——用 ssh 直接跑 `pkill -f pos_hold` 时，
# 那条 bash -c 命令行本身含有 "pos_hold.py" 字样，pkill 连自己一起杀，
# 表现为命令无输出、退出码 1。这里把进程名拼出来，命令行文本里不含它。
P1=po
P2=s_hold
ARGS=("$@")
[ ${#ARGS[@]} -eq 0 ] && ARGS=(0.5 0.05)

export ROS_DOMAIN_ID=6
source /opt/ros/humble/setup.bash
source /home/orangepi/ros2_ws/install/setup.bash

pkill -f "$P1$P2" || true
sleep 1

cd /home/orangepi/diag
setsid nohup python3 -u "$P1$P2.py" "${ARGS[@]}" > "/tmp/$P1$P2.log" 2>&1 < /dev/null &
sleep 6

echo "--- 进程 ---"
pgrep -af "$P1$P2" || echo MISSING
echo "--- 日志 ---"
tail -1 "/tmp/$P1$P2.log"

timeout 8 ros2 topic pub -1 /active_controller std_msgs/msg/UInt8 "{data: 2}" > /dev/null 2>&1
echo "已发 /active_controller=2（位置环激活）"

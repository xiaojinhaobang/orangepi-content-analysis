#!/usr/bin/env bash
# 重启试飞记录器。用法： bash ~/diag/restart_recorder.sh [输出文件]
# 必须写成脚本调用：直接在命令行里同时出现记录器文件名和匹配模式，会把自己的 shell 杀掉。
pkill -f '[f]light_log.py' 2>/dev/null
sleep 1

source /opt/ros/humble/setup.bash
source /home/orangepi/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=6

OUT="${1:-$HOME/traces/rec_$(date +%H%M%S).log}"
setsid nohup python3 -u /home/orangepi/diag/flight_log.py 1800 "$OUT" \
  > /tmp/flog.out 2>&1 < /dev/null &
sleep 3
echo "记录中 → $OUT"
tail -1 /tmp/flog.out

#!/bin/bash
# 起飞前准备：拉起 auto_guard（若未运行）+ 跑 3 秒体检。只做这两件事。
set +u
export ROS_DOMAIN_ID=6
source /opt/ros/humble/setup.bash
source /home/orangepi/ros2_ws/install/setup.bash 2>/dev/null

# 0) 清掉会抢 /target_velocity 的残留（mission 已改成不再启动它）
pids=$(ps -eo pid,args | awk '$0 ~ /pos_hold\.py/ && $0 !~ /awk/ {print $1}')
if [ -n "$pids" ]; then
  kill $pids 2>/dev/null
  echo "已停掉残留 pos_hold：$pids"
fi

# 0b) 上一轮任务结束没停的 waypoint_nav 会一直发 /target_position（陈旧目标）
pids=$(ps -eo pid,args | awk '$0 ~ /waypoint_nav\.py/ && $0 !~ /awk/ {print $1}')
if [ -n "$pids" ]; then
  kill $pids 2>/dev/null
  echo "已停掉残留 waypoint_nav：$pids"
fi

if pgrep -f "[a]uto_guard.py" >/dev/null 2>&1; then
  echo "auto_guard 已在运行（pid $(pgrep -f '[a]uto_guard.py' | head -1)）"
else
  rm -f /tmp/guard_land
  setsid nohup python3 /home/orangepi/diag/auto_guard.py > /tmp/auto_guard.out 2>&1 < /dev/null &
  sleep 2
  if pgrep -f "[a]uto_guard.py" >/dev/null 2>&1; then
    echo "auto_guard 已启动（事件日志 /tmp/auto_guard.log）"
  else
    echo "!! auto_guard 启动失败，看 /tmp/auto_guard.out"
  fi
fi

echo
python3 /home/orangepi/diag/preflight_fast.py

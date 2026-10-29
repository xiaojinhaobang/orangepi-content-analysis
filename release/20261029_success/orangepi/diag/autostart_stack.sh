#!/usr/bin/env bash
# 飞行栈守护：开机自启 + 进程掉了自动补起（现场要求"不用每次都起"）。
#
# 做四件事（每 15 秒巡检一次）：
#   1. 链路自检（cartographer 常掉，见 ensure_link.sh）
#   2. fcstat_log  → /tmp/fcstat_pwm.csv（含 ms / pwm / 4 路 PWM）
#   3. mission_node（A1~A6 子集）
#   4. arm_trigger（解锁即自动 /gs/start；跑完会退出，这里会重新拉起）
#
# 安装（不用 sudo）：crontab -e 里加一行
#   @reboot sleep 25; /home/orangepi/diag/autostart_stack.sh >/tmp/autostart_stack.log 2>&1 &
# 手动起：setsid nohup /home/orangepi/diag/autostart_stack.sh >/tmp/autostart_stack.log 2>&1 < /dev/null &
# 停：pkill -f autostart_stac[k]；再 pkill 各子进程
export ROS_DOMAIN_ID=6
source /opt/ros/humble/setup.bash
source /home/orangepi/ros2_ws/install/setup.bash
D=/home/orangepi/diag
SUBSET=${SUBSET:-}

echo "[autostart] 守护启动 $(date '+%H:%M:%S') subset=$SUBSET"
while true; do
  # 0) 传感器桥：cartographer 要 /imu 和 /odom，缺一个就永远等数据、不出 TF
  if ! pgrep -f '[i]mu_pub.py' >/dev/null; then
    setsid nohup python3 /home/orangepi/diag/imu_pub.py \
      > /tmp/imu.log 2>&1 < /dev/null &
    echo "[autostart] $(date '+%H:%M:%S') 起 imu_pub"
  fi
  if ! pgrep -f '[f]c_odom.py' >/dev/null; then
    setsid nohup python3 /home/orangepi/diag/fc_odom.py --rate 20 \
      > /tmp/odom.log 2>&1 < /dev/null &
    echo "[autostart] $(date '+%H:%M:%S') 起 fc_odom"
  fi
  # 1) 链路（ensure_link 自己会补 cartographer；只在没就绪时才跑，省时间）
  if ! bash $D/ensure_link.sh 2>&1 | grep -q READY; then
    echo "[autostart] $(date '+%H:%M:%S') 链路 NOT_READY，15 秒后重试"
  fi
  # 2) 记录器
  if ! pgrep -f '[f]cstat_log.py' >/dev/null; then
    setsid nohup python3 -u $D/fcstat_log.py /tmp/fcstat_pwm.csv \
      > /tmp/fcstat_pwm.out 2>&1 < /dev/null &
    echo "[autostart] $(date '+%H:%M:%S') 起 fcstat_log"
  fi
  # 3) 任务节点
  if ! pgrep -f '[m]ission_node.py' >/dev/null; then
    bash $D/start_mission_subset.sh "$SUBSET" > /tmp/mission_start.out 2>&1
    echo "[autostart] $(date '+%H:%M:%S') 起 mission_node ($SUBSET)"
  fi
  # 4) 解锁触发器（发完 START 会自己退出 → 这里自动重新拉起）
  if ! pgrep -f '[a]rm_trigger.py' >/dev/null; then
    setsid nohup python3 -u $D/arm_trigger.py 0.8 3600 start \
      > /tmp/arm_start.log 2>&1 < /dev/null &
    echo "[autostart] $(date '+%H:%M:%S') 起 arm_trigger"
  fi
  sleep 15
done

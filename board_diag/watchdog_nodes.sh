#!/usr/bin/env bash
# 看住"掉了就飞不了"的关键节点，发现不在就拉起来。
# 起因：body_vel_adapter 在开机自启时偶发 SIGSEGV（exit -11），
#       它挂掉位置环的指令就发不出去（等于没控制），必须自动补起来。
# 用法： setsid nohup bash ~/diag/watchdog_nodes.sh > /tmp/watchdog.log 2>&1 < /dev/null &
source /opt/ros/humble/setup.bash
source /home/orangepi/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=6

log() { echo "$(date '+%H:%M:%S') $*"; }

while true; do
  # body_vel_adapter：位置环 -> 飞控 的坐标适配层，必须一直活着。
  # ⚠ 重启链路时它会短暂消失（launch 重建），所以先等 20 秒再确认一次，
  #   否则看门狗会和 launch 各起一个 → 两个节点同时发 /target_velocity（冲突）。
  if ! pgrep -f '[b]ody_vel_adapter.py' >/dev/null 2>&1; then
    sleep 20
    if ! pgrep -f '[b]ody_vel_adapter.py' >/dev/null 2>&1; then
      log "body_vel_adapter 消失 20s，重启"
      setsid nohup python3 -u /home/orangepi/diag/body_vel_adapter.py \
        --ros-args -p nav_frame:=dr_base >> /tmp/body_vel_adapter.log 2>&1 < /dev/null &
      sleep 2
    fi
  fi
  # 识别节点：赛题要用（camera 模式），没在跑就补
  if ! pgrep -f '[q]r_vision_node' >/dev/null 2>&1; then
    sleep 20
    if ! pgrep -f '[q]r_vision_node' >/dev/null 2>&1; then
      log "qr_vision_node 消失 20s，重启"
      setsid nohup python3 -u /home/orangepi/ros2_ws/install/qr_vision/lib/qr_vision/qr_vision_node.py \
        --ros-args -p mode:=camera -p device:=/dev/video0 >> /tmp/qr.log 2>&1 < /dev/null &
      sleep 3
    fi
  fi
  # 任务节点 / 地面站桥：不在就补（控制台自启有时起不来它们的子进程）
  if ! pgrep -f '[m]ission_node.py' >/dev/null 2>&1; then
    log "mission_node 不在，重启"
    setsid nohup timeout 7200 python3 -u /home/orangepi/diag/mission_node.py \
      --ros-args -p route_file:=/tmp/route_mission.yaml >> /tmp/mission.log 2>&1 < /dev/null &
    sleep 2
  fi
  if ! pgrep -f '[g]s_bridge.py' >/dev/null 2>&1; then
    log "gs_bridge 不在，重启"
    setsid nohup timeout 7200 python3 -u /home/orangepi/diag/gs_bridge.py \
      --ros-args -p gs_ip:=$(cat "$HOME/.gs_ip" 2>/dev/null || echo 192.168.137.1) \
      >> /tmp/gs_bridge.log 2>&1 < /dev/null &
    sleep 2
  fi
  # dead_reckon：**只在地面上补**（空中重启会让位姿参考跳变 → 位置环会猛冲）
  if ! pgrep -f '[d]ead_reckon.py' >/dev/null 2>&1; then
    h=$(curl -s -m 3 localhost:8090/api/status 2>/dev/null \
        | sed -n 's/.*"height": *\([0-9]*\).*/\1/p' | head -1)
    if [ -n "$h" ] && [ "$h" -lt 15 ]; then
      log "dead_reckon 不在且飞机在地面(h=${h}cm)，重启（位姿归零）"
      setsid nohup timeout 7200 python3 -u /home/orangepi/diag/dead_reckon.py \
        --ros-args -p source:=raw -p axis_swap:=false -p yaw_from_tf:=true \
        -p yaw_lpf_alpha:=0.03 >> /tmp/dead_reckon.log 2>&1 < /dev/null &
      sleep 3
    fi
  fi
  # ★ 关键链路节点（2026-10-06 加）：它们一掉，"注入飞控的速度反馈"就可能变成 0，
  #   飞控速度环失去反馈 → 速度指令退化成加速度指令 → **越飘越快**（实测飞走 3.5m）。
  #   实测现象：起飞后约 8 秒开始往一侧飘且越飘越快。
  MISS=""
  for n in velocity_filter.py cartographer_node uart_to_stm32 position_pid_controller; do
    pgrep -f "[${n:0:1}]${n:1}" >/dev/null 2>&1 || MISS="$MISS $n"
  done
  if [ -n "$MISS" ]; then
    h=$(curl -s -m 3 localhost:8090/api/status 2>/dev/null \
        | sed -n 's/.*"height": *\([0-9]*\).*/\1/p' | head -1)
    if [ -n "$h" ] && [ "$h" -lt 15 ]; then
      log "关键节点缺失:$MISS（在地面 h=${h}）→ 重启链路"
      bash /home/orangepi/diag/restart_link.sh >> /tmp/watchdog.log 2>&1
      sleep 8
    else
      log "⚠⚠ 关键节点缺失:$MISS 且高度=${h}cm → 立即发降落（0x66[2]）"
      curl -s -m 5 -X POST localhost:8090/api/land >/dev/null 2>&1
      sleep 2
    fi
  fi
  sleep 5
done

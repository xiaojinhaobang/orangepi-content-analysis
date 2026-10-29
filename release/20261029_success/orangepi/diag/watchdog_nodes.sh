#!/usr/bin/env bash
# 看住"掉了就飞不了"的节点。
#
# ★ v3（2026-10-06）：修掉"重复实例"竞态。
#   链路现在由 flight-link.service 前台托管（launch 里已有 body_vel_adapter /
#   dead_reckon / velocity_filter / cartographer / uart / position_pid）。
#   如果看门狗再单独起一个同名节点，就会出现**两个实例同时发 tf / 指令**
#   （实测 dead_reckon 出过 2 个，位姿参考会跳变）。
#   v3 规则：
#     · 服务在跑时，**归 launch 管的节点**缺失 → 一律 `restart flight-link.service`
#       （地面才重建；空中直接发降落，避免半空中重置位姿参考）
#     · 只有**不归 launch 管**的节点（qr_vision / mission_node / gs_bridge）才单独补
#
# 用法： setsid nohup bash ~/diag/watchdog_nodes.sh > /tmp/watchdog.log 2>&1 < /dev/null &
source /opt/ros/humble/setup.bash
source /home/orangepi/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=6

log() { echo "$(date '+%H:%M:%S') $*"; }

# ★ 2026-10-06：开机宽限 90 秒。
#   实测坑：开机后 launch 的节点是**逐个**起来的（间隔 0~3 秒），看门狗如果在这
#   中间看到某个节点还没出现，就会判"链路节点缺失"→ `restart flight-link.service`
#   → 在冷启动的负载尖峰上**再叠加一次全量重启** → load 冲到 10+，板子被拖垮
#   （实测日志：14:18:18 重建链路，紧接着 SSH 就进不去了）。
_up=$(cut -d. -f1 /proc/uptime 2>/dev/null || echo 999)
case "$_up" in ''|*[!0-9]*) _up=999 ;; esac
if [ "$_up" -lt 90 ]; then
  log "开机宽限 $((90 - _up)) 秒（等 launch 节点起齐，避免启动期误判重建）"
  sleep $((90 - _up))
fi

svc_alive() { systemctl is-active --quiet flight-link.service 2>/dev/null; }

height_cm() {
  curl -s -m 3 localhost:8090/api/status 2>/dev/null \
    | sed -n 's/.*"height": *\([0-9]*\).*/\1/p' | head -1
}

restart_link_service() {
  if sudo -n systemctl restart flight-link.service 2>/dev/null; then
    log "已通过 flight-link.service 重建链路"
    return 0
  fi
  log "⚠ 无免密权限重启 flight-link.service（检查 /etc/sudoers.d/flight-link）"
  return 1
}

while true; do
  # ---- 归 flight-link.service 管的节点：缺了就让服务整条重建 ----
  MISS=""
  for n in body_vel_adapter.py dead_reckon.py velocity_filter.py cartographer_node \
           uart_to_stm32 position_pid_controller; do
    pgrep -f "[${n:0:1}]${n:1}" >/dev/null 2>&1 || MISS="$MISS $n"
  done
  if [ -n "$MISS" ]; then
    if svc_alive; then
      # 先等一会儿，避开"服务正在重建"的窗口，别和 launch 抢
      sleep 20
      MISS2=""
      for n in body_vel_adapter.py dead_reckon.py velocity_filter.py cartographer_node \
               uart_to_stm32 position_pid_controller; do
        pgrep -f "[${n:0:1}]${n:1}" >/dev/null 2>&1 || MISS2="$MISS2 $n"
      done
      if [ -n "$MISS2" ]; then
        h=$(height_cm)
        if [ -n "$h" ] && [ "$h" -lt 15 ]; then
          log "链路节点缺失:$MISS2（地面 h=${h}）→ 重建 flight-link 服务"
          restart_link_service
          sleep 10
        else
          log "⚠⚠ 链路节点缺失:$MISS2 且高度=${h}cm → 立即发降落（0x66[2]）"
          curl -s -m 5 -X POST localhost:8090/api/land >/dev/null 2>&1
          sleep 2
        fi
      fi
    else
      log "flight-link 服务不 active，跳过（$MISS）"
    fi
  fi

  # ---- 不归 launch 管的节点：仍由看门狗单独补 ----
  # ★ 2026-10-06：**不再常驻补启 qr_vision**。实测它常驻吃 116% CPU（一个半核），
  #   是整条链路最重的进程，而它只在"扫描货位"那段时间有用。
  #   现在由 mission_node 按需启动/停止；这里只保留"任务期间崩了补一次"的兜底。
  #   ⚠ 判据必须用标志文件 /tmp/mission_active —— mission_node 本身是**常驻**的
  #     （一直在等 /gs/start），拿"它存在"当判据会把 qr_vision 又变成常驻
  #     （实测踩过：load 因此又冲到 4.5）。
  if [ -f /tmp/mission_active ]; then
    if ! pgrep -f '[q]r_vision_node' >/dev/null 2>&1; then
      log "任务中 qr_vision 不在，补启一次"
      setsid nohup python3 -u /home/orangepi/ros2_ws/install/qr_vision/lib/qr_vision/qr_vision_node.py \
        --ros-args -p mode:=camera -p device:=/dev/video0 >> /tmp/qr.log 2>&1 < /dev/null &
      sleep 3
    fi
  fi
  # ★ 2026-10-06：**看门狗不再管 mission_node**。原因有两个：
  #   1) 它重启时**不带 subset** → 会把"只飞 B1-B3"变成"飞全 12 格"；
  #   2) 实测 mission_node 一旦崩溃（这板子随机段错误），看门狗每 5 秒重启一次
  #      → 每个 Python+rclpy 启动都是重活 → **load 冲到 11**，板子被自己打死。
  #   mission_node 改由 `bash ~/diag/start_mission_subset.sh B1,B2,B3` 显式启动，
  #   控制台页面上也有对应入口。
  # ★ 2026-10-06：gs_bridge 也改按需（吃 ~43% 一个核）；要开就 `touch /tmp/gs_enable`
  if [ -f /tmp/gs_enable ] && ! pgrep -f '[g]s_bridge.py' >/dev/null 2>&1; then
    log "gs_bridge 不在，重启"
    setsid nohup timeout 7200 python3 -u /home/orangepi/diag/gs_bridge.py \
      --ros-args -p gs_ip:=$(cat "$HOME/.gs_ip" 2>/dev/null || echo 192.168.137.1) \
      >> /tmp/gs_bridge.log 2>&1 < /dev/null &
    sleep 2
  fi

  sleep 5
done

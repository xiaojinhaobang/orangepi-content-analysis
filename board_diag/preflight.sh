#!/usr/bin/env bash
# preflight.sh — 上电自检：雷达 / 飞控链路 / 飞控状态 / tf
# 用法: ~/diag/preflight.sh    退出码 0 = 全部通过
# 说明: /fc_link_len 读的是"飞控最后收到的那条上位机帧的长度"，是历史回执不是心跳；
#       链路是否在世由 /fc_status(0xF1 流) 是否在刷新决定。

source /opt/ros/humble/setup.bash
source /home/orangepi/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-6}"

ok=0; warn=0
pass(){ printf '  [PASS] %s\n' "$1"; ok=$((ok+1)); }
bad(){  printf '  [FAIL] %s\n' "$1"; warn=$((warn+1)); }

echo "===== preflight $(date '+%F %T')  ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ====="

echo "[1] 雷达 /scan"
if pgrep -f bluesea2_node >/dev/null; then
  echo "  driver: 运行中 pid=$(pgrep -f bluesea2_node | head -1)"
  rate=$(timeout 8 ros2 topic hz /scan 2>/dev/null | grep -m1 'average rate' | awk '{print $3}')
  if [ -n "${rate:-}" ]; then pass "/scan ${rate} Hz"; else bad "/scan 无数据（驱动在跑但不发话题）"; fi
else
  bad "bluesea2_node 未运行 → 执行 ~/radar_start.sh"
fi

echo "[2] 飞控链路 /dev/ttyS3"
if pgrep -f uart_to_stm32_node >/dev/null; then
  echo "  node: 运行中 pid=$(pgrep -f uart_to_stm32_node | head -1)"
  len=$(timeout 8 ros2 topic echo /fc_link_len --once --field data 2>/dev/null | grep -E '^-?[0-9]+$' | head -1)
  case "${len:-}" in
    8) pass "回执 LEN=8（飞控收到 0x31 速度指令）";;
    6) pass "回执 LEN=6（飞控收到 0x32 速度观测）";;
    '') bad "/fc_link_len 无数据 → 飞控没回帧（查上电 / TX-RX 交叉 / 共地）";;
    *)  bad "回执 LEN=${len}（非 6/8，核对发出帧的长度）";;
  esac
else
  bad "uart_to_stm32_node 未运行 → ros2 run uart_to_stm32 uart_to_stm32_node"
fi

echo "[3] 飞控状态 /fc_status"
raw=$(timeout 8 ros2 topic echo /fc_status --once --field data 2>/dev/null)
st=$(printf '%s' "$raw" | tr -d '\n' | sed 's/.*\[//; s/\].*//' | tr ',' ' ')
set -- ${st:-}
if [ $# -ge 7 ]; then
  printf '  mode_sta=%s unlock_sta=%s mode_cmd=%s unlock_cmd=%s cmdCID=%s CMD0=%s CMD1=%s\n' "$1" "$2" "$3" "$4" "$5" "$6" "$7"
  case "$2" in
    0) pass "未解锁（台架安全）";;
    *) bad "已解锁 unlock_sta=$2 —— 台架/上桨前禁止发速度指令";;
  esac
else
  bad "/fc_status 无数据（需 uart_to_stm32_node 在跑且飞控回帧）"
fi

echo "[4] tf map->laser_link（缺了 0x32 会静默不发）"
out=$(timeout 8 ros2 run tf2_ros tf2_echo map laser_link 2>&1)
case "$out" in
  *Translation*) pass "tf 可用";;
  *) bad "tf 不可用 → /velocity_map 不会转成 0x32，速度观测静默失效";;
esac

echo "===== 结果: PASS=${ok}  FAIL=${warn} ====="
[ "${warn}" -eq 0 ]

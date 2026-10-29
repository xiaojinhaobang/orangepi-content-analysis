#!/usr/bin/env bash
# 起飞前一键链路自检（快）——把原来"好几条 ros2 命令 + 手动判断"压成一条，
# 并且**自动补起 cartographer**（2026-10-14 实测：开机两次它都没起来，
# 一崩 /velocity_map 就断了 → 位姿冻结 → 任务必废）。
#
# 用法: bash ~/diag/ensure_link.sh
# 退出码: 0 = READY（可以叫飞机解锁了）  1 = NOT_READY
# 注意：这里**不能**用 `set -u` —— 后面 source /opt/ros/.../setup.bash 时有未绑定
# 变量，set -u 会让脚本在那一步直接退出（2026-10-14 实测踩到）。
ok=1
say() { echo "[ensure_link] $*"; }

for s in flight-link flight-stack flight-recorder radar-lidar laser-driver; do
  a=$(systemctl is-active "$s")
  [ "$a" = active ] || { say "❌ 服务 $s = $a"; ok=0; }
done

if [ "$(pgrep -c cartographer)" = 0 ]; then
  say "cartographer 没在跑 → 重启 flight-link（免密）"
  sudo -n systemctl restart flight-link.service
  for _ in $(seq 1 20); do
    sleep 3
    [ "$(pgrep -c cartographer)" != 0 ] && break
  done
fi
c=$(pgrep -c cartographer)
n=$(pgrep -c -f 'uart_to_stm32|body_vel_adapter|position_pid_controller|dead_reckon|velocity_filter')
say "cartographer=$c   链路进程=$n/5"
[ "$c" = 0 ] && ok=0
[ "$n" -lt 4 ] && ok=0

# 只做一次 ROS 调用（每条 ros2 命令要 3~6 秒，能省就省）
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/orangepi/ros2_ws/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=6
R=$(timeout 8 ros2 topic hz /velocity_map 2>&1 | grep -o 'average rate: [0-9.]*' | head -1)
if [ -z "$R" ]; then
  say "❌ /velocity_map 没有发布者 → SLAM 没在出速度"
  ok=0
else
  say "✅ $R（雷达速度源在出数）"
fi

if [ "$ok" = 1 ]; then
  say "✅ READY —— 可以叫飞机解锁了"
  exit 0
fi
say "❌ NOT_READY（见上；再不行就断电重启）"
exit 1

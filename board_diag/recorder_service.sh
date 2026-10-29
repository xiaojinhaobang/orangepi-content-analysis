#!/usr/bin/env bash
# 试飞记录器（给 systemd 服务调用）：每次启动开一个新文件，崩了由 systemd 重启。
# 目的：flight_log.py 会被板子的随机崩溃杀掉，必须由 systemd 盯着。
source /opt/ros/humble/setup.bash
source /home/orangepi/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=6
# ★ 2026-10-07：板子无 RTC —— 每次上电时钟回到同一时刻 → 文件名相同 → 旧记录被**覆盖**。
#   实测已丢过一整顿数据（rec_20261007_191704.log 里 ID65=1100 那场被覆盖掉了）。
#   目标已存在就加 -1/-2/-3…，绝不覆盖任何已有记录。
BASE="$HOME/traces/rec_$(date +%Y%m%d_%H%M%S)"
OUT="${BASE}.log"
i=1
while [ -e "$OUT" ]; do
  OUT="${BASE}-${i}.log"
  i=$((i + 1))
done
echo ">>> recorder_service 启动 → $OUT"
exec python3 -u /home/orangepi/diag/flight_log.py 7200 "$OUT"

#!/usr/bin/env bash
# 重启看门狗（改了 watchdog_nodes.sh 之后必须这样重启才生效：老进程还跑着旧代码）。
# 必须写成脚本：直接在命令行里同时出现"看门狗文件名"+匹配模式，pkill/awk 会把自己杀掉。
pkill -f '[w]atchdog_nodes.sh' 2>/dev/null
sleep 2
setsid nohup bash /home/orangepi/diag/watchdog_nodes.sh > /tmp/watchdog.log 2>&1 < /dev/null &
sleep 3
echo "看门狗进程数: $(pgrep -fc '[w]atchdog_nodes.sh')"
# 顺手清理重复的适配节点（只保留最早那个由 launch 起的）
mapfile -t pids < <(pgrep -f '[b]ody_vel_adapter.py')
if [ "${#pids[@]}" -gt 1 ]; then
  for p in "${pids[@]:1}"; do kill -9 "$p" 2>/dev/null; done
  echo "清理了 $(( ${#pids[@]} - 1 )) 个多余的 body_vel_adapter"
fi
echo "适配节点数: $(pgrep -fc '[b]ody_vel_adapter.py')"

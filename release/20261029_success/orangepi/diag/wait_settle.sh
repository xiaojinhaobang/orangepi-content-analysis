#!/usr/bin/env bash
# 开机时所有服务一起起会造成资源尖峰 -> 多进程连环 SEGV（实测一次崩 5 个）。
# 这里等 1 分钟负载落到 <=2 再放行（平稳开机约 15~25s），最多等 45s 兜底（2026-10-19：90s→45s，阈值 2→4，开机实测省 40s）。
for i in $(seq 1 10); do
  L=$(cut -d" " -f1 /proc/loadavg)
  L=${L%%.*}
  [ "$L" -le 6 ] && exit 0
  sleep 3
done
exit 0

#!/bin/bash
# 起飞前体检：一条命令打印 PASS/FAIL。不改变任何状态，只读取。
set +u
export ROS_DOMAIN_ID=6
source /opt/ros/humble/setup.bash
source /home/orangepi/cartographer_ws/install/setup.bash 2>/dev/null
source /home/orangepi/ros2_ws/install/setup.bash 2>/dev/null

fail=0
ok()   { echo "  PASS  $1"; }
bad()  { echo "  FAIL  $1"; fail=$((fail+1)); }

rate() { timeout 4 ros2 topic hz "$1" 2>/dev/null | grep -m1 "average rate" | awk '{print $3}'; }
tfv()  { timeout 3 ros2 run tf2_ros tf2_echo "$1" "$2" 2>/dev/null | grep -m1 Translation; }
pubs() { timeout 5 ros2 topic info "$1" --verbose 2>/dev/null | grep "Publisher count" | awk '{print $3}'; }

echo "=== 1. 节点 ==="
nodes=$(timeout 8 ros2 node list 2>/dev/null)
for n in /cartographer_node /dead_reckon /position_pid_controller /uart_to_stm32 /mission_node; do
  echo "$nodes" | grep -q "^$n$" && ok "$n 在" || bad "$n 缺失"
done

echo "=== 2. 话题频率 ==="
for t in /scan /fc_vel /height; do
  r=$(rate $t); [ -n "$r" ] && ok "$t $r Hz" || bad "$t 无数据"
done

echo "=== 3. TF 链 ==="
for pair in "map odom" "odom base_link" "map base_link" "map dr_base"; do
  v=$(tfv $pair); if [ -n "$v" ]; then ok "$pair $v"; else bad "$pair 取不到"; fi
done

echo "=== 4. 单一发布者 ==="
p=$(pubs /target_velocity);  [ "$p" = "1" ] && ok "/target_velocity 1 个" || bad "/target_velocity $p 个（多于 1 会打架）"
# /target_position 的本质风险是"有人在持续发目标"，不是"有节点声明了 publisher"
# （flight_console 只在网页下令时发一次；mission 在跑航线时 10Hz）。
r=$(rate /target_position)
if [ -z "$r" ]; then ok "/target_position 起飞前无持续发布"; else bad "/target_position 正在以 $r Hz 发布（有人在发目标）"; fi
echo "  INFO  /target_position publisher 数 $(pubs /target_position)"

echo "=== 5. 关键参数 ==="
pt=$(ros2 param get /dead_reckon passthrough 2>/dev/null | awk '{print $NF}')
ge=$(ros2 param get /dead_reckon gate_enable 2>/dev/null | awk '{print $NF}')
[ "$pt" = "True" ] && ok "passthrough=True" || bad "passthrough=$pt"
[ "$ge" = "False" ] && ok "gate_enable=False" || bad "gate_enable=$ge"

echo "=== 6. 多余进程 ==="
for pat in pos_hold auto_guard flight_console; do
  c=$(pgrep -fc "$pat" 2>/dev/null || echo 0)
  echo "  INFO  $pat 进程数 $c"
done
pgrep -f scan_shift >/dev/null 2>&1 && bad "scan_shift 残留" || ok "无 scan_shift"
pgrep -f "ros2 bag play" >/dev/null 2>&1 && bad "bag play 残留" || ok "无 bag play"

echo "=== 结论 ==="
if [ $fail -eq 0 ]; then echo "全部通过（$fail 项失败）"; else echo "$fail 项失败，不要起飞"; fi

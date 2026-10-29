#!/usr/bin/env bash
# 起飞就绪自检（换电池后跑这一条，看到 READY 就能飞，不用死等）
# 用法: bash ~/diag/ready_check.sh
#
# 判据（任一条不满足就 NOT_READY 并列出原因）：
#   1. 链路 6 个关键节点都在
#   2. mission_node（等 START）+ gs_bridge（地面站桥）都在
#   3. 本次开机没有出现过进程崩溃（崩溃风暴时禁止起飞）
#   4. 雷达 /scan 有节拍（说明雷达+cartographer 真的在转）
source /opt/ros/humble/setup.bash
source /home/orangepi/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=6

ok=1
reasons=()

# 1) 链路节点
n=$(pgrep -c -f "cartographer_node|velocity_filter.py|dead_reckon.py|uart_to_stm32|position_pid_controller|body_vel_adapter" 2>/dev/null || echo 0)
[ "$n" -ge 6 ] || { ok=0; reasons+=("链路节点只有 $n/6 个"); }

# 2) 任务侧
pgrep -f "[m]ission_node.py" >/dev/null 2>&1 || { ok=0; reasons+=("mission_node 没在跑"); }
pgrep -f "[g]s_bridge.py" >/dev/null 2>&1 || { ok=0; reasons+=("gs_bridge 没在跑"); }
# 2b) ★ 2026-10-07(晚) 实测踩坑：qr_vision 只有**手动**看门狗才会拉起，
#     板子重启后它不在——已经出现过一次"自检全绿但扫码全空"的隐患。
#     laser_driver 由服务拉起，这里一起确认。
pgrep -f "[q]r_vision_node" >/dev/null 2>&1 || { ok=0; reasons+=("qr_vision_node 没在跑（扫码会全空）→ 手动起: 见下方提示"); }
pgrep -f "[l]aser_driver.py" >/dev/null 2>&1 || { ok=0; reasons+=("laser_driver 没在跑（激光打不了点）"); }

# 3) 崩溃次数：**只算最近 3 分钟**（开机那会儿或维护性重建时的旧崩溃不该拦住飞行）
crashes=$(echo orangepi | sudo -S journalctl -b 0 --since "-3min" --no-pager 2>/dev/null | grep -cE "process has died|dumped core|status=11")
crashes_boot=$(echo orangepi | sudo -S journalctl -b 0 --no-pager 2>/dev/null | grep -cE "process has died|dumped core|status=11")
[ "${crashes:-0}" -eq 0 ] || { ok=0; reasons+=("最近 3 分钟崩了 ${crashes} 次 → 请重启板子再飞"); }

# 4) 雷达节拍
hz=$(timeout 7 ros2 topic hz /scan 2>/dev/null | grep -m1 "average rate" | awk '{print $NF}')
hz_int=${hz%%.*}
if [ -z "$hz" ] || [ "${hz_int:-0}" -lt 5 ]; then
  ok=0
  reasons+=("雷达 /scan 没有节拍（读到 '${hz:-无}'）")
fi

# 5) 飞控
fc=$(timeout 7 ros2 topic echo --once /fc_status 2>/dev/null | grep -A4 "data:" | grep -oE "[0-9]+" | tr "\n" " " | awk '{print $1" "$2}')

echo "-------- 起飞就绪自检 --------"
echo "开机时长 : $(uptime -p 2>/dev/null | sed 's/up //')"
echo "链路节点 : $n/6"
echo "扫码/激光 : qr_vision=$(pgrep -c -f '[q]r_vision_node' 2>/dev/null || echo 0)  laser=$(pgrep -c -f '[l]aser_driver.py' 2>/dev/null || echo 0)   (各要 1)"
echo "雷达节拍 : ${hz:-无} Hz"
echo "飞控状态 : mode/unlock = ${fc:-读不到}   (要解锁后是 2 1)"
echo "最近3分钟崩溃 : ${crashes:-?} 次（本次开机累计 ${crashes_boot:-?} 次）"
if [ "$ok" -eq 1 ]; then
  echo "结果 : READY ✅ 可以起任务"
else
  echo "结果 : NOT_READY ❌"
  for r in "${reasons[@]}"; do echo "        · $r"; done
  echo "提示 : qr_vision 不在时手动起一条："
  echo "        source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=6;"
  echo "        setsid nohup python3 -u ~/ros2_ws/install/qr_vision/lib/qr_vision/qr_vision_node.py --ros-args -p mode:=camera -p device:=/dev/video0 >> /tmp/qr.log 2>&1 < /dev/null &"
fi
[ "$ok" -eq 1 ]

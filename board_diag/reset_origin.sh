#!/usr/bin/env bash
# 把导航原点重置到"飞机当前位置"。
# 什么时候要跑：任何搬动飞机之后（dead_reckon 会把搬动当真实飞行积分进去，
# 不重置的话它以为飞机在别处，一起飞就会先飞过去找原点）。
#
# 2026-09-27 修复：必须带 -p source:=raw。
# 方案1 的 velocity_filter 处于 open_loop，/velocity_map_filtered 恒为 0，
# 不带参数重启会退回默认 source=filtered -> 位姿永久冻结 -> 位置环以为飞机没动。
set -e
source /opt/ros/humble/setup.bash
source /home/orangepi/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-6}"
# 注意：同时杀掉 launch 里那份，否则两个节点会同时发布 map->dr_base（tf 冲突）
pkill -f dead_reckon.py || true
sleep 1
setsid nohup timeout 7200 python3 -u /home/orangepi/diag/dead_reckon.py \
  --ros-args -p source:=raw -p axis_swap:=false -p yaw_from_tf:=true \
  -p yaw_lpf_alpha:=0.03 \
  > /tmp/dead_reckon.log 2>&1 < /dev/null &
sleep 6
echo "--- 重置后的位姿（应≈0）---"
timeout 8 ros2 topic echo /dead_reckon/pose --once --field pose.position 2>/dev/null | grep -E "x:|y:" || echo "(拿不到位姿)"
echo "--- 航点执行器看到的误差 ---"
timeout 8 ros2 topic echo /waypoint_nav/status --once 2>/dev/null | grep -v "^---" | tail -1

#!/usr/bin/env bash
# save_map.sh <名字>  — 存 Cartographer 地图（pbstream）+ 记录存档时位姿
# 用法: ~/diag/save_map.sh site1
# 说明: 第 1 步存"含未完成子图"的快照（先保命），第 3 步结束轨迹后再存最终存档。

name="${1:-}"
if [ -z "$name" ]; then echo "用法: $0 <名字，如 site1>"; exit 2; fi

dir="$HOME/maps"; mkdir -p "$dir"
raw="$dir/${name}-raw.pbstream"
fin="$dir/${name}.pbstream"

source /opt/ros/humble/setup.bash
source ~/cartographer_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-6}"

echo "[1/4] 快照（含未完成子图）→ $raw"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '$raw', include_unfinished_submaps: true}" 2>&1 | tail -1

echo "[2/4] 结束轨迹 0"
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory \
  "{trajectory_id: 0}" 2>&1 | tail -1

echo "[3/4] 最终存档 → $fin"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '$fin', include_unfinished_submaps: false}" 2>&1 | tail -1

echo "[4/4] 存档时雷达位姿（map->laser_link）"
timeout 8 ros2 run tf2_ros tf2_echo map laser_link 2>&1 | grep -A3 -m1 "Translation" || echo "  (拿不到位姿，cartographer 可能已停)"

echo "--- 文件 ---"
ls -l "$raw" "$fin" 2>/dev/null || echo "  存档失败，检查 /write_state 服务调用输出"

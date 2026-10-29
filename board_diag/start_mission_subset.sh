#!/usr/bin/env bash
# 重启任务节点，只飞指定货位（半面/单格测试用）。
#   用法： bash ~/diag/start_mission_subset.sh B1,B2,B3     # 只飞 B 面上排三格
#          bash ~/diag/start_mission_subset.sh              # 不传=整条 12 货位
# 说明：必须写成脚本再调用 —— 直接在命令行里用 pkill 匹配 mission_node 会把自己的 shell 一起杀掉。
SUBSET="${1:-}"

# 清掉旧的（含 timeout 包装）
pkill -f '[m]ission_node.py' 2>/dev/null
sleep 2

source /opt/ros/humble/setup.bash
source /home/orangepi/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=6
set -u

if [ -n "$SUBSET" ]; then
  setsid nohup timeout 7200 python3 -u /home/orangepi/diag/mission_node.py \
    --ros-args -p route_file:=/tmp/route_mission.yaml -p subset:="$SUBSET" \
    > /tmp/mission.log 2>&1 < /dev/null &
  echo "任务节点已启动，只飞: $SUBSET"
else
  setsid nohup timeout 7200 python3 -u /home/orangepi/diag/mission_node.py \
    --ros-args -p route_file:=/tmp/route_mission.yaml \
    > /tmp/mission.log 2>&1 < /dev/null &
  echo "任务节点已启动，整条 12 货位"
fi
sleep 5
echo "--- 进程 ---"
pgrep -af '.py --ros-args -p route_file' | head -2
echo "--- 日志 ---"
tail -3 /tmp/mission.log

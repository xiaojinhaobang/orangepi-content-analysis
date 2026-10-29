#!/bin/bash
# A/B: replay the same /scan bag through two cartographer configs, record map->laser_link.
# Usage: carto_ab.sh <bag_dir> [seconds]
# Requires on the board: ~/diag/carto_cfg (ours), ~/diag/carto_their (theirs),
#                        ~/diag/tf_velocity_compare.py, ROS humble sourced.
set +u
BAG=${1:?bag dir}
SECS=${2:-18}
OUR=/home/orangepi/diag/carto_cfg
THEIR=/home/orangepi/diag/carto_their
source /home/orangepi/cartographer_ws/install/setup.bash

run_one () {
  local name=$1 dir=$2
  echo "=== $name ($dir) ==="
  pkill -f "[c]artographer_node" 2>/dev/null
  pkill -f "[s]tatic_transform_publisher" 2>/dev/null
  sleep 1
  ros2 run cartographer_ros cartographer_node \
    -configuration_directory "$dir" \
    -configuration_basename amphi.lua > "/tmp/ab_${name}_carto.log" 2>&1 &
  local cpid=$!
  sleep 2
  ros2 bag play "$BAG" --rate 1.0 > "/tmp/ab_${name}_play.log" 2>&1 &
  python3 /home/orangepi/diag/tf_velocity_compare.py \
    --duration "$SECS" --out "/tmp/ab_${name}.csv" >/dev/null 2>&1
  pkill -f "[r]os2 bag play" 2>/dev/null
  kill "$cpid" 2>/dev/null
  pkill -f "[s]tatic_transform_publisher" 2>/dev/null
  sleep 0.5
  echo "样本行数: $(wc -l < /tmp/ab_${name}.csv 2>/dev/null)"
  head -2 "/tmp/ab_${name}.csv" 2>/dev/null
  tail -1 "/tmp/ab_${name}.csv" 2>/dev/null
}

run_one ours "$OUR"
run_one theirs "$THEIR"
echo "DONE  → /tmp/ab_ours.csv  /tmp/ab_theirs.csv"

#!/bin/bash
# Synthetic-motion test: replay a static bag as if the robot moved at a known speed.
# Cartographer runs in ROS_DOMAIN_ID=7 so the live stack (domain 6) is untouched.
set +u
BAG=${1:-/tmp/ab_bag}
SECS=${2:-20}
source /opt/ros/humble/setup.bash
source /home/orangepi/cartographer_ws/install/setup.bash
export ROS_DOMAIN_ID=7

run_case () {
  name=$1; v=$2; dir=$3; use_shift=$4
  echo "=== case $name  v=$v cm/s  dir=$dir deg  shifter=$use_shift ==="
  pkill -f "[c]artographer_node" 2>/dev/null
  pkill -f "[r]os2 bag play" 2>/dev/null
  pkill -f "[s]can_shift" 2>/dev/null
  sleep 1
  if [ "$use_shift" = "1" ]; then
    ros2 run cartographer_ros cartographer_node \
      -configuration_directory /home/orangepi/diag/carto_cfg \
      -configuration_basename amphi.lua \
      --ros-args -r scan:=/scan_moved > "/tmp/syn_${name}_carto.log" 2>&1 &
  else
    ros2 run cartographer_ros cartographer_node \
      -configuration_directory /home/orangepi/diag/carto_cfg \
      -configuration_basename amphi.lua > "/tmp/syn_${name}_carto.log" 2>&1 &
  fi
  sleep 2
  if [ "$use_shift" = "1" ]; then
    python3 /tmp/scan_shift.py --v "$v" --dir "$dir" > "/tmp/syn_${name}_shift.log" 2>&1 &
  fi
  ros2 bag play "$BAG" > "/tmp/syn_${name}_play.log" 2>&1 &
  python3 /tmp/tf_height_rec.py --duration "$SECS" --out "/tmp/syn_${name}.csv" >/dev/null 2>&1
  echo "  rows: $(wc -l < /tmp/syn_${name}.csv 2>/dev/null)"
  tail -1 "/tmp/syn_${name}.csv" 2>/dev/null
}

run_case base 0 0 0
run_case shift0 0 0 1
run_case open20 20 0 1
run_case side20 20 90 1
pkill -f "[c]artographer_node" 2>/dev/null
pkill -f "[r]os2 bag play" 2>/dev/null
pkill -f "[s]can_shift" 2>/dev/null
echo DONE

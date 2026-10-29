#!/usr/bin/env bash
# 位置环开关 —— 给"纯飞控悬停"用（我们完全不参与）。
#   bash ~/diag/pos_loop.sh off     关：发 /active_controller=0，我们退出
#   bash ~/diag/pos_loop.sh on      开：发 /active_controller=2，位置环激活
#   bash ~/diag/pos_loop.sh status  看 /target_velocity 还在不在发指令
#
# 依据：pid_controller.cpp:211 —— data==2 激活；==3 "Drone Stop"（发 0 速度）；
# 其他值（含 0）→ is_active_controller_=false，位置环不再发布任何速度指令。
source /opt/ros/humble/setup.bash
source /home/orangepi/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=6

case "${1:-status}" in
  off)
    ros2 topic pub -1 /active_controller std_msgs/msg/UInt8 "{data: 0}" >/dev/null 2>&1
    sleep 1
    echo "已发 /active_controller=0（我们的环退出）"
    bash "$0" status
    ;;
  on)
    ros2 topic pub -1 /active_controller std_msgs/msg/UInt8 "{data: 2}" >/dev/null 2>&1
    echo "已发 /active_controller=2（位置环激活）"
    ;;
  status)
    out=$(timeout 6 ros2 topic hz /target_velocity 2>&1 | grep -m1 "average rate")
    if [ -n "$out" ]; then
      echo "位置环【在发指令】$out   ← 这不是纯飞控（悬停自测/开始任务都会重新激活它）"
    else
      echo "位置环【已停】/target_velocity 无节拍   ← 纯飞控，飞控自己管"
    fi
    ;;
  *)
    echo "用法: $0 off|on|status"
    ;;
esac

#!/usr/bin/env bash
# 飞行控制台启动脚本（供桌面自启动调用）
set -e
source /opt/ros/humble/setup.bash
# ★ 2026-09-28：链路必须能解析 cartographer_ros —— 它在 **另一个** 工作空间里。
# 只 source ros2_ws 时 ros2 launch 会报 "package 'cartographer_ros' not found"
# （实测 18:28 起链路就是因此起不来）。这里把 .bashrc 里那三行显式补全。
source /home/orangepi/kian_ws/install/setup.bash
source /home/orangepi/cartographer_ws/install/setup.bash
source /home/orangepi/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=6
export ROS_LOCALHOST_ONLY=0
mkdir -p /home/orangepi/.local/state/flight_console
# GS_IP: 电脑(地面站)的 IP。填上后开机自启就自动把 gs_bridge 指向它并开始推状态；
# 留空则要等地面站先发一包才锁定地址（重启后会表现为"收不到状态"）。
# 2026-09-30：改用电脑「移动热点」的固定网关地址。板子插的 USB 网卡(wlx…)连电脑热点 DGS，
# 电脑在该网段的地址恒为 192.168.137.1 —— 不再随手机热点/网络变化而变。
GS_IP="${GS_IP:-192.168.137.1}"
# 网页版地面站：队友浏览器打开 http://<板子IP>:8081 即可（8080 已被别的服务占用）
# 判据用 "server.py --ros"：手动用相对路径（cd webgs && python3 server.py）起的也能认出来，
# 否则守卫失效、每次重启都会再拉一个（第二个会因端口占用直接崩）
# ★ 2026-10-06：网页地面站改成**按需启动**（实测吃 ~39% 一个核）。
#   要开：`touch /tmp/gs_enable && systemctl restart flight-stack`
#   平时不开，给飞行留出 CPU。
if [ -f /tmp/gs_enable ] && ! pgrep -f "[s]erver.py --ros" >/dev/null 2>&1; then
  setsid nohup python3 -u /home/orangepi/diag/webgs/server.py --ros --port 8081 \
    > /tmp/webgs.log 2>&1 < /dev/null &
fi
# 关键节点看门狗：body_vel_adapter 开机偶发 SIGSEGV、qr_vision 需要常驻
# ★ 2026-10-06：**不再自动起看门狗**。
#   实测它带来过两次大事故：①重启"缺 subset 的 mission_node"把 3 格测试变成飞 12 格；
#   ②mission_node 崩溃后它每 5 秒重启一次 → 每个 Python+rclpy 启动都是重活 →
#   load 冲到 11，板子被自己打死。
#   现在改为**手动启动**：`setsid nohup bash ~/diag/watchdog_nodes.sh > /tmp/watchdog.log 2>&1 &`
#   （链路本身有 flight-link.service 的 Restart=always 兜底，不依赖看门狗。）
# D 题地面站链路：mission_node 等 /gs/start；gs_bridge 把 UDP 8888 的帧转成 ROS 话题。
# ★ 2026-10-07：加到开机自启里，这样电脑一开地面站、点「开始任务」就能起飞
#   （不用先开浏览器控制台点一遍）。两个进程都是常驻轻负载：
#   mission_node 空转等 START；gs_bridge 只按 2Hz 推状态。
#   注意：mission_node 用默认 subset（空=整块 12 格）。
if ! pgrep -f "[m]ission_node.py" >/dev/null 2>&1; then
# [2026-10-11 调参期间停用：mission_node 会自己发指令干扰试飞]   setsid nohup bash /home/orangepi/diag/start_mission_subset.sh \
    > /tmp/mission_autostart.log 2>&1 < /dev/null &
fi
if ! pgrep -f "[g]s_bridge.py" >/dev/null 2>&1; then
  setsid nohup python3 -u /home/orangepi/diag/gs_bridge.py \
    --ros-args -p gs_ip:="$GS_IP" > /tmp/gs_bridge.log 2>&1 < /dev/null &
fi

exec python3 -u /home/orangepi/diag/flight_console.py --port 8090 \
  --gs-ip "$GS_IP" \
  >> /home/orangepi/.local/state/flight_console/console.log 2>&1

#!/usr/bin/env bash
# 停掉飞行栈的全部进程（测试 systemd 服务、或需要干净重启时用）
# 注意：必须写成脚本调用 —— 直接在命令行里用 pkill 匹配这些名字会把自己的 shell 一起杀掉。
pkill -f '[f]light_console.py' 2>/dev/null
pkill -f '[w]atchdog_nodes.sh' 2>/dev/null
pkill -f '[s]erver.py --ros' 2>/dev/null
pkill -f '[s]tage3_simple' 2>/dev/null
pkill -f '[c]artographer_node' 2>/dev/null
pkill -f '[v]elocity_filter.py' 2>/dev/null
pkill -f '[u]art_to_stm32_node' 2>/dev/null
pkill -f '[p]osition_pid_controller' 2>/dev/null
pkill -f '[d]ead_reckon.py' 2>/dev/null
pkill -f '[b]ody_vel_adapter.py' 2>/dev/null
pkill -f '[w]aypoint_nav.py' 2>/dev/null
pkill -f '[m]ission_node.py' 2>/dev/null
pkill -f '[g]s_bridge.py' 2>/dev/null
pkill -f '[q]r_vision_node' 2>/dev/null
sleep 2
echo "已停止。剩余相关进程："
pgrep -fc 'cartographer_node|uart_to_stm32|dead_reckon|body_vel_adapter|flight_console|gs_bridge' || echo 0

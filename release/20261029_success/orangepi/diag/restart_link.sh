#!/usr/bin/env bash
# 重启香橙派飞行链路：先停旧（含已崩残留），再用控制台带正确 gs_ip 起新的。
#
# 地面站(电脑)的 IP **自动识别**，不用再手改：
#   用法1   bash ~/restart_link.sh                  # 自动识别
#   用法2   bash ~/restart_link.sh 192.168.1.23     # 手动指定
#   用法3   GS_IP=192.168.1.23 bash ~/restart_link.sh
#
# 识别顺序：命令行参数 → 环境变量 GS_IP → 本次 SSH 的来源 IP → 当前登录会话来源 IP
#          → gs_bridge 上次锁定的地面站 IP → 上次存下的 ~/.gs_ip → 兜底
# 识别到的值会存进 ~/.gs_ip，下次断网也能用上次那台电脑。
set -u

detect_gs_ip() {
  local ip=""
  [ -n "${1:-}" ]      && ip="$1"
  [ -z "$ip" ] && [ -n "${GS_IP:-}" ] && ip="$GS_IP"
  # 专用链路优先：电脑开移动热点(DGS)、板子 USB 网卡连上后，本机就有 192.168.137.x，
  # 此时电脑(网关)地址恒为 192.168.137.1 —— 比任何"自动嗅探"都可靠。
  if [ -z "$ip" ] && ip -4 addr show 2>/dev/null | grep -q '192\.168\.137\.'; then
    ip="192.168.137.1"
  fi
  if [ -z "$ip" ] && [ -n "${SSH_CLIENT:-}" ]; then
    ip="${SSH_CLIENT%% *}"                       # ssh 进来跑脚本时，这个就是那台电脑
    case "$ip" in 127.*|::1|"") ip="" ;; esac
  fi
  if [ -z "$ip" ]; then
    ip=$(who 2>/dev/null | grep -o '([0-9.]*)' | tr -d '()' | grep -v '^127\.' | head -1)
  fi
  if [ -z "$ip" ] && [ -f /tmp/gs_bridge.log ]; then
    ip=$(grep -o '地面站=[0-9.]\+' /tmp/gs_bridge.log | tail -1 | cut -d= -f2)
    [ -z "$ip" ] && ip=$(grep -o '锁定地面站地址: [0-9.]\+' /tmp/gs_bridge.log \
                         | tail -1 | awk '{print $2}')
  fi
  [ -z "$ip" ] && [ -f "$HOME/.gs_ip" ] && ip=$(tr -d ' \n' < "$HOME/.gs_ip")
  [ -z "$ip" ] && ip="192.168.137.1"
  echo "$ip"
}

GS="$(detect_gs_ip "${1:-}")"
echo "$GS" > "$HOME/.gs_ip"
echo "=== 地面站(电脑) IP: $GS （已记到 ~/.gs_ip）==="
if ping -c 1 -W 2 "$GS" >/dev/null 2>&1; then
  echo "    可达 ✓"
else
  echo "    ⚠ ping 不通（对方开了防火墙也正常，UDP 回传一般不受影响）"
fi

echo
echo "=== 1. 停旧链路 ==="
pkill -f '[s]tage3_simple'
pkill -f '[c]artographer_node'
pkill -f '[v]elocity_filter.py'
pkill -f '[u]art_to_stm32_node'
pkill -f '[p]osition_pid_controller'
pkill -f '[d]ead_reckon.py'
pkill -f '[b]ody_vel_adapter.py'
pkill -f '[w]aypoint_nav.py'
pkill -f '[m]ission_node.py'
pkill -f '[g]s_bridge.py'
sleep 3
echo "--- 残留检查（应为空）---"
pgrep -af '[s]tage3_simple|[c]artographer_node|[u]art_to_stm32_node|[d]ead_reckon.py|[g]s_bridge.py|[m]ission_node.py' || echo "  （已清空）"

echo
echo "=== 2. 启动控制台（GS_IP=$GS，会自动起链路 + mission_node + gs_bridge）==="
export GS_IP="$GS"
setsid nohup bash /home/orangepi/diag/console_start.sh > /tmp/console_start.log 2>&1 < /dev/null &
sleep 30

echo
echo "=== 3. 端口 ==="
ss -tlnp 2>/dev/null | grep -e :8090 -e :8081 || echo "  （控制台端口未监听）"

echo
echo "=== 4. 进程 ==="
pgrep -af '[s]tage3_simple' | head -3
pgrep -af '[c]artographer_node' | head -2
pgrep -af '[u]art_to_stm32_node' | head -2
pgrep -af '[d]ead_reckon.py' | head -2
pgrep -af '[g]s_bridge.py' | head -2
pgrep -af '[m]ission_node.py' | head -2

echo
echo "=== 5. 链路日志尾部 ==="
tail -12 /tmp/stage3.log

echo
echo "=== 6. 控制台状态 ==="
curl -s --max-time 5 localhost:8090/api/status || echo "  （控制台无响应）"
echo

echo
echo "=== 7. 地面站桥指向 ==="
tail -2 /tmp/gs_bridge.log
echo "提示：如果这次识别到的 IP 不是你那台电脑（比如换了网还没重连），"
echo "      直接 bash ~/restart_link.sh <正确的IP> 再跑一次即可。"

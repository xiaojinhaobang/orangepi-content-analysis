#!/usr/bin/env bash
# 把板子的 USB 无线网卡，换绑到"任意一台地面站电脑"的移动热点上。
#
# 用法：
#   bash ~/diag/join_gs_hotspot.sh --list            # 先看能扫到哪些热点
#   bash ~/diag/join_gs_hotspot.sh <SSID> [密码]      # 不带密码 = 开放网络
#
# 约定：地面站电脑用 Windows「移动热点」，它的网关地址恒为 192.168.137.1，
#       所以板子固定 192.168.137.186 —— 无论换成谁的电脑，两边地址都不用改。
#
# 换人调试的完整流程见桌面《给队友-独立调试指南.md》
set -u

# 找 USB 网卡（NetworkManager 按 MAC 命名成 wlx…）
IFACE="$(ls /sys/class/net | grep -E '^wlx' | head -1)"
[ -z "$IFACE" ] && IFACE="$(ls /sys/class/net | grep -E '^wlan[1-9]' | head -1)"
if [ -z "$IFACE" ]; then
  echo "找不到 USB 无线网卡（wlx…）。插好网卡后重试。"; exit 1
fi

BRIDGE_IP="192.168.137.1"     # 电脑（Windows 移动热点网关）
BOARD_IP="192.168.137.186"    # 板子（固定）

if [ "${1:-}" = "--list" ] || [ $# -eq 0 ]; then
  echo "USB 网卡：$IFACE"
  echo "当前扫到的热点："
  nmcli -f SSID,SIGNAL,CHAN,SECURITY device wifi list ifname "$IFACE" | head -15
  if [ $# -eq 0 ]; then
    echo
    echo "用法： bash $0 <SSID> [密码]"
    echo "例：   bash $0 DGS dgs12345678"
  fi
  exit 0
fi

SSID="$1"; PASS="${2:-}"

echo "=== 1/3 连接热点 $SSID ==="
if [ -n "$PASS" ]; then
  nmcli device wifi connect "$SSID" password "$PASS" ifname "$IFACE" || exit 1
else
  nmcli device wifi connect "$SSID" ifname "$IFACE" || exit 1
fi

echo
echo "=== 2/3 固定地址：板子 $BOARD_IP / 电脑 $BRIDGE_IP ==="
nmcli connection modify "$SSID" \
  ipv4.method manual ipv4.addresses "$BOARD_IP/24" ipv4.gateway "$BRIDGE_IP" \
  ipv4.dns "223.5.5.5 $BRIDGE_IP" ipv4.route-metric 700 ipv6.method ignore \
  connection.autoconnect yes connection.autoconnect-priority 20
nmcli connection up "$SSID" >/dev/null 2>&1
sleep 3

echo
echo "=== 3/3 地面站桥指向 $BRIDGE_IP ==="
curl -s -m 5 -X POST localhost:8090/api/gs_bridge \
     -H 'Content-Type: application/json' -d "{\"gs_ip\":\"$BRIDGE_IP\"}" || true
echo

echo
echo "=== 结果 ==="
ip -br addr show "$IFACE"
ip route | grep default || true
echo
echo "电脑那边："
echo "  · 地面站「机载 IP」填  $BOARD_IP"
echo "  · 网页地面站          http://$BOARD_IP:8081"
echo "  · 飞行控制台          http://$BOARD_IP:8090"
echo "  · SSH                ssh orangepi@$BOARD_IP"

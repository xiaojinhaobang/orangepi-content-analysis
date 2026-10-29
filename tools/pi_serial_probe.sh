#!/bin/bash
# 香橙派串口现状：设备、设备树状态、占用者、控制台、引脚复用
echo "=== 1. /dev/ttyS* ==="
ls -l /dev/ttyS* 2>/dev/null || echo "(none)"

echo "=== 2. 串口别名（设备树 aliases） ==="
for a in /proc/device-tree/aliases/serial* /proc/device-tree/aliases/uart*; do
  [ -e "$a" ] || continue
  printf "%s -> " "$(basename "$a")"
  tr -d '\0' < "$a"
  echo
done

echo "=== 3. 设备树 serial 节点状态 ==="
for d in /proc/device-tree/serial@*; do
  [ -e "$d" ] || continue
  printf "%s status=" "$(basename "$d")"
  tr -d '\0' < "$d/status" 2>/dev/null
  echo
done

echo "=== 4. 各 ttyS 占用者 ==="
for t in /dev/ttyS*; do
  [ -e "$t" ] || continue
  printf "%-14s " "$t"
  if fuser "$t" 2>/dev/null | grep -q .; then fuser -v "$t" 2>&1 | tail -2; else echo "FREE"; fi
done

echo "=== 5. 内核串口识别 ==="
dmesg 2>/dev/null | grep -iE "ttyS[0-9]|8250|serial.*port" | tail -20 || echo "(dmesg 需要权限)"

echo "=== 6. 控制台占用 ==="
cat /proc/cmdline
ls /etc/systemd/system/getty.target.wants/ 2>/dev/null | grep -i serial || echo "(无 serial getty)"

echo "=== 7. 启动 overlays ==="
grep -vE "^#|^$" /boot/orangepiEnv.txt 2>/dev/null | head -30

echo "=== 8. 引脚复用里已启用的 uart ==="
ls /proc/device-tree/pinctrl/ 2>/dev/null | grep -i uart || echo "(no pinctrl uart nodes)"

echo "=== 9. 当前串口相关进程 ==="
ps -eo pid,etime,cmd 2>/dev/null | grep -iE "uart|serial|minicom|stm32" | grep -v grep || echo "(none)"

echo "=== 10. 波特率能力抽查 ==="
for t in /dev/ttyS2 /dev/ttyS3 /dev/ttyS4 /dev/ttyS7; do
  [ -e "$t" ] || continue
  before=$(stty -F "$t" 2>/dev/null | grep -o "speed [0-9]* baud")
  if stty -F "$t" 500000 2>/dev/null; then
    echo "$t: 500000 OK (原 $before)"
    stty -F "$t" 921600 2>/dev/null && echo "$t: 921600 OK"
  else
    echo "$t: 500000 FAILED (原 $before)"
  fi
done

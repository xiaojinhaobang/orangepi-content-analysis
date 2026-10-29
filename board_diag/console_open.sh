#!/usr/bin/env bash
# 桌面图标调用：确保控制台在跑，然后打开控制台页面
set -e
STATE="$HOME/.local/state/flight_console"
mkdir -p "$STATE"

if ! pgrep -f '[f]light_console.py' >/dev/null 2>&1; then
  setsid nohup "$HOME/diag/console_start.sh" >/dev/null 2>&1 &
  sleep 7
fi

xdg-open http://localhost:8090 >/dev/null 2>&1 || true

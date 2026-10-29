#!/bin/bash
# Keep the Orange Pi preferring the "Magic7" hotspot without disabling any other network.
# Runs from a systemd timer every 30s as root.

set -u

TARGET="Magic7"
TAG="prefer-magic7"

active=$(nmcli -t -f NAME connection show --active 2>/dev/null)

# Already connected to the target hotspot -> nothing to do.
if printf '%s\n' "$active" | grep -qx "$TARGET"; then
    exit 0
fi

# Is the target hotspot visible right now?
visible=$(nmcli -t -f SSID device wifi list 2>/dev/null | grep -x "$TARGET")
if [ -z "$visible" ]; then
    exit 0
fi

logger -t "$TAG" "target '$TARGET' visible, switching (was: $(printf '%s' "$active" | tr '\n' ',')"
nmcli connection up "$TARGET" >/dev/null 2>&1

if printf '%s\n' "$(nmcli -t -f NAME connection show --active 2>/dev/null)" | grep -qx "$TARGET"; then
    logger -t "$TAG" "now connected to $TARGET"
else
    logger -t "$TAG" "switch attempt finished, target not active yet"
fi

exit 0

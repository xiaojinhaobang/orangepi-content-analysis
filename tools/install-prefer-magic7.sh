#!/bin/bash
# Installs the Magic7 preference watchdog on the Orange Pi.
set -e

SRC_DIR=/tmp
BIN=/usr/local/bin/orangepi-prefer-magic7.sh
UNIT_DIR=/etc/systemd/system

install -m 0755 "$SRC_DIR/orangepi-prefer-magic7.sh" "$BIN"
install -m 0644 "$SRC_DIR/orangepi-prefer-magic7.service" "$UNIT_DIR/orangepi-prefer-magic7.service"
install -m 0644 "$SRC_DIR/orangepi-prefer-magic7.timer" "$UNIT_DIR/orangepi-prefer-magic7.timer"

systemctl daemon-reload
systemctl enable --now orangepi-prefer-magic7.timer

echo "--- installed ---"
systemctl is-enabled orangepi-prefer-magic7.timer
systemctl is-active orangepi-prefer-magic7.timer
systemctl list-timers orangepi-prefer-magic7.timer --no-pager

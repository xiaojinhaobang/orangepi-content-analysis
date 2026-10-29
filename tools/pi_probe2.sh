#!/bin/bash
echo "=== displays (DRM) ==="
ls /sys/class/drm 2>/dev/null
echo "=== xrandr (X session) ==="
DISPLAY=:0 xrandr 2>&1 | head -15 || echo "(xrandr unavailable)"
echo "=== input devices (touchscreen?) ==="
cat /proc/bus/input/devices 2>/dev/null | grep -E "^N: Name" | head -15
echo "=== USB devices ==="
lsusb 2>/dev/null | head -15
echo "=== usb serial ==="
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "(none)"
echo "=== usb_cam / v4l2 ros pkgs ==="
ls /opt/ros/humble/share 2>/dev/null | grep -iE "^usb_cam|v4l2|camera_" || echo "(none installed)"
echo "=== i2c / spi ==="
ls /dev/i2c-* /dev/spidev* 2>/dev/null || echo "(none)"
echo "=== power / thermal ==="
cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null
echo "=== desktop screen resolution ==="
DISPLAY=:0 xdpyinfo 2>/dev/null | grep -m2 -E "dimensions|screen #" || echo "(no X display info)"

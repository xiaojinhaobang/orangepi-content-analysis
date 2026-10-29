#!/bin/bash
echo "=== python ==="
python3 --version
echo "=== vision libs (pip) ==="
pip3 list 2>/dev/null | grep -iE "opencv|zbar|numpy|rknn|torch|ultralytics|pillow|pyzbar" || echo "(none)"
echo "=== vision libs (apt) ==="
dpkg -l 2>/dev/null | grep -iE "opencv|zbar" | awk '{print $2, $3}' || echo "(none)"
echo "=== import tests ==="
python3 -c "import cv2; print('cv2', cv2.__version__)" 2>&1 | tail -1
python3 -c "import pyzbar; print('pyzbar ok')" 2>&1 | tail -1
python3 -c "import numpy; print('numpy', numpy.__version__)" 2>&1 | tail -1
python3 -c "import rknnlite; print('rknnlite ok')" 2>&1 | tail -1
python3 -c "import gpiod; print('gpiod', gpiod.__version__)" 2>&1 | tail -1
echo "=== tools ==="
which zbarimg zbarcam v4l2-ctl rknn_server 2>/dev/null || echo "(some missing)"
echo "=== cameras ==="
ls /dev/video* 2>/dev/null || echo "(no video devices)"
ls /dev/v4l/by-id 2>/dev/null || echo "(no v4l by-id)"
echo "=== NPU ==="
ls /usr/lib/librknn* /usr/lib/aarch64-linux-gnu/librknn* 2>/dev/null || echo "(no rknn lib)"
ls /sys/kernel/debug/rknpu 2>/dev/null || echo "(no rknpu debugfs without root)"
echo "=== gpio ==="
ls /dev/gpiochip* 2>/dev/null || echo "(no gpiochip)"
echo "=== ros2 vision-related packages ==="
ls /opt/ros/humble/share 2>/dev/null | grep -iE "camera|image|vision|usb|v4l|compressed" | head -20
echo "=== ros2 workspace packages (ros2_ws) ==="
ls /home/orangepi/ros2_ws/install 2>/dev/null
echo "=== openmv_bridge ==="
find /home/orangepi/ros2_ws/src/openmv_bridge -type f | head -12
echo "=== serial devices ==="
ls /dev/serial/by-id 2>/dev/null || echo "(no serial by-id)"
ls /dev/ttyS6 /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
echo "=== CPU / NPU info ==="
cat /proc/cpuinfo | grep -m1 "model name" 2>/dev/null
cat /sys/kernel/debug/rknpu/version 2>/dev/null || echo "(rknpu version needs root)"
echo "=== RAM ==="
free -m | head -2

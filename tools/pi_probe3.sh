#!/bin/bash
echo "=== UART devices ==="
ls /dev/ttyS* 2>/dev/null
echo "=== boot env / overlays ==="
cat /boot/orangepiEnv.txt 2>/dev/null || echo "(no orangepiEnv.txt)"
grep -o "overlays=[^ ]*" /boot/*.txt /boot/*.env 2>/dev/null | head -5
echo "=== PCL / Ceres / Eigen / GTSAM headers ==="
ls -d /usr/include/pcl* /usr/include/eigen3 /usr/include/ceres /usr/include/gtsam 2>/dev/null || echo "(some missing)"
echo "=== apt: key libs ==="
dpkg -l 2>/dev/null | grep -iE "libpcl|libceres|libeigen|libgtsam|libboost|libsuitesparse|liblapack" | awk '{print $2, $3}' | head -20
echo "=== ros2 relevant pkgs ==="
ls /opt/ros/humble/share 2>/dev/null | grep -iE "pcl|point_cloud|laser|slam|nav2|cartographer|tf2|robot_localization|nav_msgs" | head -20
echo "=== bluesea2 driver params ==="
find /home/orangepi -maxdepth 5 -path "*bluesea*" -name "*.yaml" 2>/dev/null | head -5
echo "=== rknn runtime ==="
ls -la /usr/lib/librknnrt.so /usr/lib/librknn_api.so 2>/dev/null
ls /opt 2>/dev/null
echo "=== CPU governor / clocks ==="
cat /sys/devices/system/cpu/cpufreq/policy0/scaling_governor 2>/dev/null
cat /sys/devices/system/cpu/cpufreq/policy4/scaling_governor 2>/dev/null
cat /sys/devices/system/cpu/cpufreq/policy0/scaling_available_frequencies 2>/dev/null
echo "=== board identity ==="
cat /proc/device-tree/model 2>/dev/null
echo "=== stty 500000 test on /dev/ttyS6 (was 921600) ==="
stty -F /dev/ttyS6 500000 && echo "set 500000: OK" || echo "set 500000: FAILED"
stty -F /dev/ttyS6 | grep -E "speed"
stty -F /dev/ttyS6 921600 && echo "restore 921600: OK" || echo "restore 921600: FAILED"
stty -F /dev/ttyS6 | grep -E "speed"
echo "=== python libs for scan matching ==="
python3 -c "import open3d; print('open3d', open3d.__version__)" 2>&1 | tail -1
python3 -c "import scipy; print('scipy', scipy.__version__)" 2>&1 | tail -1
python3 -c "import sklearn; print('sklearn', sklearn.__version__)" 2>&1 | tail -1
python3 -c "import serial; print('pyserial', serial.__version__)" 2>&1 | tail -1

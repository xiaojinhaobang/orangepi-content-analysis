#!/usr/bin/env bash
# 探板：cv2 后端 / zbar / 相机档位 / USB 带宽
set +e

echo "=== python/cv2 ==="
python3 - <<'PY'
import cv2, sys
print("python", sys.version.split()[0])
print("cv2", cv2.__version__)
for name in ("QRCodeDetector", "wechat_qrcode", "barcode"):
    print(f"attr {name}:", hasattr(cv2, name))
try:
    wq = cv2.wechat_qrcode.WeChatQRCode()
    print("wechat_qrcode: OK (default model)")
except Exception as e:
    print("wechat_qrcode: FAIL", type(e).__name__, e)
try:
    import pyzbar.pyzbar as pz
    print("pyzbar: OK", pz.__file__)
except Exception as e:
    print("pyzbar: FAIL", type(e).__name__, e)
try:
    import numpy as np
    print("numpy", np.__version__)
except Exception as e:
    print("numpy FAIL", e)
print("build info:", [l for l in cv2.getBuildInformation().splitlines() if "Media I/O" in l or "Video I/O" in l])
PY

echo "=== zbar cli ==="
which zbarimg || echo "no zbarimg"

echo "=== camera devices ==="
ls -l /dev/video* 2>/dev/null
v4l2-ctl --list-devices 2>/dev/null

echo "=== formats /dev/video0 ==="
v4l2-ctl -d /dev/video0 --list-formats-ext 2>/dev/null

echo "=== usb tree ==="
lsusb -t 2>/dev/null
lsusb 2>/dev/null | grep -i -E "cam|32e6"

echo "=== cpu ==="
nproc
grep -m1 "model name" /proc/cpuinfo
cat /sys/devices/system/cpu/cpufreq/policy0/scaling_governor 2>/dev/null

echo "=== qr_vision test dir ==="
ls -l ~/qr_vision/test 2>/dev/null | head -20

#!/bin/bash
# 香橙派视觉能力实测：库 / 相机 / 二维码 / NPU / 现有程序
echo "=== 1. Python 视觉库 ==="
python3 - <<'PY'
import importlib
mods = ["cv2","numpy","torch","torchvision","onnxruntime","pyzbar","zbar","PIL","ultralytics","rknnlite","scipy"]
for m in mods:
    try:
        mod = importlib.import_module(m)
        print("OK  %-12s %s" % (m, getattr(mod, "__version__", "")))
    except Exception as e:
        print("NO  %-12s (%s)" % (m, type(e).__name__))
PY

echo "=== 2. OpenCV 功能开关 ==="
python3 - <<'PY'
import cv2
print("cv2", cv2.__version__)
for name in ["QRCodeDetector","aruco","dnn","VideoCapture","barcode"]:
    print("  %-16s %s" % (name, hasattr(cv2, name)))
print("  cuda modules:", cv2.cuda.getCudaEnabledDeviceCount() if hasattr(cv2, "cuda") else "n/a")
PY

echo "=== 3. 相机设备 ==="
ls -l /dev/video* 2>/dev/null || echo "NO /dev/video*"
ls /dev/v4l/by-id /dev/v4l/by-path 2>/dev/null || echo "(no v4l by-id/by-path)"
which v4l2-ctl >/dev/null 2>&1 && v4l2-ctl --list-devices 2>&1 | head -40 || echo "(v4l2-ctl not installed)"
echo "--- USB 设备 ---"
lsusb 2>/dev/null || echo "(no lsusb)"

echo "=== 4. 二维码 CLI 工具 ==="
which zbarimg zbarcam 2>/dev/null || echo "(no zbar CLI)"

echo "=== 5. ROS2 图像相关包 ==="
ls /opt/ros/humble/share 2>/dev/null | grep -iE "camera|image|vision|v4l|compressed|aruco|qr" || echo "(none)"

echo "=== 6. ros2_ws 的包清单 ==="
ls /home/orangepi/ros2_ws/src 2>/dev/null || echo "(no ros2_ws/src)"

echo "=== 7. 板上视觉相关代码搜索 ==="
find /home/orangepi -maxdepth 5 \( -iname "*openmv*" -o -iname "*camera*" -o -iname "*qr*" -o -iname "*aruco*" -o -iname "*yolo*" -o -iname "*detect*" -o -iname "*opencv*" \) -not -path "*/build/*" -not -path "*/install/*" -not -path "*/.cache/*" 2>/dev/null | head -40

echo "=== 8. 正在运行的视觉/相机进程 ==="
ps aux 2>/dev/null | grep -iE "yolo|camera|qr|detect|v4l2|opencv" | grep -v grep || echo "(none)"

echo "=== 9. NPU 运行时 ==="
ls -la /usr/lib/librknn* /usr/lib/aarch64-linux-gnu/librknn* 2>/dev/null || echo "(no librknn runtime)"
/usr/bin/rknn_server --version 2>/dev/null || echo "(rknn_server not found)"

echo "=== 10. My_yolo_ws 实际内容 ==="
find /home/orangepi/My_yolo_ws -maxdepth 2 2>/dev/null | head -20

echo "=== 11. 相机相关内核模块 / 节点 ==="
lsmod 2>/dev/null | grep -iE "uvc|v4l|video" || echo "(no uvc/v4l module)"
ls /dev/media* 2>/dev/null || echo "(no media device)"

echo "=== 12. pip 完整清单（视觉相关） ==="
pip3 list 2>/dev/null | grep -iE "opencv|zbar|torch|vision|yolo|onnx|pillow|numpy|scipy|rknn" || echo "(none)"

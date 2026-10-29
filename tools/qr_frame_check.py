"""对已保存的单帧做三段检查：检测器能否发现码 / 完整解码 / 快速解码。"""
import sys

import cv2
import numpy as np

sys.path.insert(0, "/home/orangepi/ros2_ws/src/qr_vision/scripts")
from qr_decode import decode_qr  # noqa: E402

path = sys.argv[1] if len(sys.argv) > 1 else "/home/orangepi/qr_vision/diag_1280.png"
img = cv2.imread(path)
if img is None:
    raise SystemExit(f"cannot read {path}")
h, w = img.shape[:2]
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
print(f"image {w}x{h} mean={gray.mean():.1f} std={gray.std():.1f} "
      f"sharp={cv2.Laplacian(gray, cv2.CV_64F).var():.0f}")

print("--- 1. QRCodeDetector.detect() 只检测不解码 ---")
detector = cv2.QRCodeDetector()
try:
    ok, points = detector.detect(img)
    print(f"detect -> {ok}  points={None if points is None else np.round(points, 1).tolist()}")
    if ok and points is not None:
        p = np.asarray(points, dtype=np.float32).reshape(-1, 2)
        print(f"  bbox {p[:,0].min():.0f},{p[:,1].min():.0f} -> "
              f"{p[:,0].max():.0f},{p[:,1].max():.0f}  "
              f"size {p[:,0].max()-p[:,0].min():.0f}x{p[:,1].max()-p[:,1].min():.0f}px")
except Exception as exc:  # noqa: BLE001
    print(f"detect raised {exc}")

print("--- 2. decode_qr(fast=False) 完整多预处理 ---")
try:
    print("full decode ->", decode_qr(img, fast=False))
except Exception as exc:  # noqa: BLE001
    print(f"full decode raised {exc}")

print("--- 3. decode_qr(fast=True) ---")
try:
    print("fast decode ->", decode_qr(img, fast=True))
except Exception as exc:  # noqa: BLE001
    print(f"fast decode raised {exc}")

print("--- 4. 图像中心 1/2 区域放大 2 倍后再解码 ---")
crop = img[h // 4:h * 3 // 4, w // 4:w * 3 // 4]
big = cv2.resize(crop, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
print("center zoom decode ->", decode_qr(big, fast=False))

"""相机诊断：画面是否在更新 + 各分辨率帧率 + 保存样帧。"""
import time

import cv2
import numpy as np


def open_cam(w, h):
    cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


print("=== 1. 画面是否在更新（同一分辨率连读 8 帧）===")
cap = open_cam(1280, 720)
prev = None
for i in range(8):
    ok, f = cap.read()
    if not ok:
        print(f"frame {i} read failed")
        continue
    gray = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
    g = gray.astype(np.float32)
    d = 0.0 if prev is None else float(np.abs(g - prev).mean())
    print(f"frame {i} mean={g.mean():6.1f} std={g.std():5.1f} "
          f"sharp={cv2.Laplacian(gray, cv2.CV_64F).var():7.0f} diff_prev={d:6.2f}")
    prev = g
    if i == 7:
        cv2.imwrite("/home/orangepi/qr_vision/diag_1280.png", f)
cap.release()

print("=== 2. 各分辨率实际帧率 ===")
for w, h in ((640, 480), (800, 600), (1280, 720)):
    cap = open_cam(w, h)
    t0 = time.perf_counter()
    n = 0
    while time.perf_counter() - t0 < 2.0:
        if cap.read()[0]:
            n += 1
    real_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    real_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"请求 {w}x{h} -> 实际 {real_w}x{real_h}  read_fps={n / (time.perf_counter() - t0):.1f}")
    cap.release()

"""合焦距离扫描：每 0.5 秒打印中央区域清晰度 + 二维码解码结果。

用法：把二维码居中拿在镜头前，按屏幕提示停留在 20 / 50 / 100 / 150 cm，
各停 5 秒。清晰度曲线的峰值处 = 镜头的合焦距离。
"""
import sys
import time

import cv2

sys.path.insert(0, "/home/orangepi/ros2_ws/src/qr_vision/scripts")
from qr_decode import decode_qr  # noqa: E402

DEV = "/dev/video0"
SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 24.0

cap = cv2.VideoCapture(DEV, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 3)          # 不要用 1，会掉一半帧率
for _ in range(5):
    cap.read()

t0 = time.perf_counter()
last_print = -1.0
n = 0
print("  t(s)  中央Laplacian  边缘锐度p99  解码")
while time.perf_counter() - t0 < SECONDS:
    ok, frame = cap.read()
    if not ok:
        continue
    n += 1
    t = time.perf_counter() - t0
    if t - last_print < 0.5:
        continue
    last_print = t
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape
    center = gray[h // 3:2 * h // 3, w // 3:2 * w // 3]
    c_sharp = cv2.Laplacian(center, cv2.CV_64F).var()
    # 梯度 p99.5 = 边缘锐度，受目标大小影响小，适合判断合焦
    gx = cv2.Sobel(center, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(center, cv2.CV_32F, 0, 1, ksize=3)
    edge = float(np.percentile(np.hypot(gx, gy), 99.5))
    hit = ""
    if n % 3 == 0:
        results = decode_qr(frame, fast=True)
        if results:
            hit = "HIT %s" % results[0]["text"]
    print(f"{t:6.1f} {c_sharp:13.0f} {edge:12.1f}  {hit}")

cap.release()
print(f"共 {n} 帧，平均帧率 {n / (time.perf_counter() - t0):.1f} fps")

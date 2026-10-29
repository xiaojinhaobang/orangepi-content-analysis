"""拆解帧率瓶颈：读原始 MJPG（不解码）vs 解码成 BGR，以及单帧解码耗时。"""
import time

import cv2
import numpy as np

DEV = "/dev/video0"


def open_cam(convert_rgb):
    cap = cv2.VideoCapture(DEV, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 1 if convert_rgb else 0)
    return cap


def read_fps(convert_rgb, seconds=2.5):
    cap = open_cam(convert_rgb)
    for _ in range(5):
        cap.read()
    t0 = time.perf_counter()
    n, last = 0, None
    while time.perf_counter() - t0 < seconds:
        ok, f = cap.read()
        if ok:
            n += 1
            last = f
    fps = n / (time.perf_counter() - t0)
    cap.release()
    return fps, last


fps_bgr, frame = read_fps(True)
print(f"解码成 BGR        : {fps_bgr:5.1f} fps   frame shape={None if frame is None else frame.shape}")

fps_raw, raw = read_fps(False)
size = None if raw is None else int(raw.size)
print(f"原始 MJPG 字节    : {fps_raw:5.1f} fps   jpeg bytes={size}")

if raw is not None:
    buf = raw.reshape(-1).astype(np.uint8)
    t0 = time.perf_counter()
    for _ in range(10):
        img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    ms = (time.perf_counter() - t0) * 10
    print(f"单独 JPEG 解码    : {ms:.1f} ms/帧  -> 理论上限 {1000/ms:.1f} fps  "
          f"解码结果 {None if img is None else img.shape}")

"""对比 OpenCV 各取帧路径的帧率：grab-only / read / 不同缓冲区 / 不同分辨率。"""
import time

import cv2

DEV = "/dev/video0"


def open_cam(w, h, buffers=None):
    cap = cv2.VideoCapture(DEV, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    if buffers is not None:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, buffers)
    for _ in range(5):
        cap.grab()
    return cap


def bench(label, w, h, buffers, mode, seconds=2.5):
    cap = open_cam(w, h, buffers)
    t0 = time.perf_counter()
    n = 0
    frames = []
    while time.perf_counter() - t0 < seconds:
        if mode == "grab":
            ok = cap.grab()
        else:
            ok, f = cap.read()
            if ok:
                frames.append(f)
        n += int(bool(ok))
    fps = n / (time.perf_counter() - t0)
    cap.release()
    print(f"{label:38s} {fps:5.1f} fps")
    return fps


print("--- 1280x720 ---")
bench("default buffers, read", 1280, 720, None, "read")
bench("default buffers, grab only", 1280, 720, None, "grab")
bench("buffers=1, read", 1280, 720, 1, "read")
bench("buffers=3, read", 1280, 720, 3, "read")
bench("buffers=6, read", 1280, 720, 6, "read")
bench("buffers=6, grab only", 1280, 720, 6, "grab")

print("--- 640x480 ---")
bench("buffers=6, read", 640, 480, 6, "read")
bench("buffers=6, grab only", 640, 480, 6, "grab")

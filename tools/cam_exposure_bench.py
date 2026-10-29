"""曝光/增益组合台架：测实际帧率与画面亮度，找帧率与亮度的平衡点。"""
import subprocess
import time

import cv2
import numpy as np

DEV = "/dev/video0"


def ctl(*args):
    subprocess.run(["v4l2-ctl", "-d", DEV, *args], capture_output=True)


def measure(label, seconds=2.0):
    cap = cv2.VideoCapture(DEV, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
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
    if last is None:
        print(f"{label:34s} read failed")
        return
    g = cv2.cvtColor(last, cv2.COLOR_BGR2GRAY)
    print(f"{label:34s} fps={fps:5.1f}  mean={g.mean():6.1f}  "
          f"std={g.std():5.1f}  sharp={cv2.Laplacian(g, cv2.CV_64F).var():6.0f}")


print("组合测试（exposure_time_absolute 单位 100us，gain 0~63）")
ctl("--set-ctrl=auto_exposure=3")
measure("auto(基线)")

for expo, gain in ((313, 0), (200, 0), (150, 0), (100, 0), (100, 40), (70, 63), (50, 63)):
    ctl("--set-ctrl=auto_exposure=1",
        f"--set-ctrl=exposure_time_absolute={expo}",
        f"--set-ctrl=gain={gain}")
    measure(f"manual expo={expo}({expo/10:.1f}ms) gain={gain}")

ctl("--set-ctrl=auto_exposure=3")
print("已恢复自动曝光")

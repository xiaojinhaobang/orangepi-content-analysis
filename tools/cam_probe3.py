"""Third-stage probe: real driver fps, exposure controls, frame freshness."""

import hashlib
import subprocess
import time

import cv2
import numpy as np


def run(cmd, timeout=30):
    """Run a command and print its trimmed output."""
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except Exception as error:  # noqa: BLE001 - report and continue
        print(f"$ {' '.join(cmd)}\n  FAILED: {error}")
        return
    print(f"$ {' '.join(cmd)}")
    for stream in (proc.stdout, proc.stderr):
        text = (stream or "").strip()
        if text:
            print("  " + text.replace("\n", "\n  "))


def driver_fps(device, width, height, fourcc, count):
    """Time a fixed-count v4l2-ctl stream to get the true driver frame rate."""
    cmd = [
        "v4l2-ctl", "-d", device,
        f"--set-fmt-video=width={width},height={height},pixelformat={fourcc}",
        "--stream-mmap", f"--stream-count={count}", "--stream-to=/dev/null",
    ]
    start = time.perf_counter()
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
    elapsed = time.perf_counter() - start
    print(f"[driver] {width}x{height} {fourcc}: {count} frames in {elapsed:.2f}s "
          f"= {count / elapsed:.1f} fps")
    tail = (proc.stdout or "").strip().splitlines()[-2:]
    for line in tail:
        print("   " + line)


def frame_freshness(device, width, height, frames):
    """Check whether consecutive raw MJPG buffers actually differ."""
    capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    capture.set(cv2.CAP_PROP_FPS, 30)
    capture.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    digests = []
    images = []
    start = time.perf_counter()
    for _ in range(frames):
        ok, buffer = capture.read()
        if not ok:
            continue
        digests.append(hashlib.md5(buffer.tobytes()).hexdigest()[:10])
        images.append(buffer)
    elapsed = time.perf_counter() - start
    capture.release()

    unique = len(set(digests))
    print(f"[fresh] {len(digests)} raw frames in {elapsed:.2f}s = {len(digests) / elapsed:.1f} fps, "
          f"unique={unique}")
    print(f"[fresh] digests: {' '.join(digests[:12])}")

    if len(images) >= 2:
        first = cv2.imdecode(images[0], cv2.IMREAD_GRAYSCALE)
        last = cv2.imdecode(images[-1], cv2.IMREAD_GRAYSCALE)
        if first is not None and last is not None and first.shape == last.shape:
            diff = cv2.absdiff(first, last)
            print(f"[fresh] first-vs-last: mean|diff|={diff.mean():.2f} max={diff.max()} "
                  f"changed_px={(diff > 8).mean() * 100:.1f}%")
            print(f"[fresh] mean brightness={first.mean():.1f}")


def main():
    device = "/dev/video0"
    run(["v4l2-ctl", "-d", device, "-l"])
    run(["v4l2-ctl", "-d", device, "--list-ctrls-menus"])
    driver_fps(device, 1280, 720, "MJPG", 60)
    driver_fps(device, 640, 480, "MJPG", 60)
    driver_fps(device, 640, 480, "YUYV", 60)
    frame_freshness(device, 1280, 720, 15)


if __name__ == "__main__":
    main()

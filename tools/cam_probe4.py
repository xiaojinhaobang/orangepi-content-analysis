"""Fourth-stage probe: locate the host-side frame rate bottleneck (driver 26.7 fps vs cv2 12.8 fps)."""

import os
import subprocess
import time

import cv2
import numpy as np

DEVICE = "/dev/video0"


def system_load() -> None:
    """Print load average and the busiest processes."""
    print(f"[cpu] loadavg={os.getloadavg()} cpus={os.cpu_count()}")
    out = subprocess.run(
        ["ps", "-eo", "pcpu,pmem,comm", "--sort=-pcpu"],
        capture_output=True, text=True, timeout=15,
    ).stdout
    for line in out.splitlines()[:8]:
        print("   " + line.rstrip())


def driver_fps(width: int, height: int, fourcc: str, count: int = 60) -> None:
    """Time v4l2-ctl streaming as the reference frame rate."""
    cmd = [
        "v4l2-ctl", "-d", DEVICE,
        f"--set-fmt-video=width={width},height={height},pixelformat={fourcc}",
        "--stream-mmap", f"--stream-count={count}", "--stream-to=/dev/null",
    ]
    start = time.perf_counter()
    subprocess.run(cmd, capture_output=True, text=True, timeout=60)
    elapsed = time.perf_counter() - start
    print(f"[driver] {width}x{height} {fourcc}: {count / elapsed:.1f} fps")


def open_capture(width: int, height: int, fourcc: str, convert_rgb: int, fps: int = 30):
    """Open the camera with an explicit pixel format and conversion mode."""
    capture = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    capture.set(cv2.CAP_PROP_FPS, fps)
    capture.set(cv2.CAP_PROP_CONVERT_RGB, convert_rgb)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return capture


def read_fps(width: int, height: int, fourcc: str, convert_rgb: int,
             frames: int = 40, fps: int = 30, save: str = "") -> None:
    """Measure per-read latency so blocking behaviour shows up in the histogram."""
    capture = open_capture(width, height, fourcc, convert_rgb, fps)
    if not capture.isOpened():
        print(f"[cv2] {width}x{height} {fourcc} convert={convert_rgb}: OPEN FAILED")
        return
    for _ in range(5):  # let auto exposure settle
        capture.read()

    latencies = []
    shape = None
    start = time.perf_counter()
    for _ in range(frames):
        mark = time.perf_counter()
        ok, frame = capture.read()
        latencies.append((time.perf_counter() - mark) * 1000.0)
        if ok and hasattr(frame, "shape"):
            shape = frame.shape
    elapsed = time.perf_counter() - start
    if save and ok:
        cv2.imwrite(save, frame)
    capture.release()

    stats = np.asarray(latencies)
    print(f"[cv2] {width}x{height} {fourcc} convert={convert_rgb} capfps={fps}: "
          f"{frames / elapsed:.1f} fps  lat mean={stats.mean():.1f} "
          f"p10={np.percentile(stats, 10):.1f} p90={np.percentile(stats, 90):.1f} "
          f"ms  shape={shape}")


def decode_cost(width: int, height: int, fourcc: str, frames: int = 15) -> None:
    """Separate capture from decode: time imdecode on raw buffers."""
    capture = open_capture(width, height, fourcc, convert_rgb=0)
    buffers = []
    for _ in range(frames):
        ok, buffer = capture.read()
        if ok:
            buffers.append(buffer)
    capture.release()
    if not buffers:
        print("[decode] no raw buffers captured")
        return

    mark = time.perf_counter()
    for buffer in buffers:
        cv2.imdecode(buffer, cv2.IMREAD_GRAYSCALE)
    gray_ms = (time.perf_counter() - mark) * 1000.0 / len(buffers)

    mark = time.perf_counter()
    for buffer in buffers:
        cv2.imdecode(buffer, cv2.IMREAD_COLOR)
    color_ms = (time.perf_counter() - mark) * 1000.0 / len(buffers)
    print(f"[decode] {width}x{height} {fourcc} over {len(buffers)} buffers: "
          f"gray={gray_ms:.1f} ms color={color_ms:.1f} ms")


def main() -> None:
    """Run the full comparison."""
    system_load()
    read_fps(1280, 720, "MJPG", 1, save="/tmp/now_1280.jpg")
    read_fps(1280, 720, "MJPG", 0)
    read_fps(640, 480, "MJPG", 0)
    read_fps(640, 480, "YUYV", 1)
    read_fps(800, 600, "MJPG", 0)
    decode_cost(1280, 720, "MJPG")
    decode_cost(640, 480, "MJPG")
    decode_cost(800, 600, "MJPG")
    driver_fps(1280, 720, "MJPG")


if __name__ == "__main__":
    main()

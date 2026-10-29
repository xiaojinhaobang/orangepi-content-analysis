"""Second-stage camera probe: true fps ceiling and cheap decode paths."""

import argparse
import subprocess
import time

import cv2
import numpy as np


def v4l2ctl_fps(device, width, height, fourcc, seconds=4):
    """Stream with v4l2-ctl (bypasses OpenCV) to find the driver-level ceiling."""
    cmd = [
        "v4l2-ctl", "-d", device,
        f"--set-fmt-video=width={width},height={height},pixelformat={fourcc}",
        "--stream-mmap", "--stream-to=/dev/null",
    ]
    start = time.perf_counter()
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=seconds + 8)
    elapsed = time.perf_counter() - start
    print(f"[v4l2-ctl] {width}x{height} {fourcc} -> {elapsed:.2f}s for the run")
    print("   stdout:", (proc.stdout or "").strip().replace("\n", " | ")[-300:])
    if proc.stderr.strip():
        print("   stderr:", proc.stderr.strip().replace("\n", " | ")[-300:])


def raw_mjpg_capture(device, width, height, seconds):
    """Grab raw MJPG buffers (no decode) and report the achievable rate."""
    capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    capture.set(cv2.CAP_PROP_FPS, 30)
    capture.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    ok, buffer = capture.read()
    if not ok or buffer is None:
        print("[raw] CONVERT_RGB=0 unsupported")
        capture.release()
        return None

    size = buffer.size
    print(f"[raw] shape={buffer.shape} dtype={buffer.dtype} bytes={size} "
          f"(~{size * 15 / 1e6:.1f} MB/s at 15 fps)")

    for _ in range(3):
        capture.read()
    start = time.perf_counter()
    count = 0
    last = buffer
    while time.perf_counter() - start < seconds:
        ok, last = capture.read()
        count += int(ok)
    elapsed = time.perf_counter() - start
    capture.release()
    print(f"[raw] grab={count / elapsed:.1f} fps (no decode at all)")
    return last


def bench_imdecode(buffer, repeats):
    """Compare JPEG decode cost at full/2/4/8 reduced scale."""
    flags = (
        ("full-gray", cv2.IMREAD_GRAYSCALE),
        ("gray-1/2", cv2.IMREAD_REDUCED_GRAYSCALE_2),
        ("gray-1/4", cv2.IMREAD_REDUCED_GRAYSCALE_4),
        ("gray-1/8", cv2.IMREAD_REDUCED_GRAYSCALE_8),
    )
    for name, flag in flags:
        image = cv2.imdecode(buffer, flag)
        if image is None:
            print(f"[imdecode] {name}: unsupported")
            continue
        for _ in range(2):
            cv2.imdecode(buffer, flag)
        start = time.perf_counter()
        for _ in range(repeats):
            image = cv2.imdecode(buffer, flag)
        cost = (time.perf_counter() - start) / repeats * 1000
        print(f"[imdecode] {name:10s} {image.shape[1]}x{image.shape[0]} {cost:6.2f} ms")


def bench_detect_scale(buffer, code_text, repeats):
    """Cost of the QR search stage on progressively smaller frames."""
    full = cv2.imdecode(buffer, cv2.IMREAD_GRAYSCALE)
    detector = cv2.QRCodeDetector()
    print(f"[detect] source frame {full.shape[1]}x{full.shape[0]}")
    for divisor in (1, 2, 4):
        image = cv2.resize(full, None, fx=1 / divisor, fy=1 / divisor, interpolation=cv2.INTER_AREA)
        for _ in range(2):
            detector.detectAndDecodeMulti(image)
        start = time.perf_counter()
        for _ in range(repeats):
            out = detector.detectAndDecodeMulti(image)
        cost = (time.perf_counter() - start) / repeats * 1000
        texts = [t for t in (out[1] or ()) if t]
        print(f"[detect] 1/{divisor} ({image.shape[1]}x{image.shape[0]}) {cost:6.2f} ms  texts={texts}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--seconds", type=float, default=4.0)
    parser.add_argument("--repeats", type=int, default=10)
    parser.add_argument("--skip-v4l2ctl", action="store_true")
    args = parser.parse_args()

    if not args.skip_v4l2ctl:
        for width, height, fourcc in ((1280, 720, "MJPG"), (640, 480, "YUYV")):
            try:
                v4l2ctl_fps(args.device, width, height, fourcc)
            except Exception as error:  # noqa: BLE001 - report and continue
                print(f"[v4l2-ctl] {width}x{height} {fourcc} failed: {error}")

    buffer = raw_mjpg_capture(args.device, 1280, 720, args.seconds)
    if buffer is not None:
        bench_imdecode(buffer, args.repeats)
        bench_detect_scale(buffer, "7", args.repeats)
        cv2.imwrite("/home/orangepi/qr_vision/raw_probe.png",
                    cv2.imdecode(buffer, cv2.IMREAD_COLOR))


if __name__ == "__main__":
    main()

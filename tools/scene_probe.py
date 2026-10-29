"""Inspect what the camera sees: QR detection-only test plus an ASCII luminance map."""

import argparse
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, "/home/orangepi/ros2_ws/src/qr_vision/scripts")
from qr_decode import decode_qr  # noqa: E402

RAMP = " .:-=+*#%@"


def ascii_map(gray: np.ndarray, cols: int = 72, rows: int = 30) -> str:
    """Downsample to a coarse ASCII luminance map."""
    small = cv2.resize(gray, (cols, rows), interpolation=cv2.INTER_AREA)
    lines = []
    for row in small:
        lines.append("".join(RAMP[min(len(RAMP) - 1, int(value) * len(RAMP) // 256)] for value in row))
    return "\n".join(lines)


def main() -> None:
    """Probe the live camera."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--frames", type=int, default=8)
    args = parser.parse_args()

    capture = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not capture.isOpened():
        raise SystemExit(f"cannot open {args.device}")
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    best = None
    for index in range(args.frames):
        ok, frame = capture.read()
        if not ok:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 60, 180)
        # decode_qr() wraps detectAndDecodeMulti(), whose return arity is stable
        # across OpenCV builds (detectAndDecode() is not).
        results = decode_qr(frame, fast=False)
        print(f"frame {index}: decoded={[item['text'] for item in results]} "
              f"edge_pct={100.0 * float((edges > 0).mean()):.2f}")
        for item in results:
            pts = np.asarray(item["points"], dtype=np.float32).reshape(-1, 2)
            print(f"          '{item['text']}' center={item['center']} "
                  f"x=[{pts[:, 0].min():.0f}..{pts[:, 0].max():.0f}] "
                  f"y=[{pts[:, 1].min():.0f}..{pts[:, 1].max():.0f}] "
                  f"width={pts[:, 0].max() - pts[:, 0].min():.0f}px")
            best = frame

    ok, frame = capture.read()
    capture.release()
    if ok:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        print(f"--- scene (mean={gray.mean():.0f} std={gray.std():.0f} "
              f"p5={np.percentile(gray, 5):.0f} p95={np.percentile(gray, 95):.0f}) ---")
        print(ascii_map(gray))
        cv2.imwrite("/home/orangepi/qr_vision/scene.png", best if best is not None else frame)


if __name__ == "__main__":
    main()

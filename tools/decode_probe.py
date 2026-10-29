"""Probe the live camera: decode QR codes from N frames and report timings."""

import argparse
import sys
import time

import cv2

sys.path.insert(0, "/home/orangepi/ros2_ws/src/qr_vision/scripts")
from qr_decode import code_to_position, decode_qr, parse_code  # noqa: E402


def main() -> None:
    """Capture frames and report QR detections."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--frames", type=int, default=20)
    parser.add_argument("--save", default="/home/orangepi/qr_vision/hit.png")
    args = parser.parse_args()

    capture = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not capture.isOpened():
        raise SystemExit(f"cannot open {args.device}")
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    hits = 0
    fast_total = 0.0
    for index in range(args.frames):
        ok, frame = capture.read()
        if not ok:
            print(f"frame {index}: read failed")
            continue

        start = time.perf_counter()
        results = decode_qr(frame, fast=True)
        fast_ms = (time.perf_counter() - start) * 1000.0
        fast_total += fast_ms

        mode = "fast"
        if not results:
            start = time.perf_counter()
            results = decode_qr(frame, fast=False)
            mode = f"variants {(time.perf_counter() - start) * 1000.0:.0f}ms"

        if results:
            hits += 1
            codes = [code_to_position(code) if (code := parse_code(item["text"])) else "?"
                     for item in results]
            print(f"frame {index:2d}: {mode:16s} texts={[item['text'] for item in results]} "
                  f"positions={codes} centers={[item['center'] for item in results]}")
            if hits == 1:
                cv2.imwrite(args.save, frame)
        else:
            print(f"frame {index:2d}: {mode:16s} no code  fast={fast_ms:.1f}ms "
                  f"mean={frame.mean():.0f}")

    capture.release()
    print(f"--- {hits}/{args.frames} frames decoded, mean fast decode "
          f"{fast_total / max(1, args.frames):.1f} ms ---")


if __name__ == "__main__":
    main()

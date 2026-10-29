"""Live QR probe: per-frame hit, code pixel size, sharpness and decode timing."""

import argparse
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, "/home/orangepi/ros2_ws/src/qr_vision/scripts")
from qr_decode import code_to_position, decode_qr, parse_code  # noqa: E402


def main() -> None:
    """Probe the live camera frame by frame."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument("--frames", type=int, default=25)
    parser.add_argument("--save", default="/home/orangepi/qr_vision/live_hit.png")
    args = parser.parse_args()

    capture = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not capture.isOpened():
        raise SystemExit(f"cannot open {args.device}")
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    # 注意：BUFFERSIZE=1 会让帧率从 30fps 掉到 15fps（实测），必须 >=3
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 3)

    start = time.perf_counter()
    read = 0
    while time.perf_counter() - start < 2.0:
        ok, _ = capture.read()
        read += int(ok)
    print(f"camera {int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
          f"{int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))} {args.fourcc} "
          f"read_fps={read / (time.perf_counter() - start):.1f}")

    hits = 0
    durations = []
    for index in range(args.frames):
        ok, frame = capture.read()
        if not ok:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        frame_start = time.perf_counter()
        results = decode_qr(frame, fast=True)
        fast_ms = (time.perf_counter() - frame_start) * 1000.0
        durations.append(fast_ms)

        if results:
            hits += 1
            for item in results:
                points = np.asarray(item["points"], dtype=np.float32).reshape(-1, 2)
                x0, x1 = points[:, 0].min(), points[:, 0].max()
                y0, y1 = points[:, 1].min(), points[:, 1].max()
                roi = gray[int(max(y0, 0)):int(y1), int(max(x0, 0)):int(x1)]
                sharpness = cv2.Laplacian(roi, cv2.CV_64F).var() if roi.size else 0.0
                code = parse_code(item["text"])
                print(f"frame {index:2d} HIT text={item['text']} "
                      f"pos={code_to_position(code) if code else '?'} "
                      f"center={item['center']} size={x1 - x0:.0f}x{y1 - y0:.0f}px "
                      f"sharp={sharpness:.0f} fast={fast_ms:.0f}ms")
                cv2.imwrite(args.save, frame)
                break
        else:
            print(f"frame {index:2d} no code fast={fast_ms:.0f}ms "
                  f"mean={gray.mean():.0f} sharp={cv2.Laplacian(gray, cv2.CV_64F).var():.0f}")

    capture.release()
    if durations:
        print(f"--- hits {hits}/{args.frames}  mean fast={np.mean(durations):.0f}ms  "
              f"p90={np.percentile(durations, 90):.0f}ms ---")


if __name__ == "__main__":
    main()

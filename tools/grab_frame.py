"""Grab one frame from a V4L2 camera and report basic image statistics."""

import argparse

import cv2
import numpy as np


def main() -> None:
    """Capture a single frame."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--out", default="/home/orangepi/qr_vision/preview.jpg")
    parser.add_argument("--warmup", type=int, default=10)
    args = parser.parse_args()

    capture = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not capture.isOpened():
        raise SystemExit(f"cannot open {args.device}")
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    frame = None
    for _ in range(args.warmup):
        ok, frame = capture.read()
        if not ok:
            raise SystemExit("frame read failed")
    capture.release()

    cv2.imwrite(args.out, frame)
    print(f"saved {args.out} {frame.shape[1]}x{frame.shape[0]} "
          f"mean={frame.mean():.1f} std={frame.std():.1f} "
          f"dark_pct={100.0 * float((frame.mean(axis=2) < 20).mean()):.1f}")

    small = cv2.resize(frame, (320, 180))
    print("thumb:", np.round(small[::60, ::80].reshape(-1, 3).mean(axis=0)).tolist())


if __name__ == "__main__":
    main()

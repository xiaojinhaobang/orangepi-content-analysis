"""Decode every image in a directory and report text, shelf position and timing."""

import argparse
import glob
import os
import sys
import time

import cv2

sys.path.insert(0, "/home/orangepi/ros2_ws/src/qr_vision/scripts")
from qr_decode import code_to_position, decode_qr, draw_results, parse_code  # noqa: E402


def main() -> None:
    """Run the decode sweep."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir", default="/home/orangepi/qr_vision/test")
    parser.add_argument("--save-dir", default="/home/orangepi/qr_vision/annot")
    args = parser.parse_args()

    files = sorted(glob.glob(os.path.join(args.dir, "*.png"))
                   + glob.glob(os.path.join(args.dir, "*.jpg")))
    if not files:
        raise SystemExit(f"no images in {args.dir}")
    os.makedirs(args.save_dir, exist_ok=True)

    hits = 0
    for path in files:
        image = cv2.imread(path)
        if image is None:
            continue

        start = time.perf_counter()
        results = decode_qr(image, fast=True)
        fast_ms = (time.perf_counter() - start) * 1000.0
        mode = "fast"
        if not results:
            start = time.perf_counter()
            results = decode_qr(image, fast=False)
            mode = f"variants {1000 * (time.perf_counter() - start):.0f}ms"

        texts = [item["text"] for item in results]
        positions = []
        for text in texts:
            code = parse_code(text)
            positions.append(code_to_position(code) if code else "?")

        print(f"{os.path.basename(path):12s} {image.shape[1]}x{image.shape[0]:<5d} {mode:14s} "
              f"texts={texts} positions={positions} fast={fast_ms:.0f}ms")

        if results:
            hits += 1
            cv2.imwrite(os.path.join(args.save_dir, os.path.basename(path)),
                        draw_results(image, results))

    print(f"--- {hits}/{len(files)} images decoded ---")


if __name__ == "__main__":
    main()

"""Benchmark decode_qr() on the Orange Pi: average milliseconds per frame."""

import sys
import time

import cv2

sys.path.insert(0, "/home/orangepi/qr_vision")
from opencv_qr import decode_qr  # noqa: E402


def bench(path: str, fast: bool, runs: int = 30) -> float:
    image = cv2.imread(path)
    if image is None:
        raise SystemExit(f"cannot read {path}")
    decode_qr(image, fast)  # warm up
    start = time.perf_counter()
    for _ in range(runs):
        results = decode_qr(image, fast)
    elapsed = (time.perf_counter() - start) / runs * 1000.0
    print(f"{path.split('/')[-1]:14s} fast={fast!s:5s} {elapsed:7.2f} ms/frame  hit={len(results)}")
    return elapsed


if __name__ == "__main__":
    for name, px in (("1_near.png", "640px"), ("1_mid.png", "320px"), ("1_far.png", "160px")):
        for fast in (True, False):
            bench(f"/home/orangepi/qr_vision/test/{name}", fast)

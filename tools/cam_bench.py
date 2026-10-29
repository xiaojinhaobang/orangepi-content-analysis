"""Camera FPS + QR decode distance benchmark for the Orange Pi 5 Max."""

import argparse
import time

import cv2
import numpy as np


def open_capture(device, width, height, fourcc, fps):
    """Open a V4L2 capture with an explicit format and frame rate."""
    capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not capture.isOpened():
        raise SystemExit(f"cannot open {device}")
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if fps:
        capture.set(cv2.CAP_PROP_FPS, fps)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return capture


def bench_capture(device, width, height, fourcc, fps, seconds):
    """Measure pure grab rate (no decode) for one capture configuration."""
    capture = open_capture(device, width, height, fourcc, fps)
    actual_w = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = capture.get(cv2.CAP_PROP_FPS)
    code = int(capture.get(cv2.CAP_PROP_FOURCC))
    got_code = "".join(chr((code >> (8 * i)) & 0xFF) for i in range(4))

    for _ in range(3):  # warm up / drop stale buffers
        capture.read()

    start = time.perf_counter()
    count = 0
    while time.perf_counter() - start < seconds:
        ok, _ = capture.read()
        count += int(ok)
    elapsed = time.perf_counter() - start
    capture.release()

    print(f"[capture] req={width}x{height}@{fps} {fourcc} -> "
          f"got={actual_w}x{actual_h} got_fourcc={got_code} got_fps={actual_fps:.0f} | "
          f"grab={count / elapsed:.1f} fps ({count} frames / {elapsed:.1f}s)")


def make_qr(text, width_px):
    """Return a BGR QR image whose code area is about width_px pixels wide."""
    encoder = cv2.QRCodeEncoder.create()
    qr = encoder.encode(text)
    if qr.ndim == 2:
        qr = cv2.cvtColor(qr, cv2.COLOR_GRAY2BGR)
    side = max(1, int(round(width_px)))
    return cv2.resize(qr, (side, side), interpolation=cv2.INTER_AREA)


def compose_scene(qr, frame_w, frame_h, blur, noise, seed=0):
    """Paste the QR onto a light background with blur and noise (fake distance)."""
    rng = np.random.default_rng(seed)
    scene = np.full((frame_h, frame_w, 3), 205, np.uint8)
    scene[:] = np.clip(scene.astype(np.int16) + rng.normal(0, 12, scene.shape), 0, 255).astype(np.uint8)

    qh, qw = qr.shape[:2]
    margin = 12
    pad = np.full((qh + 2 * margin, qw + 2 * margin, 3), 255, np.uint8)
    pad[margin:margin + qh, margin:margin + qw] = qr
    qh, qw = pad.shape[:2]

    x0 = (frame_w - qw) // 2
    y0 = (frame_h - qh) // 2
    scene[y0:y0 + qh, x0:x0 + qw] = pad

    if blur > 0:
        k = int(blur) * 2 + 1
        scene = cv2.GaussianBlur(scene, (k, k), blur)
    if noise > 0:
        scene = np.clip(scene.astype(np.int16) + rng.normal(0, noise, scene.shape), 0, 255).astype(np.uint8)
    return scene


def decode_with(detector, image):
    """Return decoded texts; tolerates old/new OpenCV return signatures."""
    out = detector.detectAndDecodeMulti(image)
    decoded = out[1] if len(out) > 1 else ()
    return [text for text in (decoded or ()) if text]


def bench_decode(frame_w, frame_h, blur, noise, sizes, repeats):
    """Sweep QR pixel widths and report decode success/latency per backend."""
    detectors = {"QRCodeDetector": cv2.QRCodeDetector()}
    if hasattr(cv2, "QRCodeDetectorAruco"):
        detectors["QRCodeDetectorAruco"] = cv2.QRCodeDetectorAruco()

    print(f"[decode] frame={frame_w}x{frame_h} blur={blur} noise={noise} "
          f"backends={list(detectors)}")

    # Empty-scene latency: this is the common case while flying.
    empty = compose_scene(np.full((10, 10, 3), 255, np.uint8), frame_w, frame_h, blur, noise)
    for name, detector in detectors.items():
        detector.detectAndDecodeMulti(empty)
        start = time.perf_counter()
        for _ in range(repeats):
            decode_with(detector, empty)
        print(f"  {name:24s} empty-frame {(time.perf_counter() - start) / repeats * 1000:6.1f} ms")

    for size in sizes:
        qr = make_qr("7", size)
        scene = compose_scene(qr, frame_w, frame_h, blur, noise)
        gray = cv2.cvtColor(scene, cv2.COLOR_BGR2GRAY)
        row = [f"  qr_width={size:4d}px"]
        for name, detector in detectors.items():
            hits = 0
            elapsed = 0.0
            for _ in range(repeats):
                start = time.perf_counter()
                texts = decode_with(detector, gray)
                elapsed += time.perf_counter() - start
                hits += int("7" in texts)
            row.append(f"{name}: hit {hits}/{repeats} {elapsed / repeats * 1000:5.1f}ms")
        print(" | ".join(row))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("fps", "decode", "all"), default="all")
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--seconds", type=float, default=3.0)
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--blur", type=float, default=1.2)
    parser.add_argument("--noise", type=float, default=6.0)
    args = parser.parse_args()

    if args.mode in ("fps", "all"):
        for width, height, fourcc, fps in (
            (1280, 720, "MJPG", 30),
            (1280, 720, "MJPG", 0),
            (800, 600, "MJPG", 30),
            (640, 480, "MJPG", 30),
            (640, 480, "YUYV", 30),
        ):
            try:
                bench_capture(args.device, width, height, fourcc, fps, args.seconds)
            except SystemExit as error:
                print(f"[capture] {error}")

    if args.mode in ("decode", "all"):
        sizes = (40, 60, 80, 100, 150, 200, 300, 400)
        for frame_w, frame_h in ((1280, 720), (800, 600)):
            bench_decode(frame_w, frame_h, args.blur, args.noise, sizes, args.repeats)


if __name__ == "__main__":
    main()

"""Generate test QR images (contents 1..24) mimicking the D题 shelf codes."""

from pathlib import Path

import cv2
import numpy as np
import os

OUT_DIR = Path(__file__).parent / "qr_test"


def make_qr(text: str, target_px: int) -> np.ndarray:
    """Return a white-background BGR image with a QR code of roughly target_px width."""
    encoder = cv2.QRCodeEncoder.create()
    qr = encoder.encode(text)
    if qr.ndim == 2:
        qr = cv2.cvtColor(qr, cv2.COLOR_GRAY2BGR)

    scale = max(1, round(target_px / max(qr.shape[:2])))
    qr = cv2.resize(qr, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)

    canvas = np.full((qr.shape[0] + 80, qr.shape[1] + 80, 3), 255, np.uint8)
    canvas[40:40 + qr.shape[0], 40:40 + qr.shape[1]] = qr
    return canvas


def main() -> None:
    print("cwd:", os.getcwd())
    print("out dir:", OUT_DIR.resolve())
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for text in ("1", "12", "24"):
        for tag, px in (("near", 640), ("mid", 320), ("far", 160)):
            image = make_qr(text, px)
            path = OUT_DIR / f"{text}_{tag}.png"
            # cv2.imwrite fails on non-ASCII Windows paths, so encode then write the bytes.
            success, buffer = cv2.imencode(".png", image)
            if success:
                path.write_bytes(buffer.tobytes())
            print(f"{path.name}: {image.shape[1]}x{image.shape[0]} written={success} exists={path.exists()}")


if __name__ == "__main__":
    main()

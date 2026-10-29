"""QR decoding core, ported from 学习资料/02_上位机与视觉/opencv_qr.py.

Kept dependency-free (OpenCV + numpy only) so it can also be imported by
standalone scripts or reused inside other nodes.
"""

import cv2
import numpy as np

DETECTOR = cv2.QRCodeDetector()
FACE_LETTERS = "ABCD"
SLOTS_PER_FACE = 6
CODES_PER_FACE = SLOTS_PER_FACE


def code_to_position(code: int) -> str:
    """Map a 1~24 goods code to its A1~D6 shelf position.

    Mapping: 1~6 -> A1~A6, 7~12 -> B1~B6, 13~18 -> C1~C6, 19~24 -> D1~D6.
    """
    if not 1 <= code <= 24:
        return ""
    face = (code - 1) // CODES_PER_FACE
    slot = (code - 1) % CODES_PER_FACE + 1
    return f"{FACE_LETTERS[face]}{slot}"


def _variants(image, fast):
    """Return (variant, scale) pairs; scale maps points back to the input image."""
    if fast:
        return [(image, 1.0)]

    variants = [(image, 1.0)]
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    variants.append((gray, 1.0))

    h, w = gray.shape[:2]
    if max(h, w) <= 1280:
        scale = min(6.0, 1280 / max(h, w))
        if scale > 1.0:
            resized = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
            variants.append((resized, scale))

    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    variants.append((binary, 1.0))
    variants.append((cv2.bitwise_not(binary), 1.0))
    return variants


def decode_qr(image, fast=False):
    """Decode every QR code in a BGR or grayscale image.

    Args:
        image: BGR or grayscale image.
        fast: decode the raw frame only instead of extra preprocessing variants.

    Returns:
        List of dicts: {"text", "points" (4x2 int), "center" (2 int)}.
    """
    results = []
    for variant, scale in _variants(image, fast):
        _, decoded, points, _ = DETECTOR.detectAndDecodeMulti(variant)
        if points is None:
            continue
        for text, corners in zip(decoded, points):
            if not text:
                continue
            corners = np.asarray(corners, dtype=np.float32) / scale
            center = corners.mean(axis=0)
            duplicate = any(
                item["text"] == text
                and np.linalg.norm(np.asarray(item["center"], dtype=np.float32) - center)
                < 0.05 * max(image.shape[:2])
                for item in results
            )
            if duplicate:
                continue
            results.append(
                {
                    "text": text,
                    "points": corners.astype(int).tolist(),
                    "center": center.astype(int).tolist(),
                }
            )
    return results


def parse_code(text: str):
    """Return the 1~24 goods code, or None when the payload is not a valid code."""
    digits = "".join(char for char in text if char.isdigit())
    if not digits:
        return None
    code = int(digits)
    return code if 1 <= code <= 24 else None


def draw_results(image, results):
    """Draw QR polygons, decoded text and shelf positions onto a copy of the frame."""
    output = image.copy()
    if output.ndim == 2:
        output = cv2.cvtColor(output, cv2.COLOR_GRAY2BGR)

    for result in results:
        points = np.asarray(result["points"], dtype=np.int32).reshape(-1, 1, 2)
        cv2.polylines(output, [points], True, (0, 255, 0), 2)

        code = parse_code(result["text"])
        label = f"{result['text']} {code_to_position(code)}" if code else result["text"]
        label = "".join(char if char.isascii() and char.isprintable() else "?" for char in label)[:48]

        center = tuple(int(value) for value in result["center"])
        cv2.drawMarker(output, center, (0, 255, 255), cv2.MARKER_CROSS, 18, 2, cv2.LINE_AA)

        x, y = int(np.min(points, axis=0)[0][0]), int(np.min(points, axis=0)[0][1])
        y = max(y, 24)
        (text_width, text_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
        cv2.rectangle(output, (x, y - text_height - 10), (x + text_width + 8, y), (0, 0, 0), -1)
        cv2.putText(output, label, (x + 4, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    return output

# -*- coding: utf-8 -*-
"""二维码识别共享模块(纯 OpenCV, 不依赖 YOLO/torch/onnx)。

来源: `Desktop/qr_opencv_project/opencv_qr.py`(用户的识别方案), 这里抽出
`decode_qr` / `draw_results` 两个纯函数供机载端复用, 去掉命令行外壳与视频写出。
改识别算法请改本文件, 保持机载端(虚拟机 / 以后的 RDK X5)只用这一个入口。
"""
import cv2
import numpy as np

DETECTOR = cv2.QRCodeDetector()


def _variants(image, fast):
    """返回 (图像, 缩放比) 列表; 缩放比用于把角点坐标还原回原图尺度。"""
    variants = [(image, 1.0)]
    if fast:
        return variants

    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    variants.append((gray, 1.0))

    h, w = gray.shape
    if max(h, w) <= 1280:                  # 小图先放大一档, 提高小二维码召回
        scale = min(6.0, 1280 / max(h, w))
        if scale > 1.0:
            resized = cv2.resize(gray, None, fx=scale, fy=scale,
                                 interpolation=cv2.INTER_NEAREST)
            variants.append((resized, scale))

    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    variants.append((binary, 1.0))
    variants.append((cv2.bitwise_not(binary), 1.0))
    return variants


def decode_qr(image, fast=False):
    """解码图中的所有二维码。

    Args:
        image: BGR 或灰度图。
        fast: True 时只用原图, 不做灰度/放大/Otsu/反色变体(省算力, 召回略低)。

    Returns:
        [{"text": str, "points": [[x,y]×4], "center": [x,y]}, ...]
        坐标原点在图像左上角, x 向右、y 向下。
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
            # 同一码会被多个预处理变体重复检出, 同内容且中心接近的算一次
            duplicate = any(
                item["text"] == text
                and np.linalg.norm(np.asarray(item["center"], dtype=np.float32) - center)
                < 0.05 * max(image.shape[:2])
                for item in results
            )
            if duplicate:
                continue
            results.append({
                "text": text,
                "points": corners.astype(int).tolist(),
                "center": center.astype(int).tolist(),
            })
    return results


def draw_results(image, results):
    """把二维码框和解出的文本画到图上(返回新图, 不改原图)。"""
    output = image.copy()
    for result in results:
        points = np.asarray(result["points"], dtype=np.int32).reshape(-1, 1, 2)
        cv2.polylines(output, [points], True, (0, 255, 0), 2)

        x, y = np.min(points, axis=0)[0]
        text = result["text"].replace("\n", " ")
        text = "".join(c if c.isascii() and c.isprintable() else "?" for c in text)[:48]
        y = max(y, 24)
        (text_width, text_height), _ = cv2.getTextSize(
            text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.rectangle(output, (x, y - text_height - 10), (x + text_width + 8, y),
                      (0, 0, 0), -1)
        cv2.putText(output, text, (x + 4, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0), 2)
    return output

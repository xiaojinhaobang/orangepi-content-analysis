#!/usr/bin/env python3
"""把简易地图 + 航点表画成 PNG，便于核对。

用法: python3 draw_map.py <waypoints.yaml> <out.png> [px_per_cm]
依赖: PyYAML + Pillow
"""

import sys

import yaml
from PIL import Image, ImageDraw, ImageFont


def load_font(size):
    """优先找能显示中文的字体，找不到就用默认字体。"""
    for path in (
        r"C:\Windows\Fonts\msyh.ttc",
        r"C:\Windows\Fonts\simhei.ttf",
        "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    ):
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            continue
    return ImageFont.load_default()


def main():
    src, out = sys.argv[1], sys.argv[2]
    scale = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0
    cfg = yaml.safe_load(open(src, encoding="utf-8"))
    f = cfg["field"]
    x0, x1, y0, y1 = f["x_min"], f["x_max"], f["y_min"], f["y_max"]

    pad, top = 60, 40
    w, h = int((x1 - x0) * scale), int((y1 - y0) * scale)
    img = Image.new("RGB", (w + 2 * pad, h + top + pad), "white")
    d = ImageDraw.Draw(img)
    f_big, f_mid, f_small = load_font(22), load_font(18), load_font(15)

    def px(x, y):
        """场地位(cm, y 向上) -> 像素(y 向下)"""
        return (pad + (x - x0) * scale, top + (y1 - y) * scale)

    def box(xa, ya, xb, yb):
        """对角两点(cm) -> PIL 需要的 (left, top, right, bottom)"""
        p1, p2 = px(xa, ya), px(xb, yb)
        return [min(p1[0], p2[0]), min(p1[1], p2[1]),
                max(p1[0], p2[0]), max(p1[1], p2[1])]

    d.rectangle(box(x0, y0, x1, y1), outline="black", width=3)
    d.text((pad, 8), "field %g x %g cm" % (x1 - x0, y1 - y0), fill="black", font=f_big)

    for o in cfg.get("obstacles") or []:
        d.rectangle(box(o["x_min"], o["y_min"], o["x_max"], o["y_max"]),
                    fill=(220, 60, 60))
        d.text(px(o["x_min"], o["y_max"]), o.get("name", ""), fill=(120, 0, 0), font=f_small)

    pts = [px(float(wp["x"]), float(wp["y"])) for wp in cfg["waypoints"]]
    for a, b in zip(pts, pts[1:]):
        d.line([a, b], fill=(40, 90, 220), width=3)

    for i, (wp, p) in enumerate(zip(cfg["waypoints"], pts)):
        d.ellipse([p[0] - 6, p[1] - 6, p[0] + 6, p[1] + 6], fill=(20, 130, 20))
        label = "%d %s  z=%.0f" % (i, wp.get("name", ""), wp.get("z", 0))
        d.text((p[0] + 10, p[1] - 22), label, fill=(0, 90, 0), font=f_mid)

    o = px(x0, y0)
    d.line([o, px(x0 + 55, y0)], fill="black", width=3)
    d.line([o, px(x0, y0 + 55)], fill="black", width=3)
    d.text((o[0] + 58, o[1] - 12), "+x", fill="black", font=f_mid)
    d.text((o[0] + 6, o[1] - 64), "+y", fill="black", font=f_mid)
    d.text((o[0] + 8, o[1] + 6), "origin (0,0)", fill="black", font=f_small)

    d.text((pad, top + h + 14),
           "green dot = waypoint (index name z)   blue = flight path   red = obstacle",
           fill="black", font=f_small)
    img.save(out)
    print("saved", out, "%dx%d" % img.size,
          "| waypoints", len(cfg["waypoints"]),
          "| obstacles", len(cfg.get("obstacles") or []))


if __name__ == "__main__":
    main()

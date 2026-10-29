# -*- coding: utf-8 -*-
"""生成货架二维码打印表(A4 300DPI 带裁剪线) + 编号->货位对照表 num_cell.json。

一次输入, 两个产物:
  num_cell.json  给 vm_qr_link.py --num-map 用(二维码只含编号时补出坐标)
  qr_sheet_pN.png A4 打印页, 每页 6 张(2 列 x 3 行), 虚线裁剪框

用法:
    python make_qr_sheet.py --map 8=A1,11=A2,9=A3,7=A4,5=A5,6=A6
    python make_qr_sheet.py --map-file num_cell.json --out qr_sheets
    python make_qr_sheet.py --map 8=A1 --qr-size 600

--map 的方向是 编号=货位(和 num_cell.json 里的 {编号: "货位"} 一致)。
"""
import argparse
import json
from pathlib import Path

import qrcode
from PIL import Image, ImageDraw, ImageFont

DPI = 300
A4 = (int(8.27 * DPI), int(11.69 * DPI))       # 2480 x 3508
MARGIN = 120
GAP = 60
COLS, ROWS = 2, 3
PER_PAGE = COLS * ROWS

# 打印件不需要纠错余量拉满, M 级(15%)对贴纸污损/反光够用且码更疏
_ERR = qrcode.constants.ERROR_CORRECT_M

FONT_CANDIDATES = [
    "C:/Windows/Fonts/msyh.ttc",               # 微软雅黑
    "C:/Windows/Fonts/simhei.ttf",             # 黑体
    "C:/Windows/Fonts/arial.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
]


def load_font(size):
    for path in FONT_CANDIDATES:
        if Path(path).is_file():
            try:
                return ImageFont.truetype(path, size)
            except OSError:
                continue
    return ImageFont.load_default()


def make_qr(text, box):
    """生成二维码图, 边长 box 像素。

    必须用**整数倍**缩放: 直接 resize 到 box 会让模块宽窄不一(有的 3px 有的 2px),
    某些图案组合(实测 "7")会因此解不出来。这里按模块数取整数倍, 再居中留白。
    """
    qr = qrcode.QRCode(error_correction=_ERR, border=2)
    qr.add_data(text)
    qr.make(fit=True)
    total_modules = qr.modules_count + 2 * qr.border
    qr.box_size = max(1, box // total_modules)
    img = qr.make_image(fill_color="black", back_color="white").convert("L")
    canvas = Image.new("L", (box, box), 255)
    canvas.paste(img, ((box - img.width) // 2, (box - img.height) // 2))
    return canvas


def dashed_rect(draw, xy, dash=18, gap=12, fill=200, width=2):
    """虚线矩形(裁剪线)"""
    x0, y0, x1, y1 = xy
    for x in range(int(x0), int(x1), dash + gap):
        draw.line([(x, y0), (min(x + dash, x1), y0)], fill=fill, width=width)
        draw.line([(x, y1), (min(x + dash, x1), y1)], fill=fill, width=width)
    for y in range(int(y0), int(y1), dash + gap):
        draw.line([(x0, y), (x0, min(y + dash, y1))], fill=fill, width=width)
        draw.line([(x1, y), (x1, min(y + dash, y1))], fill=fill, width=width)


def parse_map(spec):
    """'8=A1,11=A2' -> {8: 'A1', 11: 'A2'}"""
    out = {}
    for part in (spec or "").replace(";", ",").split(","):
        part = part.strip()
        if not part:
            continue
        if "=" not in part:
            raise ValueError(f"bad entry '{part}', expect 编号=货位 like 8=A1")
        num, cell = part.split("=", 1)
        out[int(num.strip())] = cell.strip().upper()
    return out


def build_sheet(items, qr_size, page_no, total_pages):
    """items: [(编号, 货位), ...] 最多 PER_PAGE 条。返回 PIL Image。"""
    page = Image.new("L", A4, 255)
    draw = ImageDraw.Draw(page)
    card_w = (A4[0] - 2 * MARGIN - (COLS - 1) * GAP) // COLS
    card_h = (A4[1] - 2 * MARGIN - (ROWS - 1) * GAP) // ROWS

    for idx, (num, cell) in enumerate(items):
        r, c = divmod(idx, COLS)
        x0 = MARGIN + c * (card_w + GAP)
        y0 = MARGIN + r * (card_h + GAP)
        dashed_rect(draw, (x0, y0, x0 + card_w, y0 + card_h))

        # 左上角小字: 该贴到哪个货位(装配提示, 贴完可裁掉)
        draw.text((x0 + 24, y0 + 20), f"→ {cell}", font=load_font(56), fill=0)

        qr = make_qr(str(num), qr_size)
        qx = x0 + (card_w - qr_size) // 2
        qy = y0 + (card_h - qr_size) // 2
        page.paste(qr, (qx, qy))

        # 底部大号编号(人眼核对用)
        font = load_font(120)
        text = str(num)
        tw = draw.textbbox((0, 0), text, font=font)[2]
        draw.text((x0 + (card_w - tw) // 2, y0 + card_h - 170), text,
                  font=font, fill=0)

    draw.text((MARGIN, A4[1] - 90),
              f"D题货架二维码  page {page_no}/{total_pages}  "
              f"(300DPI, 沿虚线裁剪)",
              font=load_font(40), fill=0)
    return page


def main():
    ap = argparse.ArgumentParser(description="make QR print sheet + num_cell.json")
    ap.add_argument("--map", default=None, help="编号=货位, 逗号分隔: 8=A1,11=A2")
    ap.add_argument("--map-file", default=None, help="已有的 num_cell.json")
    ap.add_argument("--out", default="qr_sheets", help="输出目录")
    ap.add_argument("--json-out", default="num_cell.json",
                    help="对照表输出路径(给 --num-map 用)")
    ap.add_argument("--qr-size", type=int, default=700, help="二维码边长像素")
    args = ap.parse_args()

    if args.map:
        mapping = parse_map(args.map)
    elif args.map_file:
        raw = json.loads(Path(args.map_file).read_text(encoding="utf-8"))
        mapping = {}
        for k, v in raw.items():
            # 两种方向都收: {"8":"A1"} / {"A1":8}
            if str(v)[:1].upper() in "ABCD":
                mapping[int(k)] = str(v).upper()
            else:
                mapping[int(v)] = str(k).upper()
    else:
        raise SystemExit("need --map or --map-file")

    items = sorted(mapping.items(), key=lambda kv: (kv[1][0], int(kv[1][1:])))
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    total_pages = (len(items) + PER_PAGE - 1) // PER_PAGE
    for p in range(total_pages):
        chunk = items[p * PER_PAGE:(p + 1) * PER_PAGE]
        sheet = build_sheet(chunk, args.qr_size, p + 1, total_pages)
        path = out_dir / f"qr_sheet_p{p + 1}.png"
        sheet.save(path, dpi=(DPI, DPI))
        print(f"[sheet] {path}  {len(chunk)} codes: "
              + ", ".join(f"{n}@{c}" for n, c in chunk))

    json_path = Path(args.json_out)
    json_path.write_text(json.dumps({str(n): c for n, c in items},
                                    ensure_ascii=False, indent=2),
                         encoding="utf-8")
    print(f"[map]   {json_path}  {len(items)} entries")
    print(f"[map]   missing cells: "
          + (", ".join(c for c in
                       (f"{f}{p}" for f in "ABCD" for p in range(1, 7))
                       if c not in mapping.values()) or "none"))


if __name__ == "__main__":
    main()

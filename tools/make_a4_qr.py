"""生成题目同规格（19cm x 19cm）的 A4 二维码打印文件（PNG + PDF）。

用法: python make_a4_qr.py 6        # 生成编号 6 的码
输出: A4_QR_<编号>_19cm.png / .pdf（放在本脚本同目录）
要点: 模块尺寸取整数（box_size = 像素 // 总模块数），否则模块宽窄不一、解码不稳。
"""
import sys
from pathlib import Path

import qrcode
from PIL import Image, ImageDraw, ImageFont

CONTENT = sys.argv[1] if len(sys.argv) > 1 else "6"
SIZE_CM = float(sys.argv[2]) if len(sys.argv) > 2 else 19.0
DPI = 300
A4_PX = (2480, 3508)                      # A4 @300dpi
TARGET_PX = int(SIZE_CM / 2.54 * DPI)     # 19cm -> 2244px
OUT_DIR = Path(__file__).resolve().parent

qr = qrcode.QRCode(error_correction=qrcode.constants.ERROR_CORRECT_M,
                   box_size=1, border=4)
qr.add_data(CONTENT)
qr.make(fit=True)
modules = qr.modules_count
total_modules = modules + 8               # 含 4 模块静区
box = max(1, TARGET_PX // total_modules)  # 整数倍模块缩放
side = total_modules * box

code = qr.make_image(fill_color="black", back_color="white").convert("RGB")
code = code.resize((side, side), Image.NEAREST)

page = Image.new("RGB", A4_PX, "white")
page.paste(code, ((A4_PX[0] - side) // 2, (A4_PX[1] - side) // 2 - 150))

draw = ImageDraw.Draw(page)
try:
    font = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 48)
except Exception:                          # noqa: BLE001
    font = ImageFont.load_default()
text = f"编号 {CONTENT}   QR {modules}x{modules} 模块   边长 {side / DPI * 2.54:.1f} cm"
draw.text(((A4_PX[0] - draw.textlength(text, font=font)) / 2, 150), text,
          fill="black", font=font)
foot = "打印务必 100% 原尺寸（不要'适合页面'），用哑光纸"
draw.text(((A4_PX[0] - draw.textlength(foot, font=font)) / 2, A4_PX[1] - 220), foot,
          fill="gray", font=font)

png = OUT_DIR / f"A4_QR_{CONTENT}_{SIZE_CM:.0f}cm.png"
pdf = OUT_DIR / f"A4_QR_{CONTENT}_{SIZE_CM:.0f}cm.pdf"
page.save(png)
page.save(pdf, "PDF", resolution=DPI)
print(f"编号={CONTENT} 模块={modules}x{modules} 边长={side}px={side / DPI * 2.54:.1f}cm")
print("已生成:", png.name, "/", pdf.name)

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""画场地俯视图（方案基线口径，单位 cm，原点=场地左下角、x 向右、y 向上）。

用法： python tools/draw_field_map.py [输出png] [要高亮的货位,如 B1,B2,B3]
"""
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, Rectangle

plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei']
plt.rcParams['axes.unicode_minus'] = False

# ---- 方案基线几何（cm）----
W, H = 420.0, 420.0               # 2026-10-07 现场重测：7×7 块 60cm 砖
TAKEOFF = (60.0, 60.0)     # 与 mission_node.py 的 TAKEOFF_FIELD 保持一致
LANDING = (400.0, 400.0)   # 与 mission_node.py LANDING_FIELD 同步
PANEL_X = 180.0
PANEL_Y = (90.0, 300.0)           # 含框 210cm
COL_Y = (145.0, 195.0, 245.0)     # 三列中心
OBS_DIST = 40.0                   # 相机到板面（2026-10-06：35 → 30 → 40，给漂移留裕度）
CAM_FWD = 23.5                    # 相机在机体前
DETOUR_Y = 355.0                  # 绕行通道（板远端 300 + 55cm 余量）

OUT = sys.argv[1] if len(sys.argv) > 1 else 'simple_map/field_map.png'
HILITE = [s.strip().upper() for s in (sys.argv[2].split(',') if len(sys.argv) > 2 else [])]


def cell_label(face, idx):
    return '%s%d' % (face, idx)


fig, ax = plt.subplots(figsize=(7.2, 9.6))

# 场地
ax.add_patch(Rectangle((0, 0), W, H, fill=False, ec='#333', lw=2))
ax.text(W / 2, H + 8, '场地 420 × 420 cm（7×7 块 60cm 砖；原点=左下角，x 向右，y 向上）',
        ha='center', fontsize=11)

# 白板（俯视是一条线，两侧面稍微错开画）
ax.plot([PANEL_X, PANEL_X], [PANEL_Y[0], PANEL_Y[1]], color='#444', lw=6, solid_capstyle='butt')
ax.text(PANEL_X, PANEL_Y[1] + 8, '白板（含框 210cm）', ha='center', fontsize=10)
ax.text(PANEL_X - 6, PANEL_Y[1] - 20, 'B 面', ha='right', color='#1a73e8', fontsize=10, fontweight='bold')
ax.text(PANEL_X + 6, PANEL_Y[1] - 20, 'A 面', ha='left', color='#188038', fontsize=10, fontweight='bold')

# 12 个货位
for c, y in enumerate(COL_Y, start=1):
    for row, dz in ((0, 0), (1, 1)):
        pos = c + row * 3
        for face, dx, col in (('B', -8, '#1a73e8'), ('A', 8, '#188038')):
            name = cell_label(face, pos)
            hot = name in HILITE
            ax.add_patch(Rectangle((PANEL_X + dx - 4, y - 10), 8, 20,
                                   fc=col if hot else 'white', ec=col,
                                   lw=2.0 if hot else 1.0, alpha=0.95 if hot else 0.9, zorder=3))
            ax.text(PANEL_X + dx, y + (14 if face == 'B' else -18), name,
                    ha='center', va='center', fontsize=8.5,
                    color=col, fontweight='bold' if hot else 'normal')

# 起降点
ax.add_patch(Rectangle((TAKEOFF[0] - 25, TAKEOFF[1] - 25), 50, 50, fc='#333', zorder=4))
ax.text(TAKEOFF[0], TAKEOFF[1], '起飞点', color='white', ha='center', va='center', fontsize=9, zorder=5)
ax.text(TAKEOFF[0], TAKEOFF[1] - 34, '(30, 80)', ha='center', fontsize=8.5, color='#333')
ax.add_patch(plt.Circle(LANDING, 25, fc='none', ec='#333', lw=2, zorder=4))
ax.text(LANDING[0], LANDING[1], '降落点', ha='center', va='center', fontsize=9, zorder=5)
ax.text(LANDING[0], LANDING[1] - 34, '(225, 345)', ha='center', fontsize=8.5, color='#333')

# 观察位置（相机距板面 35cm → 机体在 x = 150 - 35 - 23.5 = 91.5）
body_x = PANEL_X - OBS_DIST - CAM_FWD
prev = TAKEOFF
for c, y in enumerate(COL_Y, start=1):
    name = 'B%d' % c
    hot = name in HILITE
    ax.add_patch(plt.Circle((body_x, y), 6, fc='#e37400' if hot else 'white',
                            ec='#e37400', lw=2 if hot else 1.2, zorder=6))
    ax.text(body_x - 12, y, 'B%d 观察位 (%.1f, %.0f)' % (c, body_x, y), ha='right', va='center',
            fontsize=9, color='#e37400' if hot else '#666',
            fontweight='bold' if hot else 'normal')
    if hot or not HILITE:
        ax.add_patch(FancyArrowPatch(prev, (body_x, y), arrowstyle='-|>', mutation_scale=12,
                                     color='#e37400', lw=1.6, zorder=5))
    prev = (body_x, y)

# 相机到板面 35cm 标注
ax.annotate('', xy=(PANEL_X, COL_Y[0]), xytext=(body_x, COL_Y[0]),
            arrowprops=dict(arrowstyle='<|-|>', color='#999', lw=1.2))
ax.text((PANEL_X + body_x) / 2, COL_Y[0] + 6, '%.0fcm（相机到板面）' % OBS_DIST,
        ha='center', fontsize=8.5, color='#999')

# 绕行段（扫完 B 面 → 沿板面到板远端之外 → 横穿到降落侧 → 降落）
if HILITE:
    last_y = COL_Y[-1]
    det = [(body_x, last_y), (body_x, DETOUR_Y), (LANDING[0], DETOUR_Y), LANDING]
    for a, b in zip(det, det[1:]):
        ax.add_patch(FancyArrowPatch(a, b, arrowstyle='-|>', mutation_scale=12,
                                     color='#d93025', lw=1.8, ls='--', zorder=5))
ax.text(body_x - 12, DETOUR_Y + 8, '绕行通道 y=%.0f（板远端 300 + 55）' % DETOUR_Y,
            ha='left', fontsize=8.5, color='#d93025', fontweight='bold')

ax.set_xlim(-70, W + 30)
ax.set_ylim(-40, H + 40)
ax.set_aspect('equal')
ax.set_xlabel('x (cm)')
ax.set_ylabel('y (cm)')
title = 'D题 场地俯视图' + ('　—　本场航线：%s' % ' / '.join(HILITE) if HILITE else '')
ax.set_title(title, fontsize=13)
ax.grid(alpha=0.25, linestyle=':')
plt.tight_layout()
plt.savefig(OUT, dpi=130)
print('saved', OUT)

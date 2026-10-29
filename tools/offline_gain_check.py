#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""offline_gain_check — 离线对比「现在的位置环参数」vs「降增益参数」会发出多大的修正速度。

背景：起点悬停失控是正反馈——雷达位置滑移 → 位置环追 → 飞机真动 → 雷达更差。
在"无法早期识别雷达位姿错"的前提下，最直接的软件手段是**压低环增益**：
  kp 0.35→0.15、vmax 12→6、位置参考限速 5cm/s

本脚本用已有日志回放三种配置，输出各自发出的修正速度峰值/均值/95 分位。
（动作变小 = 正反馈的驱动变小）

用法：python3 tools/offline_gain_check.py
"""
import glob
import math
import os
import re

RE_T = re.compile(r"^t=\s*([-\d.]+)")
RE_X = re.compile(r"x=\s*([-\d.]+)")
RE_Y = re.compile(r"y=\s*([-\d.]+)")
RE_H = re.compile(r"h=(\d+)")
RE_UN = re.compile(r"unlock=(\d)")

CFG = [
    ("现在  kp=0.35 vmax=12 无参考限速", 0.35, 12.0, 0.0),
    ("新方案 kp=0.15 vmax=6 参考限速5", 0.15, 6.0, 5.0),
]


def parse(path):
    rows = []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            if not line.startswith("t="):
                continue
            m_t, m_x, m_y = RE_T.search(line), RE_X.search(line), RE_Y.search(line)
            if not (m_t and m_x and m_y):
                continue
            m_h, m_u = RE_H.search(line), RE_UN.search(line)
            rows.append({"t": float(m_t.group(1)), "x": float(m_x.group(1)),
                         "y": float(m_y.group(1)),
                         "h": int(m_h.group(1)) if m_h else -1,
                         "un": int(m_u.group(1)) if m_u else -1})
    return rows


def windows(rows, min_n=40):
    out, cur = [], []
    for r in rows:
        if r["un"] == 1 and r["h"] >= 100:
            cur.append(r)
        else:
            if len(cur) >= min_n:
                out.append(cur)
            cur = []
    if len(cur) >= min_n:
        out.append(cur)
    return out


def sim(w, kp, vmax, rate):
    x0, y0 = w[0]["x"], w[0]["y"]
    rx, ry = x0, y0
    cmds = []
    for i, r in enumerate(w):
        if i > 0:
            p = w[i - 1]
            dt = max(1e-3, min(0.5, r["t"] - p["t"]))
            if rate > 0:
                step = rate * dt
                dx, dy = r["x"] - rx, r["y"] - ry
                d = math.hypot(dx, dy)
                if d > step and d > 0:
                    rx += dx / d * step
                    ry += dy / d * step
                else:
                    rx, ry = r["x"], r["y"]
            else:
                rx, ry = r["x"], r["y"]
            ex, ey = rx - x0, ry - y0
            v = min(vmax, kp * math.hypot(ex, ey))
            cmds.append(v)
    if not cmds:
        return (0.0, 0.0, 0.0)
    cmds.sort()
    return (cmds[-1], sum(cmds) / len(cmds), cmds[int(0.95 * (len(cmds) - 1))])


def main():
    paths = sorted(glob.glob("flight_data/offline/pg_*.txt")) + \
            sorted(glob.glob("flight_data/offline/fz_*.txt"))
    for p in paths:
        ws = windows(parse(p))
        if not ws:
            continue
        print("=== %s" % os.path.basename(p))
        for w in ws:
            print("  %.1fs 窗口" % (w[-1]["t"] - w[0]["t"]))
            for name, kp, vmax, rate in CFG:
                mx, mean, p95 = sim(w, kp, vmax, rate)
                print("    %-32s 峰值 %5.1f  均值 %5.1f  95%% %5.1f  cm/s"
                      % (name, mx, mean, p95))


if __name__ == "__main__":
    main()

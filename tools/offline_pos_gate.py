#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""offline_pos_gate — 离线验证「位置校验门」能否挡住雷达位置在起点飞行时的加速滑移。

背景（2026-10-16 实测）：
  · 起点悬停：雷达位置速度 6→13→17→24→42 cm/s 单调加速（正反馈发散），最大横移 28.6cm
  · 空手快推 1 米（40~83cm/s）：雷达位置跟得住、无跳变 → 所以不是"运动"本身，是桨转起来后位姿估计崩
  · 同一时间我们的速度指令最大只有 vmax=12cm/s

判据（本脚本实现）：
  飞机物理上不可能跑得比"我方速度指令 + 余量"快（飞控速度环就是跟着我们的指令走的）。
  所以：|v_雷达位置| > |我方指令| + MARGIN  →  该帧雷达位置不可信 → 丢弃，
        位置改用「我方指令按融合航向递推」（dead-reckon），不再追那条假滑移。

输出：每段悬停窗口对比「原始雷达最大横移」vs「过门控后最大横移」+ 拒绝帧数。

用法：python3 tools/offline_pos_gate.py [文件...]
环境变量：GATE_MARGIN=8（cm/s）
"""
import glob
import math
import os
import re
import sys

RE_T = re.compile(r"^t=\s*([-\d.]+)")
RE_X = re.compile(r"x=\s*([-\d.]+)")
RE_Y = re.compile(r"y=\s*([-\d.]+)")
RE_PSI = re.compile(r"psi=\s*([-\d.]+)")
RE_TVD = re.compile(r"tvd=\(\s*([-\d.]+),\s*([-\d.]+)\)")
RE_H = re.compile(r"h=(\d+)")
RE_UN = re.compile(r"unlock=(\d)")

MARGIN = float(os.environ.get("GATE_MARGIN", "8.0"))    # cm/s


def wrap(d):
    while d > 180.0:
        d -= 360.0
    while d < -180.0:
        d += 360.0
    return d


def parse(path):
    rows = []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            if not line.startswith("t="):
                continue
            m_t, m_x, m_y = RE_T.search(line), RE_X.search(line), RE_Y.search(line)
            if not (m_t and m_x and m_y):
                continue
            m_p, m_v, m_h, m_u = (RE_PSI.search(line), RE_TVD.search(line),
                                  RE_H.search(line), RE_UN.search(line))
            rows.append({
                "t": float(m_t.group(1)), "x": float(m_x.group(1)), "y": float(m_y.group(1)),
                "psi": float(m_p.group(1)) if m_p else 0.0,
                "vx": float(m_v.group(1)) if m_v else 0.0,
                "vy": float(m_v.group(2)) if m_v else 0.0,
                "h": int(m_h.group(1)) if m_h else -1,
                "un": int(m_u.group(1)) if m_u else -1,
            })
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


def run_gate(w):
    """返回 (原始最大横移, 门控后最大横移, 拒绝帧数, 帧数)"""
    x0, y0 = w[0]["x"], w[0]["y"]
    ax, ay = x0, y0                  # 门控后的位置
    raw_max = 0.0
    acc_max = 0.0
    rej = 0
    for i, r in enumerate(w):
        raw_max = max(raw_max, math.hypot(r["x"] - x0, r["y"] - y0))
        if i > 0:
            p = w[i - 1]
            dt = r["t"] - p["t"]
            if dt <= 0:
                dt = 0.05
            v_slam = math.hypot(r["x"] - p["x"], r["y"] - p["y"]) / dt
            v_cmd = math.hypot(r["vx"], r["vy"])
            if v_slam > v_cmd + MARGIN:
                # 丢弃雷达位置，用我方指令按航向递推
                rej += 1
                c, s = math.cos(math.radians(r["psi"])), math.sin(math.radians(r["psi"]))
                mx = r["vx"] * c + r["vy"] * s        # 机体系 → 地图系
                my = -r["vx"] * s + r["vy"] * c
                ax += mx * dt
                ay += my * dt
            else:
                ax, ay = r["x"], r["y"]
        acc_max = max(acc_max, math.hypot(ax - x0, ay - y0))
    return raw_max, acc_max, rej, len(w)


def main():
    paths = sys.argv[1:] or sorted(glob.glob("flight_data/offline/pg_*.txt"))
    print("GATE: |v_雷达| > |v_指令| + %.1f cm/s → 丢弃该帧雷达位置（改用指令递推）" % MARGIN)
    for p in paths:
        rows = parse(p)
        ws = windows(rows)
        print("=" * 74)
        print("%s   悬停窗口 %d 段" % (os.path.basename(p), len(ws)))
        for w in ws:
            raw, acc, rej, n = run_gate(w)
            print("  %.1fs (%d 帧): 原始最大横移 %6.1f cm  →  门控后 %6.1f cm   (拒绝 %d 帧, %.0f%%)"
                  % (w[-1]["t"] - w[0]["t"], n, raw, acc, rej, 100.0 * rej / n))


if __name__ == "__main__":
    main()

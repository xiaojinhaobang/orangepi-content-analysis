#!/usr/bin/env python3
"""航线/平移航段报告：把飞行按"移动/停顿"切段，量每段的速度、航向、高度。

用途：评估"各个方向前进飞得稳不稳、快不快"。
用法: python tools/route_report.py <trace.log> [最小移动速度 cm/s，默认 3]
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dump_flight_segments import parse          # noqa: E402


def unwrap_deg(a):
    out = [a[0]]
    for v in a[1:]:
        d = v - out[-1]
        while d > 180.0:
            d -= 360.0
        while d < -180.0:
            d += 360.0
        out.append(out[-1] + d)
    return np.array(out)


def smooth(a, n=5):
    if len(a) < n:
        return a
    k = np.ones(n) / n
    return np.convolve(a, k, mode="same")


def main():
    path = sys.argv[1]
    vth = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
    rows = [r for r in parse(path)
            if r["t"] is not None and not math.isnan(r["x"]) and not math.isnan(r["y"])]
    fly = [r for r in rows if not math.isnan(r["h"]) and r["h"] > 20
           and str(r["unlock"]) == "1"]
    if len(fly) < 100:
        print("%s: 飞行样本太少（%d）" % (os.path.basename(path), len(fly)))
        return
    t = np.array([r["t"] for r in fly])
    x = np.array([r["x"] for r in fly])
    y = np.array([r["y"] for r in fly])
    psi = unwrap_deg([r["psi"] for r in fly])
    h = np.array([r["h"] for r in fly])
    tvx = np.array([r["tvx"] for r in fly])
    tvy = np.array([r["tvy"] for r in fly])
    yawv = np.array([r["yawv"] for r in fly])
    dt = float(np.median(np.diff(t)))
    vx, vy = smooth(np.gradient(x, dt)), smooth(np.gradient(y, dt))
    spd = np.hypot(vx, vy)
    spd_s = smooth(spd, 11)
    tvc = np.hypot(tvx, tvy)

    moving = spd_s > vth
    print("== %s ==" % os.path.basename(path))
    print("  飞行 %.1f~%.1f s（%.0fs）  总路程 %.0f cm  最大速度 %.1f cm/s"
          % (t[0], t[-1], t[-1] - t[0],
             float(np.sum(spd_s) * dt), float(spd_s.max())))
    print("  平移时: 平均速度 %.1f cm/s   平均指令 %.1f cm/s   → 速度跟踪增益 %.2f"
          % (spd_s[moving].mean() if moving.any() else math.nan,
             tvc[moving].mean() if moving.any() else math.nan,
             (spd_s[moving].mean() / tvc[moving].mean())
             if moving.any() and tvc[moving].mean() > 0.1 else math.nan))
    print("  移动时间占比 %.0f%%   高度: 均值 %.0f  范围 %.0f~%.0f cm"
          % (100 * moving.mean(), h.mean(), h.min(), h.max()))

    # 切段
    segs = []
    i = 0
    n = len(moving)
    while i < n:
        if moving[i]:
            j = i
            while j < n and moving[j]:
                j += 1
            if (t[j - 1] - t[i]) >= 0.6:      # 太短的丢掉
                segs.append((i, j))
            i = j
        else:
            i += 1
    print("  共 %d 个移动航段：" % len(segs))
    print("    #   时段(s)    时长  路程   均速  峰速 | 航向变化  峰峰  纯漂 | 直线度 | 上升 滑行 | 高度变化")
    for k, (i, j) in enumerate(segs, 1):
        dur = t[j - 1] - t[i]
        dist = float(np.sum(spd_s[i:j]) * dt)
        dp = psi[j - 1] - psi[i]
        pp = psi[i:j].max() - psi[i:j].min()
        seg_yaw = np.abs(yawv[i:j])
        steady = seg_yaw < 5.0            # 排除开环转头（|yaw|≈10°/s）窗口
        if steady.sum() >= 20:
            ps = psi[i:j][steady]
            pp2 = ps.max() - ps.min()
        else:
            pp2 = math.nan
        dh = h[j - 1] - h[i]
        # 直线度：到首尾连线的最大垂距（偏航摆得凶时这个会跟着大）
        ax, ay = x[i], y[i]
        bx2, by2 = x[j - 1], y[j - 1]
        L = math.hypot(bx2 - ax, by2 - ay)
        if L > 5:
            dev = np.abs((bx2 - ax) * (ay - y[i:j]) - (ax - x[i:j]) * (by2 - ay)) / L
            straight = float(dev.max())
        else:
            straight = math.nan
        # 速度上升：指令首次 >3 起，到达到该段中位速度的 90% 用了多久
        rise = math.nan
        idx = np.nonzero(tvc[i:j] > 3)[0]
        if len(idx) > 2:
            m = np.median(spd_s[i:j][spd_s[i:j] > vth]) if (spd_s[i:j] > vth).any() else 0
            tgt = 0.9 * m
            hit = np.nonzero(spd_s[i + idx[0]:j] >= tgt)[0]
            if len(hit):
                rise = hit[0] * dt
        # 停止滑行：指令回落到 <1 之后，速度降到 2cm/s 以下前的位移
        glide = math.nan
        zc = np.nonzero(tvc[i:j] < 1)[0]
        if len(zc) > 2:
            z0 = i + zc[-1] if zc[-1] > len(tvc[i:j]) * 0.5 else i + zc[len(zc) // 2]
            stop = np.nonzero(spd_s[z0:j] < 2.0)[0]
            if len(stop):
                glide = float(np.sum(spd_s[z0:z0 + stop[0]]) * dt)
        print("   %2d  %6.1f~%6.1f %5.1fs %6.0f %5.1f %5.1f | %+8.1f° %6.1f° %6.1f° | %5.1fcm | %4.1f %5.1f | %+6.0f cm"
              % (k, t[i], t[j - 1], dur, dist, spd_s[i:j].mean(), spd_s[i:j].max(),
                 dp, pp, pp2, straight, rise, glide, dh))
    mv = [s for s in segs]
    if mv:
        dps = [psi[j - 1] - psi[i] for i, j in mv]
        pps = [psi[i:j].max() - psi[i:j].min() for i, j in mv]
        sps = [spd_s[i:j].mean() for i, j in mv]
        print("  平移段统计: 均速 %.1f cm/s（%.1f~%.1f） | 航向变化 |Δ| 中位 %.1f° 最大 %.1f°"
              " | 航向峰峰 中位 %.1f° 最大 %.1f°"
              % (np.mean(sps), np.min(sps), np.max(sps),
                 np.median(np.abs(dps)), np.max(np.abs(dps)),
                 np.median(pps), np.max(pps)))


if __name__ == "__main__":
    main()

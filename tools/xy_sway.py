#!/usr/bin/env python3
"""量悬停段的水平慢摆：幅值 / 峰峰 / 主周期 / 与偏航的耦合。

用法: python tools/xy_sway.py <trace.log> <t_lo> <t_hi> [标签]
"""
import math
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dump_flight_segments import parse          # noqa: E402

RE_FCV = re.compile(r"fcVelX=\s*(-?[\d.]+)\s+fcVelY=\s*(-?[\d.]+)")
RE_FCA = re.compile(r"fcAccX=\s*(-?[\d.]+)\s+fcAccY=\s*(-?[\d.]+)")


def parse_fc(path, lo, hi):
    """独立解析飞控自己的速度/加速度估计（用来判断雷达位置是不是在飘）。"""
    rows = []
    with open(path, errors="ignore", encoding="utf-8") as fh:
        for line in fh:
            mt = re.search(r"t=\s*(-?[\d.]+)", line)
            if not mt:
                continue
            t = float(mt.group(1))
            if not (lo <= t <= hi):
                continue
            mv = RE_FCV.search(line)
            ma = RE_FCA.search(line)
            if not mv:
                continue
            rows.append((t, float(mv.group(1)), float(mv.group(2)),
                         float(ma.group(1)) if ma else math.nan,
                         float(ma.group(2)) if ma else math.nan))
    return rows


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


def dom_period(sig, dt, fmin=0.03, fmax=1.2):
    """去均值后的主周期（秒）+ 该频率上的能量占比。"""
    x = np.asarray(sig, dtype=float)
    x = x - x.mean()
    n = len(x)
    if n < 16:
        return math.nan, math.nan
    w = np.hanning(n)
    f = np.fft.rfftfreq(n, dt)
    p = np.abs(np.fft.rfft(x * w)) ** 2
    band = (f >= fmin) & (f <= fmax)
    if not band.any() or p[band].sum() <= 0:
        return math.nan, math.nan
    i = np.argmax(np.where(band, p, 0))
    frac = p[i] / p[band].sum()
    return (1.0 / f[i] if f[i] > 0 else math.nan), frac


def main():
    path, lo, hi = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
    tag = sys.argv[4] if len(sys.argv) > 4 else os.path.basename(path)
    rows = [r for r in parse(path)
            if r["t"] is not None and lo <= r["t"] <= hi
            and not math.isnan(r["x"]) and not math.isnan(r["y"])]
    if len(rows) < 50:
        print("%s: 样本太少（%d）" % (tag, len(rows)))
        return
    t = np.array([r["t"] for r in rows])
    x = np.array([r["x"] for r in rows])
    y = np.array([r["y"] for r in rows])
    psi = unwrap_deg([r["psi"] for r in rows])
    h = np.array([r["h"] if not math.isnan(r["h"]) else np.nan for r in rows])
    gz = np.array([r["gzz"] / 18.4 for r in rows])
    dt = float(np.median(np.diff(t)))

    xc, yc = x - x.mean(), y - y.mean()
    taux, fx = dom_period(xc, dt)
    tauy, fy = dom_period(yc, dt)
    taup, fp = dom_period(psi, dt)
    taug, fg = dom_period(gz, dt)

    def corr(a, b):
        a, b = np.asarray(a, float), np.asarray(b, float)
        a, b = a - a.mean(), b - b.mean()
        d = math.sqrt((a * a).sum() * (b * b).sum())
        return float((a * b).sum() / d) if d > 0 else math.nan

    print("== %s ==" % tag)
    print("  窗口 %.1f~%.1f s（%.1f s，%d 样，dt=%.3f s）  高度 均值 %.0f cm"
          % (lo, hi, hi - lo, len(rows), dt, np.nanmean(h)))
    print("  水平: x 峰峰 %5.1f cm   y 峰峰 %5.1f cm   合成 %5.1f cm"
          % (xc.max() - xc.min(), yc.max() - yc.min(),
             math.hypot(xc.max() - xc.min(), yc.max() - yc.min())))
    print("  水平 sd: x %4.1f  y %4.1f cm        | 偏航: psi 峰峰 %5.1f°  sd %.1f°/s"
          % (xc.std(), yc.std(), psi.max() - psi.min(), gz.std()))
    print("  主周期: x %5.1f s (能量 %.0f%%)   y %5.1f s (%.0f%%)   psi %5.1f s   gz %5.1f s"
          % (taux, 100 * fx, tauy, 100 * fy, taup, taug))
    print("  耦合: corr(x,psi)=%+.2f  corr(y,psi)=%+.2f  corr(x,gz)=%+.2f  corr(y,gz)=%+.2f"
          % (corr(xc, psi), corr(yc, psi), corr(xc, gz), corr(yc, gz)))
    print("        corr(x,y)=%+.2f   corr(x,h)=%+.2f  corr(y,h)=%+.2f"
          % (corr(xc, yc), corr(xc, h), corr(yc, h)))

    # ---- 我们的水平指令（tvd = pos_hold 发给飞控的机体系速度 cm/s）----
    tvx = np.array([r["tvx"] for r in rows])
    tvy = np.array([r["tvy"] for r in rows])
    if not np.isnan(tvx).all():
        vx = np.gradient(x, dt)
        vy = np.gradient(y, dt)
        def best_lag(cmd, vel):
            """指令 → 位置速度 的最佳延迟（±0.5 s）。"""
            out = (math.nan, math.nan)
            for lag in range(-10, 11):
                a = cmd if lag >= 0 else cmd[-lag:]
                b = vel[lag:] if lag >= 0 else vel[:lag]
                n = min(len(a), len(b))
                if n < 20:
                    continue
                c = corr(a[:n], b[:n])
                if math.isnan(out[1]) or abs(c) > abs(out[1]):
                    out = (lag * dt, c)
            return out

        bx = best_lag(tvx, vx)
        by = best_lag(tvy, vy)
        zx = int(np.sum(np.abs(tvx) < 0.5))
        zy = int(np.sum(np.abs(tvy) < 0.5))
        print("  指令: tvx 均值 %+.2f 峰峰 %5.2f | tvy 均值 %+.2f 峰峰 %5.2f cm/s"
              % (tvx.mean(), tvx.max() - tvx.min(), tvy.mean(), tvy.max() - tvy.min()))
        print("        |tvx|<0.5 占 %d%%   |tvy|<0.5 占 %d%%   （=被 1cm/s 分辨率吃掉）"
              % (100 * zx / len(tvx), 100 * zy / len(tvy)))
        print("        corr(tvx, dx/dt): 延迟 %+.2f s → %+.2f    corr(tvy, dy/dt): 延迟 %+.2f s → %+.2f"
              % (bx[0], bx[1], by[0], by[1]))

    # ---- 独立校验：雷达位置 vs 飞控自己的速度（判断"真动"还是"估计飘"）----
    fc = parse_fc(path, lo, hi)
    if len(fc) > 50:
        ft = np.array([r[0] for r in fc])
        fvx = np.array([r[1] for r in fc])
        fvy = np.array([r[2] for r in fc])
        fdt = float(np.median(np.diff(ft)))
        ix = np.cumsum(fvx) * fdt          # 飞控速度积分 → 相对位移(cm)
        iy = np.cumsum(fvy) * fdt
        ix = ix - ix.mean()
        iy = iy - iy.mean()
        # 与雷达位置对齐（同长度时直接比；否则按时间插值）
        n = min(len(xc), len(ix))
        print("  飞控侧: 速度均值 (%+.2f, %+.2f) cm/s  积分位移峰峰 (%.1f, %.1f) cm"
              % (fvx.mean(), fvy.mean(), ix.max() - ix.min(), iy.max() - iy.min()))
        print("  对比 : corr(雷达x, 飞控积分x)=%+.2f   corr(雷达y, 飞控积分y)=%+.2f"
              % (corr(xc[:n], ix[:n]), corr(yc[:n], iy[:n])))


if __name__ == "__main__":
    main()

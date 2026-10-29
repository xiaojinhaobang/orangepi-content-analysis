#!/usr/bin/env python3
"""Analyse a throttle sweep: for each PWM band, how fast does the map pose move
and how noisy is the gyro? Detects the drift onset."""
import argparse
import math
import re
from statistics import median

RE_T = re.compile(r"t=\s*(-?[\d.]+)")
RE_XY = re.compile(r"x=\s*(-?[\d.]+|nan)\s+y=\s*(-?[\d.]+|nan)")
RE_PWM = re.compile(r"fcPwm=\(\s*(-?[\d.]+|nan),\s*(-?[\d.]+|nan),\s*(-?[\d.]+|nan),\s*(-?[\d.]+|nan)\)")
RE_G = re.compile(r"fcGyroXY=\(\s*(-?[\d.]+|nan),\s*(-?[\d.]+|nan)\)")


def num(s):
    try:
        return float(s)
    except ValueError:
        return float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--width", type=float, default=50.0)
    args = ap.parse_args()

    rows = []
    with open(args.log, "r", encoding="utf-8", errors="ignore") as fh:
        for line in fh:
            mt, mp, mw = RE_T.search(line), RE_XY.search(line), RE_PWM.search(line)
            if not (mt and mp and mw):
                continue
            x, y = num(mp.group(1)), num(mp.group(2))
            if x != x or y != y:
                continue
            pwm = [num(mw.group(i)) for i in range(1, 5)]
            if any(v != v for v in pwm):
                continue
            mg = RE_G.search(line)
            g = math.hypot(num(mg.group(1)), num(mg.group(2))) if mg else float("nan")
            rows.append((num(mt.group(1)), x * 100, y * 100, sum(pwm) / 4.0,
                         max(pwm) - min(pwm), g))
    if len(rows) < 100:
        print("样本不足")
        return

    # 地图速度：±5 样本（约 0.5 s）
    for i, r in enumerate(rows):
        a, b = max(0, i - 5), min(len(rows) - 1, i + 5)
        span = rows[b][0] - rows[a][0]
        r_speed = math.hypot(rows[b][1] - rows[a][1], rows[b][2] - rows[a][2]) / span if span > 0.2 else float("nan")
        rows[i] = r + (r_speed,)

    bins = {}
    for r in rows:
        if r[6] != r[6]:
            continue
        b = int(r[3] // args.width) * int(args.width)
        bins.setdefault(b, []).append(r)

    print(f"{args.log}")
    print(f"  总样本 {len(rows)}   时间 {rows[0][0]:.0f}~{rows[-1][0]:.0f}s")
    print("  PWM段   样本  位姿速度中位  p90   陀螺噪声(中位)  四路极差")
    for b in sorted(bins):
        s = bins[b]
        if len(s) < 20:
            continue
        v = sorted(r[6] for r in s)
        g = sorted(r[5] for r in s if r[5] == r[5])
        sp = sorted(r[4] for r in s)
        print(f"  {b:4.0f}-{b + args.width:4.0f} {len(s):6d} {v[len(v)//2]:10.1f} {v[int(len(v)*0.9)]:6.1f} "
              f"{g[len(g)//2] if g else float('nan'):12.0f} {sp[len(sp)//2]:9.1f}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Measure the period/amplitude of the map-pose oscillation during a static window."""
import argparse
import math
import re

RE_T = re.compile(r"t=\s*(-?[\d.]+)")
RE_XY = re.compile(r"x=\s*(-?[\d.]+|nan)\s+y=\s*(-?[\d.]+|nan)")


def num(s):
    try:
        return float(s)
    except ValueError:
        return float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--t0", type=float, required=True)
    ap.add_argument("--t1", type=float, required=True)
    args = ap.parse_args()

    ts, xs, ys = [], [], []
    with open(args.log, "r", encoding="utf-8", errors="ignore") as fh:
        for line in fh:
            mt, mp = RE_T.search(line), RE_XY.search(line)
            if not (mt and mp):
                continue
            t = num(mt.group(1))
            if not (args.t0 <= t <= args.t1):
                continue
            x, y = num(mp.group(1)), num(mp.group(2))
            if x != x or y != y:
                continue
            ts.append(t)
            xs.append(x * 100)
            ys.append(y * 100)
    if len(xs) < 40:
        print("样本太少")
        return

    dt = (ts[-1] - ts[0]) / (len(ts) - 1)
    mx = sum(xs) / len(xs)
    my = sum(ys) / len(ys)
    dx = [v - mx for v in xs]
    dy = [v - my for v in ys]
    print(f"窗口 {ts[0]:.1f}~{ts[-1]:.1f}s  {len(xs)} 样本  采样 {1 / dt:.1f} Hz")
    print(f"x 均值 {mx:.1f} cm 峰峰 {max(xs) - min(xs):.1f} cm   "
          f"y 均值 {my:.1f} cm 峰峰 {max(ys) - min(ys):.1f} cm")

    # 自相关找主周期
    def acf(series, lag):
        n = len(series)
        num_ = sum(series[i] * series[i + lag] for i in range(n - lag))
        den = sum(v * v for v in series)
        return num_ / den if den else 0.0

    best = []
    for lag in range(2, min(200, len(dx) // 2)):
        r = acf(dx, lag) + acf(dy, lag)
        best.append((r, lag))
    best.sort(reverse=True)
    print("\n最强的几个周期（自相关峰）：")
    shown = []
    for r, lag in best:
        period = lag * dt
        if period < 0.4:
            continue
        if any(abs(period - p) < 0.15 for p in shown):
            continue
        shown.append(period)
        print(f"  周期 {period:5.2f}s  相关 {r / 2:5.2f}")
        if len(shown) >= 3:
            break


if __name__ == "__main__":
    main()

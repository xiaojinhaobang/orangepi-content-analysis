#!/usr/bin/env python3
"""Find segments where motors ran while the aircraft was NOT flying (bench spin-up),
and measure how far the map pose moved during them."""
import argparse
import glob
import math
import os
import re

RE_T = re.compile(r"t=\s*(-?[\d.]+)")
RE_XY = re.compile(r"x=\s*(-?[\d.]+|nan)\s+y=\s*(-?[\d.]+|nan)")
RE_PWM = re.compile(r"fcPwm=\(\s*(-?[\d.]+|nan),\s*(-?[\d.]+|nan),\s*(-?[\d.]+|nan),\s*(-?[\d.]+|nan)\)")
RE_THR = re.compile(r"fcThr=\s*(-?[\d.]+|nan)")


def num(s):
    try:
        return float(s)
    except ValueError:
        return float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs="+")
    args = ap.parse_args()
    files = []
    for p in args.paths:
        files.extend(sorted(glob.glob(p)))

    for f in files:
        rows = []
        with open(f, "r", encoding="utf-8", errors="ignore") as fh:
            for line in fh:
                mt, mp = RE_T.search(line), RE_XY.search(line)
                if not (mt and mp):
                    continue
                x, y = num(mp.group(1)), num(mp.group(2))
                if x != x or y != y:
                    continue
                mw = RE_PWM.search(line)
                pwm = max((num(mw.group(i)) for i in range(1, 5)), default=float("nan")) if mw else float("nan")
                rows.append((num(mt.group(1)), x * 100, y * 100, pwm))
        segs, cur = [], []
        for r in rows:
            if r[3] == r[3] and r[3] > 50:
                if cur and r[0] - cur[-1][0] > 0.5:
                    segs.append(cur)
                    cur = []
                cur.append(r)
            elif cur:
                segs.append(cur)
                cur = []
        if cur:
            segs.append(cur)
        good = [s for s in segs if len(s) > 40 and s[-1][0] - s[0][0] > 3]
        if not good:
            continue
        print(f"\n{f}")
        for s in good[:6]:
            span = s[-1][0] - s[0][0]
            xs = [r[1] for r in s]
            ys = [r[2] for r in s]
            net = math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])
            print(f"  电机转 t={s[0][0]:.0f}~{s[-1][0]:.0f}s ({span:.1f}s) 位姿峰峰 "
                  f"x {max(xs) - min(xs):6.1f} y {max(ys) - min(ys):6.1f} cm  净漂 {net:6.1f} cm")


if __name__ == "__main__":
    main()

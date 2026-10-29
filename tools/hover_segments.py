#!/usr/bin/env python3
"""Find hover segments (throttle high, motors running) and measure pose drift inside them."""
import argparse
import math
import re

RE_T = re.compile(r"t=\s*(-?[\d.]+)")
RE_XY = re.compile(r"x=\s*(-?[\d.]+|nan)\s+y=\s*(-?[\d.]+|nan)")
RE_THR = re.compile(r"fcThr=\s*(-?[\d.]+|nan)")
RE_FC = re.compile(r"fcVelX=\s*(-?[\d.]+|nan)\s+fcVelY=\s*(-?[\d.]+|nan)")


def num(s):
    try:
        return float(s)
    except ValueError:
        return float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--thr", type=float, default=480.0)
    ap.add_argument("--min_s", type=float, default=5.0)
    args = ap.parse_args()

    rows = []
    with open(args.log, "r", encoding="utf-8", errors="ignore") as fh:
        for line in fh:
            mt, mp = RE_T.search(line), RE_XY.search(line)
            if not (mt and mp):
                continue
            x, y = num(mp.group(1)), num(mp.group(2))
            if x != x or y != y:
                continue
            mthr, mf = RE_THR.search(line), RE_FC.search(line)
            rows.append((num(mt.group(1)), x * 100, y * 100,
                         num(mthr.group(1)) if mthr else float("nan"),
                         math.hypot(num(mf.group(1)), num(mf.group(2))) if mf else float("nan")))
    segs, cur = [], []
    for r in rows:
        if r[3] == r[3] and r[3] > args.thr:
            if cur and r[0] - cur[-1][0] > 0.3:
                segs.append(cur)
                cur = []
            cur.append(r)
        elif cur:
            segs.append(cur)
            cur = []
    if cur:
        segs.append(cur)

    print(f"{args.log}")
    for s in segs:
        span = s[-1][0] - s[0][0]
        if span < args.min_s:
            continue
        xs = [r[1] for r in s]
        ys = [r[2] for r in s]
        net = math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])
        print(f"  段 t={s[0][0]:.0f}~{s[-1][0]:.0f}s  {span:5.1f}s  样本 {len(s):4d}"
              f"  峰峰 x {max(xs) - min(xs):5.0f} y {max(ys) - min(ys):5.0f} cm"
              f"  起 ({xs[0]:.0f},{ys[0]:.0f}) 终 ({xs[-1]:.0f},{ys[-1]:.0f})"
              f"  净漂 {net:5.0f} cm → {net / span:4.1f} cm/s")


if __name__ == "__main__":
    main()

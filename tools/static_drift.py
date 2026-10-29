#!/usr/bin/env python3
"""Split map-pose motion by measured airframe motion (IMU), to answer:
while the airframe is really still, how fast does the map pose creep?"""
import argparse
import math
import re

RE_T = re.compile(r"t=\s*(-?[\d.]+)")
RE_XY = re.compile(r"x=\s*(-?[\d.]+|nan)\s+y=\s*(-?[\d.]+|nan)")
RE_G = re.compile(r"fcGyroXY=\(\s*(-?[\d.]+|nan),\s*(-?[\d.]+|nan)\)")
RE_A = re.compile(r"fcAccX=\s*(-?[\d.]+|nan)\s+fcAccY=\s*(-?[\d.]+|nan)")
RE_PWM = re.compile(r"fcPwm=\(\s*(-?[\d.]+|nan),\s*(-?[\d.]+|nan),\s*(-?[\d.]+|nan),\s*(-?[\d.]+|nan)\)")


def num(s):
    try:
        return float(s)
    except ValueError:
        return float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
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
            mg, ma = RE_G.search(line), RE_A.search(line)
            g = math.hypot(num(mg.group(1)), num(mg.group(2))) if mg else float("nan")
            a = math.hypot(num(ma.group(1)), num(ma.group(2))) if ma else float("nan")
            rows.append({"t": num(mt.group(1)), "x": x * 100, "y": y * 100, "g": g, "a": a})

    # 逐样本地图速度（居中差分 0.4 s）
    for i, r in enumerate(rows):
        a, b = max(0, i - 2), min(len(rows) - 1, i + 2)
        dt = rows[b]["t"] - rows[a]["t"]
        r["v"] = math.hypot(rows[b]["x"] - rows[a]["x"], rows[b]["y"] - rows[a]["y"]) / dt if dt > 0.05 else float("nan")

    print(f"{args.log}: 样本 {len(rows)}")
    gs = sorted(r["g"] for r in rows if r["g"] == r["g"])
    as_ = sorted(r["a"] for r in rows if r["a"] == r["a"])
    print(f"陀螺幅值 中位 {gs[len(gs)//2]:.0f} p90 {gs[int(len(gs)*0.9)]:.0f} 最大 {gs[-1]:.0f}")
    print(f"加计(水平) 中位 {as_[len(as_)//2]:.0f} p90 {as_[int(len(as_)*0.9)]:.0f} 最大 {as_[-1]:.0f}")

    buckets = [(0, 5), (5, 15), (15, 40), (40, 100), (100, 10000)]
    print("\n陀螺幅值区间   样本数   地图速度中位   p90    最大")
    for lo, hi in buckets:
        sel = [r["v"] for r in rows if r["g"] == r["g"] and lo <= r["g"] < hi and r["v"] == r["v"]]
        if len(sel) < 20:
            continue
        sel.sort()
        print(f"  {lo:5d}-{hi:<6d} {len(sel):7d}   {sel[len(sel)//2]:9.1f}  "
              f"{sel[int(len(sel)*0.9)]:6.1f}  {sel[-1]:7.1f}")

    still = [r for r in rows if r["g"] == r["g"] and r["g"] < 5 and r["v"] == r["v"]]
    if still:
        vs = sorted(r["v"] for r in still)
        print(f"\n陀螺<5 的静止段: {len(still)} 样本，地图速度 中位 {vs[len(vs)//2]:.1f} "
              f"p90 {vs[int(len(vs)*0.9)]:.1f} cm/s")
        # 这些静止窗口里，地图累计走了多远
        segs = []
        cur = []
        for r in still:
            if cur and r["t"] - cur[-1]["t"] > 0.3:
                segs.append(cur)
                cur = []
            cur.append(r)
        if cur:
            segs.append(cur)
        long_segs = [s for s in segs if s[-1]["t"] - s[0]["t"] > 10]
        print(f"  连续静止>10s 的段 {len(long_segs)} 个：")
        for s in long_segs[:8]:
            span = s[-1]["t"] - s[0]["t"]
            net = math.hypot(s[-1]["x"] - s[0]["x"], s[-1]["y"] - s[0]["y"])
            print(f"    t={s[0]['t']:7.1f}s  {span:5.1f}s  净位移 {net:7.1f} cm  "
                  f"≈{net / span:6.1f} cm/s")


if __name__ == "__main__":
    main()

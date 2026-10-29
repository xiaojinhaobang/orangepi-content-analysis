#!/usr/bin/env python3
"""Find tilt episodes in rec_*.log: how fast roll/pitch grew and at what throttle.

Used to compare a takeoff that stayed level with one that tipped over.
"""
import argparse
import math
import re

RE_T = re.compile(r"t=\s*(-?[\d.]+)")
RE_TILT = re.compile(r"fcRoll=\s*(-?[\d.]+|nan)\s+fcPitch=\s*(-?[\d.]+|nan)")
RE_THR = re.compile(r"fcThr=\s*(-?[\d.]+|nan)")


def num(s):
    try:
        return float(s)
    except ValueError:
        return float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--limit", type=float, default=5.0, help="只报倾角超过这个值的区段（度）")
    args = ap.parse_args()

    rows = []
    with open(args.log, "r", encoding="utf-8", errors="ignore") as fh:
        for line in fh:
            mt, mi = RE_T.search(line), RE_TILT.search(line)
            if not (mt and mi):
                continue
            roll, pitch = num(mi.group(1)), num(mi.group(2))
            if roll != roll or pitch != pitch:
                continue
            mthr = RE_THR.search(line)
            rows.append({"t": num(mt.group(1)), "roll": roll, "pitch": pitch,
                         "tilt": math.hypot(roll, pitch),
                         "thr": num(mthr.group(1)) if mthr else float("nan")})
    if not rows:
        print(f"{args.log}: 没有倾角样本")
        return

    episodes = []
    cur = []
    for r in rows:
        if r["tilt"] > args.limit:
            if cur and r["t"] - cur[-1]["t"] > 0.3:
                episodes.append(cur)
                cur = []
            cur.append(r)
        elif cur:
            episodes.append(cur)
            cur = []
    if cur:
        episodes.append(cur)

    print(f"\n=== {args.log}")
    print(f"样本 {len(rows)}  时长 {rows[-1]['t'] - rows[0]['t']:.0f}s  倾角>={args.limit}° 的区段 {len(episodes)} 个")
    for i, seg in enumerate(episodes):
        peak = max(seg, key=lambda r: r["tilt"])
        t0 = seg[0]["t"]
        # 峰值前 0.5 s 的倾角增长率
        before = [r for r in rows if peak["t"] - 0.5 <= r["t"] <= peak["t"]]
        rate = (peak["tilt"] - before[0]["tilt"]) / max(0.05, peak["t"] - before[0]["t"]) if before else float("nan")
        print(f"  #{i} t={t0:8.1f}s 持续{seg[-1]['t'] - t0:5.2f}s 峰值{peak['tilt']:5.1f}° "
              f"(roll{peak['roll']:6.1f} pitch{peak['pitch']:6.1f}) "
              f"峰值前增长率{rate:6.1f}°/s thr={peak['thr']:.0f}")


if __name__ == "__main__":
    main()

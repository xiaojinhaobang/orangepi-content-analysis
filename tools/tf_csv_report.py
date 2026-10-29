#!/usr/bin/env python3
"""Statistics for tf_velocity_compare CSVs (map pose vs commanded/measured velocity)."""
import argparse
import csv
import math


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--tail", type=int, default=0)
    args = ap.parse_args()

    rows = []
    with open(args.csv, newline="", encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            try:
                rows.append({k: float(v) for k, v in r.items()})
            except (TypeError, ValueError):
                continue
    if len(rows) < 20:
        print("样本太少")
        return

    for r in rows:
        r["speed"] = math.hypot(r["vel_x"], r["vel_y"])

    ts = [r["t"] for r in rows]
    xs = [r["map_x"] for r in rows]
    ys = [r["map_y"] for r in rows]
    dt = (ts[-1] - ts[0]) / (len(ts) - 1)
    print(f"{args.csv}: {len(rows)} 样本  {ts[-1]:.1f}s  采样 {1 / dt:.1f} Hz")
    print(f"map 坐标 x {min(xs):.1f}~{max(xs):.1f} cm  y {min(ys):.1f}~{max(ys):.1f} cm")
    print(f"位姿峰峰 x {max(xs) - min(xs):.1f} cm  y {max(ys) - min(ys):.1f} cm  "
          f"净位移 {math.hypot(xs[-1] - xs[0], ys[-1] - ys[0]):.1f} cm")

    vs, fvs = [], []
    for i in range(2, len(rows) - 2):
        a, b = rows[i - 2], rows[i + 2]
        span = b["t"] - a["t"]
        if span > 0.05:
            vs.append(math.hypot(b["map_x"] - a["map_x"], b["map_y"] - a["map_y"]) / span)
        fvs.append(rows[i]["speed"])
    vs.sort()
    fvs.sort()
    print(f"地图位姿速度 中位 {vs[len(vs)//2]:6.1f}  p90 {vs[int(len(vs)*0.9)]:7.1f}  最大 {vs[-1]:8.1f} cm/s")
    print(f"/velocity_map 中位 {fvs[len(fvs)//2]:6.2f}  p90 {fvs[int(len(fvs)*0.9)]:7.2f}  最大 {fvs[-1]:8.2f} cm/s")

    if args.tail:
        print("\n   t      map_x    map_y   vel_x  vel_y")
        for r in rows[-args.tail:]:
            print(f" {r['t']:6.2f}  {r['map_x']:8.1f} {r['map_y']:8.1f}  {r['vel_x']:6.2f} {r['vel_y']:6.2f}")


if __name__ == "__main__":
    main()

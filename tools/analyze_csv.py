#!/usr/bin/env python3
"""Analyze flight_trace.py CSV logs (1 Hz: pose + slam velocity + FC status)."""

import csv
import math
import sys


def load(path):
    rows = []
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            rows.append({
                "t": float(r["t"]),
                "x": float(r["x_m"]) * 100.0,
                "y": float(r["y_m"]) * 100.0,
                "yaw": float(r["yaw_deg"]),
                "vx": float(r["slam_vx_cms"]),
                "vy": float(r["slam_vy_cms"]),
                "tx": int(r["tacc_x"]),
                "ty": int(r["tacc_y"]),
                "mode": int(r["mode"]) if r["mode"] != "None" else -1,
                "unlock": int(r["unlock"]) if r["unlock"] != "None" else -1,
                "len": r["link_len"],
            })
    return rows


def main(path):
    rows = load(path)
    print(f"=== {path}  {len(rows)} rows, t={rows[0]['t']}..{rows[-1]['t']} s ===")
    print(f"pose x: {min(r['x'] for r in rows):.1f}..{max(r['x'] for r in rows):.1f} cm")
    print(f"pose y: {min(r['y'] for r in rows):.1f}..{max(r['y'] for r in rows):.1f} cm")

    print("\n-- state changes (mode / unlock) --")
    prev = None
    for r in rows:
        cur = (r["mode"], r["unlock"])
        if cur != prev:
            print(f"  t={r['t']:6.1f}  mode={r['mode']} unlock={r['unlock']}  "
                  f"x={r['x']:8.1f} y={r['y']:8.1f} yaw={r['yaw']:+6.1f} LEN={r['len']}")
            prev = cur

    print("\n-- unlocked windows: pose displacement --")
    i = 0
    while i < len(rows):
        if rows[i]["unlock"] == 1:
            j = i
            while j + 1 < len(rows) and rows[j + 1]["unlock"] == 1:
                j += 1
            a, b = rows[i], rows[j]
            dx, dy = b["x"] - a["x"], b["y"] - a["y"]
            print(f"  t={a['t']:6.1f}..{b['t']:6.1f} ({b['t']-a['t']:5.1f}s) "
                  f"dx={dx:+7.1f} dy={dy:+7.1f} |d|={math.hypot(dx,dy):6.1f} cm")
            i = j + 1
        else:
            i += 1

    print("\n-- timeline (10 s) --")
    last = None
    for r in rows:
        slot = int(r["t"] // 10)
        if slot == last:
            continue
        last = slot
        print(f"t={r['t']:6.1f} x={r['x']:8.1f} y={r['y']:8.1f} yaw={r['yaw']:+6.1f} "
              f"v=({r['vx']:+6.1f},{r['vy']:+6.1f}) tacc=({r['tx']:+4d},{r['ty']:+4d}) "
              f"mode={r['mode']} unlock={r['unlock']}")


if __name__ == "__main__":
    for p in sys.argv[1:]:
        main(p)

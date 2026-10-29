#!/usr/bin/env python3
"""Analyze hover/flight trace logs produced by hover_test.py on the Orange Pi.

Log line format:
  t=  0.2 x=1362.6 y=904.6 yaw=5.6 drift=0.0 dyaw=0.0 | raw=(0.0, 0.0)
  filt=(0.0, 0.0) tacc=(0, 0) tv=[0.0, 0.0, -2.7, 0.0] | h=3 mode=1 unlock=0
"""

import math
import re
import sys

def _num(line, key, group=1):
    m = re.search(key + r"=\s*(-?[\d.]+)", line)
    return float(m.group(group)) if m else None


def _pair(line, key):
    m = re.search(key + r"=\(\s*(-?[\d.]+),\s*(-?[\d.]+)\)", line)
    return (float(m.group(1)), float(m.group(2))) if m else (None, None)


def parse(path):
    rows = []
    with open(path, errors="ignore") as fh:
        for lineno, line in enumerate(fh, 1):
            if "t=" not in line or "mode=" not in line:
                continue
            fx, fy = _pair(line, "filt")
            rx, ry = _pair(line, "raw")
            tx, ty = _pair(line, "tacc")
            m = re.search(r"tv=(\[[^\]]*\]|None)", line)
            rec = {
                "t": _num(line, "t"), "x": _num(line, "x"), "y": _num(line, "y"),
                "yaw": _num(line, "yaw"), "drift": _num(line, "drift"),
                "dyaw": _num(line, "dyaw"), "fx": fx, "fy": fy, "rx": rx, "ry": ry,
                "tx": tx, "ty": ty, "tv": m.group(1) if m else None,
                "h": _num(line, "h"), "mode": _num(line, "mode"),
                "unlock": _num(line, "unlock"),
            }
            if None in (rec["t"], rec["x"], rec["y"], rec["h"]):
                continue
            rec["h"] = int(rec["h"])
            rec["mode"] = int(rec["mode"] or -1)
            rec["unlock"] = int(rec["unlock"] or -1)
            rec["line"] = lineno
            rows.append(rec)
    return rows


def tv_xy(tv):
    if not tv or tv == "None":
        return None
    nums = re.findall(r"-?[\d.]+e?-?\d*", tv)
    return (float(nums[0]), float(nums[1])) if len(nums) >= 2 else None


def mean(vals):
    vals = [v for v in vals if v is not None]
    return sum(vals) / len(vals) if vals else float("nan")


def rms(vals):
    vals = [v for v in vals if v is not None]
    return math.sqrt(sum(v * v for v in vals) / len(vals)) if vals else float("nan")


def report(path):
    rows = parse(path)
    print(f"\n=== {path}  ({len(rows)} samples) ===")
    if not rows:
        print("no parsable lines")
        return

    # full timeline: h / unlock / mode, sampled every ~2 s
    print("\n-- full timeline (2 s) --")
    last = None
    for r in rows:
        slot = int(r["t"] // 2)
        if slot == last:
            continue
        last = slot
        air = "#" * min(40, max(0, r["h"] // 5))
        print(f"t={r['t']:7.1f} h={r['h']:4d} mode={r['mode']} unlock={r['unlock']} "
              f"x={r['x']:8.1f} y={r['y']:8.1f} drift={r['drift']:7.1f} |{air}")

    air = [r for r in rows if r["h"] > 15]
    print(f"time span      : t={rows[0]['t']:.1f} .. {rows[-1]['t']:.1f} s")
    print(f"height         : min={min(r['h'] for r in rows)}  max={max(r['h'] for r in rows)} cm")
    print(f"unlocked frac  : {mean([r['unlock'] for r in rows]):.2f}   mode set={sorted({r['mode'] for r in rows})}")
    if not air:
        print("NEVER AIRBORNE (h never > 15 cm) -> ground-only log")
        return

    t0, t1 = air[0]["t"], air[-1]["t"]
    print(f"airborne       : t={t0:.1f} .. {t1:.1f} s  ({t1-t0:.1f} s)")
    # reference = pose at the last sample before takeoff (or first airborne sample)
    ref = air[0]
    print(f"takeoff ref    : x={ref['x']:.1f} y={ref['y']:.1f} h={ref['h']} unlock={ref['unlock']}")
    print(f"end of air     : x={air[-1]['x']:.1f} y={air[-1]['y']:.1f} h={air[-1]['h']} unlock={air[-1]['unlock']}")

    dx = air[-1]["x"] - ref["x"]
    dy = air[-1]["y"] - ref["y"]
    print(f"NET DISPLACEMENT (dead-reckon) : dx={dx:.1f} dy={dy:.1f} dist={math.hypot(dx,dy):.1f} cm")

    print("\n  t     h  unlock | dead-reckon x,y   drift | radar raw x,y      filt x,y     | tacc x,y   | cmd tv x,y")
    last_slot = None
    for r in air:
        slot = int(r["t"])
        if slot == last_slot:
            continue
        last_slot = slot
        tv = tv_xy(r["tv"])
        tvs = "None" if tv is None else f"{tv[0]:+6.2f},{tv[1]:+6.2f}"
        print(f"{r['t']:6.1f} {r['h']:4d} {r['unlock']:6d} | {r['x']:8.1f},{r['y']:8.1f} "
              f"{r['drift']:7.1f} | {r['rx']:+7.2f},{r['ry']:+7.2f} {r['fx']:+7.2f},{r['fy']:+7.2f} | "
              f"{r['tx']:+5d},{r['ty']:+5d} | {tvs}")

    print("\n-- airborne statistics --")
    for name, key in (("radar raw", ("rx", "ry")), ("filtered/injected", ("fx", "fy")), ("tacc echo", ("tx", "ty"))):
        xs = [r[key[0]] for r in air if r[key[0]] is not None]
        ys = [r[key[1]] for r in air if r[key[1]] is not None]
        if not xs or not ys:
            continue
        print(f"{name:18s} mean=({mean(xs):+6.2f},{mean(ys):+6.2f}) rms=({rms(xs):6.2f},{rms(ys):6.2f}) "
              f"peak=({max(xs,key=abs):+7.1f},{max(ys,key=abs):+7.1f})")

    tvs = [tv_xy(r["tv"]) for r in air]
    tvx = [v[0] for v in tvs if v]
    tvy = [v[1] for v in tvs if v]
    print(f"{'pos-loop cmd tv':18s} mean=({mean(tvx):+6.2f},{mean(tvy):+6.2f}) rms=({rms(tvx):6.2f},{rms(tvy):6.2f}) "
          f"peak=({max(tvx,key=abs) if tvx else float('nan'):+7.1f},{max(tvy,key=abs) if tvy else float('nan'):+7.1f}) "
          f"coverage={len(tvx)}/{len(air)}")

    # drift rate per second, windowed
    print("\n-- drift rate (dead-reckon) --")
    step = max(1, len(air) // 10)
    for i in range(0, len(air) - step, step):
        a, b = air[i], air[i + step]
        dt = b["t"] - a["t"]
        if dt <= 0:
            continue
        vx = (b["x"] - a["x"]) / dt
        vy = (b["y"] - a["y"]) / dt
        print(f"  t={a['t']:6.1f}..{b['t']:6.1f}  v=({vx:+6.2f},{vy:+6.2f}) cm/s  |v|={math.hypot(vx,vy):5.2f}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        raise SystemExit(2)
    for p in sys.argv[1:]:
        report(p)

#!/usr/bin/env python3
"""Summarise one rec_*.log flight: where the map pose went, how fast, how high."""
import argparse
import math
import re

RE_T = re.compile(r"t=\s*(-?[\d.]+)")
RE_XY = re.compile(r"x=\s*(-?[\d.]+|nan)\s+y=\s*(-?[\d.]+|nan)")
RE_Z = re.compile(r"z=\s*(-?[\d.]+|nan)")
RE_FC = re.compile(r"fcVelX=\s*(-?[\d.]+|nan)\s+fcVelY=\s*(-?[\d.]+|nan)")
RE_THR = re.compile(r"fcThr=\s*(-?[\d.]+|nan)")
RE_PWM = re.compile(r"fcPwm=\(\s*(-?[\d.]+|nan),\s*(-?[\d.]+|nan),\s*(-?[\d.]+|nan),\s*(-?[\d.]+|nan)\)")
RE_TGT = re.compile(r"fcTgtVx=\s*(-?[\d.]+|nan)\s+fcTgtVy=\s*(-?[\d.]+|nan)")


def num(s):
    try:
        return float(s)
    except ValueError:
        return float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--window", type=float, default=0.0, help="只看最后 N 秒")
    args = ap.parse_args()

    rows = []
    with open(args.log, "r", encoding="utf-8", errors="ignore") as fh:
        for line in fh:
            mt = RE_T.search(line)
            if not mt:
                continue
            r = {"t": num(mt.group(1))}
            m = RE_XY.search(line)
            r["x"], r["y"] = (num(m.group(1)) * 100, num(m.group(2)) * 100) if m else (float("nan"),) * 2
            m = RE_Z.search(line)
            r["z"] = num(m.group(1)) * 100 if m else float("nan")
            m = RE_FC.search(line)
            r["vx"], r["vy"] = (num(m.group(1)), num(m.group(2))) if m else (float("nan"),) * 2
            m = RE_THR.search(line)
            r["thr"] = num(m.group(1)) if m else float("nan")
            m = RE_PWM.search(line)
            r["pwm"] = max(num(m.group(i)) for i in range(1, 5)) if m else float("nan")
            m = RE_TGT.search(line)
            r["tvx"], r["tvy"] = (num(m.group(1)), num(m.group(2))) if m else (float("nan"),) * 2
            rows.append(r)

    if args.window > 0:
        t_end = rows[-1]["t"]
        rows = [r for r in rows if r["t"] >= t_end - args.window]
    if not rows:
        print("没有样本")
        return

    xs = [r["x"] for r in rows if r["x"] == r["x"]]
    ys = [r["y"] for r in rows if r["y"] == r["y"]]
    zs = [r["z"] for r in rows if r["z"] == r["z"]]
    vs = [math.hypot(r["vx"], r["vy"]) for r in rows if r["vx"] == r["vx"] and r["vy"] == r["vy"]]
    t0, t1 = rows[0]["t"], rows[-1]["t"]
    print(f"{args.log}: {len(rows)} 样本  {t0:.0f}~{t1:.0f}s（{t1 - t0:.0f}s）")
    if xs:
        print(f"地图位姿 x {min(xs):7.0f}~{max(xs):7.0f} cm   y {min(ys):7.0f}~{max(ys):7.0f} cm")
        far = max(math.hypot(x, y) for x, y in zip(xs, ys))
        print(f"离原点最远 {far:.0f} cm")
    if zs:
        zs.sort()
        print(f"高度 中位 {zs[len(zs)//2]:.0f} cm  最高 {zs[-1]:.0f} cm")
    if vs:
        vs.sort()
        print(f"飞控速度 中位 {vs[len(vs)//2]:.0f}  p90 {vs[int(len(vs)*0.9)]:.0f}  最大 {vs[-1]:.0f} cm/s")

    # 按飞控自报速度积分出“实际飞了多远”（与地图位姿无关）
    dist = 0.0
    dist_far = 0.0
    for a, b in zip(rows, rows[1:]):
        if a["vx"] != a["vx"] or a["vy"] != a["vy"]:
            continue
        dt = b["t"] - a["t"]
        if not (0 < dt < 0.3):
            continue
        dist += math.hypot(a["vx"], a["vy"]) * dt
        if math.hypot(a["vx"], a["vy"]) > 5.0:
            dist_far += math.hypot(a["vx"], a["vy"]) * dt
    print(f"飞控速度积分路程：全程 {dist / 100:.1f} m，其中 |v|>5cm/s 的部分 {dist_far / 100:.1f} m")

    tv = [math.hypot(r["tvx"], r["tvy"]) for r in rows if r["tvx"] == r["tvx"] and r["tvy"] == r["tvy"]]
    if tv:
        tv.sort()
        print(f"我们下发的水平指令速度：中位 {tv[len(tv)//2]:.1f}  p90 {tv[int(len(tv)*0.9)]:.1f}  最大 {tv[-1]:.1f} cm/s")

    print("\n   t      x      y     z     thr   fcVel  8s窗口位移")
    step = max(1, int(len(rows) / 40))
    for i in range(0, len(rows) - step, step):
        r = rows[i]
        nxt = rows[min(len(rows) - 1, i + step * 8)]
        d = math.hypot(nxt["x"] - r["x"], nxt["y"] - r["y"]) if r["x"] == r["x"] and nxt["x"] == nxt["x"] else float("nan")
        print(f" {r['t'] - t0:6.1f} {r['x']:7.0f} {r['y']:7.0f} {r['z']:5.0f} {r['thr']:6.0f} "
              f"{math.hypot(r['vx'], r['vy']) if r['vx'] == r['vx'] else float('nan'):6.0f} {d:8.0f}")


if __name__ == "__main__":
    main()

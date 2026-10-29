#!/usr/bin/env python3
"""Characterise how (and when) the map->laser pose drifts in rec_*.log traces.

Read-only analysis. Answers: is the drift discrete spikes or slow creep, does it
keep one direction, when does it start, and does it match the FC velocity.
"""
import argparse
import math
import re
from statistics import median

RE_T = re.compile(r"t=\s*(-?[\d.]+)")
RE_XY = re.compile(r"x=\s*(-?[\d.]+|nan)\s+y=\s*(-?[\d.]+|nan)\s+psi=\s*(-?[\d.]+|nan)")
RE_FILT = re.compile(r"filt=\(\s*(-?[\d.]+|nan),\s*(-?[\d.]+|nan)\)")
RE_RAW = re.compile(r"raw=\(\s*(-?[\d.]+|nan),\s*(-?[\d.]+|nan)\)")
RE_Z = re.compile(r"z=\s*(-?[\d.]+|nan)")
RE_FCVEL = re.compile(r"fcVelX=\s*(-?[\d.]+|nan)\s+fcVelY=\s*(-?[\d.]+|nan)")
RE_PWM = re.compile(r"fcPwm=\(\s*(-?[\d.]+|nan),\s*(-?[\d.]+|nan),\s*(-?[\d.]+|nan),\s*(-?[\d.]+|nan)\)")


def num(text):
    try:
        v = float(text)
    except ValueError:
        return float("nan")
    return v


def parse(path):
    rows = []
    with open(path, "r", encoding="utf-8", errors="ignore") as handle:
        for line in handle:
            m_t = RE_T.search(line)
            if not m_t:
                continue
            row = {"t": num(m_t.group(1))}
            m = RE_XY.search(line)
            row["x"] = num(m.group(1)) * 100.0 if m else float("nan")
            row["y"] = num(m.group(2)) * 100.0 if m else float("nan")
            m = RE_FILT.search(line)
            row["fvx"], row["fvy"] = (num(m.group(1)), num(m.group(2))) if m else (float("nan"),) * 2
            m = RE_RAW.search(line)
            row["rvx"], row["rvy"] = (num(m.group(1)), num(m.group(2))) if m else (float("nan"),) * 2
            m = RE_Z.search(line)
            row["z"] = num(m.group(1)) * 100.0 if m else float("nan")
            m = RE_FCVEL.search(line)
            row["cvx"], row["cvy"] = (num(m.group(1)), num(m.group(2))) if m else (float("nan"),) * 2
            m = RE_PWM.search(line)
            if m:
                vals = [num(m.group(i)) for i in range(1, 5)]
                row["pwm"] = max(v for v in vals if v == v) if any(v == v for v in vals) else float("nan")
            else:
                row["pwm"] = float("nan")
            rows.append(row)
    return rows


def pct(values, q):
    if not values:
        return float("nan")
    s = sorted(values)
    k = max(0, min(len(s) - 1, int(round((len(s) - 1) * q))))
    return s[k]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    args = ap.parse_args()

    for path in args.logs:
        rows = parse(path)
        good = [r for r in rows if r["x"] == r["x"] and r["y"] == r["y"]]
        if len(good) < 50:
            print(f"\n=== {path}: 有效位姿样本 {len(good)}，跳过")
            continue

        # 0.2 s centred difference -> cm/s
        t0 = good[0]["t"]
        speeds = []
        for i, r in enumerate(good):
            a = max(0, i - 2)
            b = min(len(good) - 1, i + 2)
            dt = good[b]["t"] - good[a]["t"]
            if dt <= 0.02:
                continue
            vx = (good[b]["x"] - good[a]["x"]) / dt
            vy = (good[b]["y"] - good[a]["y"]) / dt
            r["mvx"], r["mvy"] = vx, vy
            r["mspeed"] = math.hypot(vx, vy)
            speeds.append(r["mspeed"])

        # consecutive-sample step (native recorder grid, 20 Hz)
        steps = []
        for i in range(1, len(good)):
            dt = good[i]["t"] - good[i - 1]["t"]
            if 0.01 < dt < 0.12:
                steps.append(math.hypot(good[i]["x"] - good[i - 1]["x"],
                                        good[i]["y"] - good[i - 1]["y"]))
        # quantisation: how much of a step is zero (pose repeated between scans)
        zero_frac = sum(1 for s in steps if s < 0.02) / max(1, len(steps))

        print(f"\n=== {path}")
        print(f"时长 {good[-1]['t'] - t0:.1f}s   样本 {len(good)}   平均 {len(good) / max(0.1, good[-1]['t'] - t0):.1f} Hz")
        print(f"地图速度 cm/s: 中位 {median(speeds):.1f}  p90 {pct(speeds, 0.9):.1f}  p99 {pct(speeds, 0.99):.1f}  最大 {max(speeds):.1f}")
        print(f"相邻帧位移 cm: 中位 {median(steps):.2f}  p90 {pct(steps, 0.9):.2f}  p99 {pct(steps, 0.99):.2f}  最大 {max(steps):.2f}")
        print(f"位移<0.2mm 的帧占比 {zero_frac * 100:.0f}%  （位姿被重复读取，说明量化在帧间隔上）")

        # 找“地图说自己在动”的连续区段
        moving = [r for r in good if r.get("mspeed", 0) > 8.0]
        if moving:
            print(f"地图速度>8cm/s 的帧 {len(moving)}/{len(good)} = {100 * len(moving) / len(good):.0f}%")

        # 方向一致性：把速度归一化后求平均向量长度（1=同一方向，0=乱走）
        units = []
        for r in good:
            s = r.get("mspeed", 0)
            if s > 8.0:
                units.append((r["mvx"] / s, r["mvy"] / s))
        if units:
            mx = sum(u[0] for u in units) / len(units)
            my = sum(u[1] for u in units) / len(units)
            print(f"漂移方向集中度 {math.hypot(mx, my):.2f}  (平均方向 {math.degrees(math.atan2(my, mx)):.0f}°)")

        # 累积路径 vs 净位移
        path = sum(math.hypot(good[i]["x"] - good[i - 1]["x"], good[i]["y"] - good[i - 1]["y"])
                   for i in range(1, len(good)))
        net = math.hypot(good[-1]["x"] - good[0]["x"], good[-1]["y"] - good[0]["y"])
        print(f"路径长 {path:.0f}cm  净位移 {net:.0f}cm  比值 {path / max(net, 0.1):.2f}（≈1 为单向连续漂）")

        # 每 5 秒一格，看漂移随时间/高度怎么长
        print("\n  t窗     地图速度中位  飞控速度中位  z中位   PWM中位  方向集中度")
        bucket = 5.0
        start = good[0]["t"]
        end = good[-1]["t"]
        k = start
        while k < end:
            seg = [r for r in good if k <= r["t"] < k + bucket and "mspeed" in r]
            if len(seg) > 10:
                ms = median([r["mspeed"] for r in seg])
                cs = median([math.hypot(r["cvx"], r["cvy"]) for r in seg
                             if r["cvx"] == r["cvx"] and r["cvy"] == r["cvy"]] or [float("nan")])
                zs = median([r["z"] for r in seg if r["z"] == r["z"]] or [float("nan")])
                ps = median([r["pwm"] for r in seg if r["pwm"] == r["pwm"]] or [float("nan")])
                us = [(r["mvx"] / r["mspeed"], r["mvy"] / r["mspeed"]) for r in seg if r["mspeed"] > 8.0]
                conc = float("nan")
                if us:
                    conc = math.hypot(sum(u[0] for u in us) / len(us), sum(u[1] for u in us) / len(us))
                print(f"  {k - start:5.0f}s  {ms:11.1f}  {cs:12.1f}  {zs:6.1f}  {ps:8.0f}  {conc:9.2f}")
            k += bucket

        # 尖峰 vs 蠕动：单个 0.2s 窗口里超过 5cm 的次数，以及这些"跳"之间的间隔
        jumps = [r["t"] - t0 for r in good
                 if r.get("mspeed", 0) * 0.2 > 5.0]
        if len(jumps) > 2:
            gaps = [jumps[i] - jumps[i - 1] for i in range(1, len(jumps))]
            gaps = [g for g in gaps if g > 0.05]
            print(f"\n0.2s 内位移>5cm 的时刻 {len(jumps)} 次；相邻间隔中位 "
                  f"{median(gaps) if gaps else float('nan'):.2f}s，"
                  f"间隔分布 p10 {pct(gaps, 0.1) if gaps else float('nan'):.2f}s / "
                  f"p90 {pct(gaps, 0.9) if gaps else float('nan'):.2f}s")


if __name__ == "__main__":
    main()

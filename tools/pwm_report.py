#!/usr/bin/env python3
"""Per-channel PWM statistics of our own rec_*.log, in the same shape the other
team reports (mean / ripple per channel during a hover segment)."""
import argparse
import re

RE_T = re.compile(r"t=\s*(-?[\d.]+)")
RE_PWM = re.compile(r"fcPwm=\(\s*(-?[\d.]+|nan),\s*(-?[\d.]+|nan),\s*(-?[\d.]+|nan),\s*(-?[\d.]+|nan)\)")
RE_ROLL = re.compile(r"fcRoll=\s*(-?[\d.]+|nan)\s+fcPitch=\s*(-?[\d.]+|nan)")
RE_FCVEL = re.compile(r"fcVelX=\s*(-?[\d.]+|nan)\s+fcVelY=\s*(-?[\d.]+|nan)")


def num(s):
    try:
        return float(s)
    except ValueError:
        return float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--thr", type=float, default=300.0, help="PWM 均值高于此值算电机在转")
    args = ap.parse_args()

    rows = []
    with open(args.log, "r", encoding="utf-8", errors="ignore") as fh:
        for line in fh:
            mt, mp = RE_T.search(line), RE_PWM.search(line)
            if not (mt and mp):
                continue
            pwm = [num(mp.group(i)) for i in range(1, 5)]
            if any(v != v for v in pwm) or max(pwm) <= 0:
                continue
            mr = RE_ROLL.search(line)
            mv = RE_FCVEL.search(line)
            rows.append({
                "t": num(mt.group(1)),
                "pwm": pwm,
                "roll": num(mr.group(1)) if mr else float("nan"),
                "pitch": num(mr.group(2)) if mr else float("nan"),
                "vx": num(mv.group(1)) if mv else float("nan"),
                "vy": num(mv.group(2)) if mv else float("nan"),
            })
    if not rows:
        print("没有带 fcPwm 的样本")
        return

    # 连续区段：相邻样本间隔 <0.5s 且均值 > thr
    segs = []
    cur = []
    for r in rows:
        mean = sum(r["pwm"]) / 4.0
        if mean > args.thr:
            if cur and r["t"] - cur[-1]["t"] > 0.5:
                segs.append(cur)
                cur = []
            cur.append(r)
        elif cur:
            segs.append(cur)
            cur = []
    if cur:
        segs.append(cur)

    print(f"{args.log}: 电机转动区段 {len(segs)} 个")
    for i, seg in enumerate(segs):
        if len(seg) < 20:
            continue
        print(f"\n区段{i}: {seg[0]['t']:.1f}s ~ {seg[-1]['t']:.1f}s  ({len(seg)} 样本)")
        means = []
        print("  通道   均值    波动范围       幅度")
        for c in range(4):
            vals = [r["pwm"][c] for r in seg]
            m = sum(vals) / len(vals)
            means.append(m)
            print(f"  PWM{c + 1}  {m:7.1f}  {min(vals):7.1f}~{max(vals):7.1f}  {max(vals) - min(vals):6.1f}")
        spread = max(means) - min(means)
        print(f"  四路均值极差 {spread:.1f}  相对 {100 * spread / (sum(means) / 4):.1f}%")
        print(f"  roll 中位 {sorted(r['roll'] for r in seg if r['roll'] == r['roll'])[len(seg) // 2]:.2f}°  "
              f"pitch 中位 {sorted(r['pitch'] for r in seg if r['pitch'] == r['pitch'])[len(seg) // 2]:.2f}°")
        vs = [((r["vx"] ** 2 + r["vy"] ** 2) ** 0.5) for r in seg if r["vx"] == r["vx"]]
        if vs:
            vs.sort()
            print(f"  fcVel 中位 {vs[len(vs) // 2]:.1f} cm/s  p90 {vs[int(len(vs) * 0.9)]:.1f} cm/s")


if __name__ == "__main__":
    main()

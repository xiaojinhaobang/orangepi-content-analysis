#!/usr/bin/env python3
"""Compare /velocity_map (map frame) with /fc_vel (body frame) rotated by yaw."""
import argparse
import math
import re

RE_PSI = re.compile(r"psi=\s*(-?[\d.]+|nan)")
RE_FILT = re.compile(r"filt=\(\s*(-?[\d.]+|nan),\s*(-?[\d.]+|nan)\)")
RE_FC = re.compile(r"fcVelX=\s*(-?[\d.]+|nan)\s+fcVelY=\s*(-?[\d.]+|nan)")


def num(s):
    try:
        return float(s)
    except ValueError:
        return float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--min_v", type=float, default=5.0)
    args = ap.parse_args()

    for path in args.logs:
        angs, dots, mags, angs0 = [], [], [], []
        with open(path, "r", encoding="utf-8", errors="ignore") as fh:
            for line in fh:
                mp, mf, mc = RE_PSI.search(line), RE_FILT.search(line), RE_FC.search(line)
                if not (mp and mf and mc):
                    continue
                psi = num(mp.group(1))
                vx, vy = num(mf.group(1)), num(mf.group(2))
                cx, cy = num(mc.group(1)), num(mc.group(2))
                if any(v != v for v in (psi, vx, vy, cx, cy)):
                    continue
                if math.hypot(vx, vy) < args.min_v or math.hypot(cx, cy) < args.min_v:
                    continue
                r = math.radians(psi)
                wx = cx * math.cos(r) - cy * math.sin(r)
                wy = cx * math.sin(r) + cy * math.cos(r)
                n1, n2 = math.hypot(vx, vy), math.hypot(wx, wy)
                d = (vx * wx + vy * wy) / (n1 * n2)
                dots.append(d)
                angs.append(math.degrees(math.acos(max(-1.0, min(1.0, d)))))
                mags.append(n1 / max(n2, 0.1))
                d0 = (vx * cx + vy * cy) / (n1 * math.hypot(cx, cy))
                angs0.append(math.degrees(math.acos(max(-1.0, min(1.0, d0)))))
        if not angs:
            print(f"{path}: no comparable samples")
            continue
        angs.sort()
        dots.sort()
        mags.sort()
        print(f"\n=== {path}")
        print(f"  samples {len(angs)}")
        print(f"  angle (0 = same dir): median {angs[len(angs)//2]:.0f} deg"
              f"  p25 {angs[len(angs)//4]:.0f}  p75 {angs[3*len(angs)//4]:.0f}")
        print(f"  dot: median {dots[len(dots)//2]:+.2f}")
        print(f"  mag ratio vel_map/|fc_vel|: median {mags[len(mags)//2]:.2f}")
        angs0.sort()
        print(f"  control (no rotation): median {angs0[len(angs0)//2]:.0f} deg")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""对比飞控罗盘航向(fcYaw) 与 SLAM 航向(psi) 的偏差：磁干扰会直接体现成这个差值漂移。"""
import argparse
import re
import sys

RE_T = re.compile(r"t=\s*(-?[\d.]+)")
RE_PSI = re.compile(r"psi=\s*(-?[\d.]+|nan)")
RE_YAW = re.compile(r"fcYaw=\s*(-?[\d.]+|nan)")
RE_THR = re.compile(r"fcThr=\s*(-?[\d.]+|nan)")


def num(s):
    try:
        return float(s)
    except ValueError:
        return float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--thr", type=float, default=480.0)
    a = ap.parse_args()
    for p in a.logs:
        rows = []
        with open(p, "r", encoding="utf-8", errors="ignore") as fh:
            for line in fh:
                mt, mp, my = RE_T.search(line), RE_PSI.search(line), RE_YAW.search(line)
                if not (mt and mp and my):
                    continue
                psi, yaw = num(mp.group(1)), num(my.group(1))
                if psi != psi or yaw != yaw:
                    continue
                mthr = RE_THR.search(line)
                rows.append((num(mt.group(1)), psi, yaw, num(mthr.group(1)) if mthr else float("nan")))
        fly = [r for r in rows if r[3] == r[3] and r[3] > a.thr]
        print("\n%s  样本 %d  飞行段 %d" % (p.split("/")[-1].split("\\")[-1], len(rows), len(fly)))
        if len(fly) < 20:
            print("  无飞行段（油门未超过阈值）")
            continue
        diffs = [((r[2] - r[1] + 180) % 360 - 180, r[0]) for r in fly]
        d = [x[0] for x in diffs]
        span = max(1.0, diffs[-1][1] - diffs[0][1])
        print("  fcYaw-psi: 起 %+.1f° 末 %+.1f° 范围 %+.1f~%+.1f° 漂移 %+.1f°/s"
              % (d[0], d[-1], min(d), max(d), (d[-1] - d[0]) / span))


if __name__ == "__main__":
    sys.exit(main())

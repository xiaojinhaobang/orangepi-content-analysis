#!/usr/bin/env python3
"""For every big pose step, decide: did the pose come back (SLAM glitch) or stay (the airframe moved)?"""
import argparse
import math
import re

RE_T = re.compile(r"t=\s*(-?[\d.]+)")
RE_XY = re.compile(r"x=\s*(-?[\d.]+|nan)\s+y=\s*(-?[\d.]+|nan)")
RE_G = re.compile(r"fcGyroXY=\(\s*(-?[\d.]+|nan),\s*(-?[\d.]+|nan)\)")
RE_FC = re.compile(r"fcVelX=\s*(-?[\d.]+|nan)\s+fcVelY=\s*(-?[\d.]+|nan)")


def num(s):
    try:
        return float(s)
    except ValueError:
        return float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--step", type=float, default=8.0, help="判定为跳变的单帧位移 cm")
    args = ap.parse_args()

    for path in args.logs:
        rows = []
        with open(path, "r", encoding="utf-8", errors="ignore") as fh:
            for line in fh:
                mt, mp = RE_T.search(line), RE_XY.search(line)
                if not (mt and mp):
                    continue
                x, y = num(mp.group(1)), num(mp.group(2))
                if x != x or y != y:
                    continue
                mg = RE_G.search(line)
                g = math.hypot(num(mg.group(1)), num(mg.group(2))) if mg else float("nan")
                rows.append((num(mt.group(1)), x * 100, y * 100, g))
        if len(rows) < 200:
            print(f"{path}: 样本不足")
            continue

        jumps, stays = [], []
        for i in range(1, len(rows) - 12):
            dt = rows[i][0] - rows[i - 1][0]
            if not (0.01 < dt < 0.3):
                continue
            d = math.hypot(rows[i][1] - rows[i - 1][1], rows[i][2] - rows[i - 1][2])
            if d < args.step:
                continue
            # 1 秒后还在不在原处
            later = rows[i + 12]
            back = math.hypot(later[1] - rows[i - 1][1], later[2] - rows[i - 1][2])
            g = rows[i][3]
            if back < 0.4 * d:
                jumps.append((rows[i][0], d, g))
            else:
                stays.append((rows[i][0], d, g))
        print(f"\n=== {path.split(chr(92))[-1]}")
        print(f"单帧位移 >{args.step:.0f}cm 的事件：回到原处 {len(jumps)} 次（SLAM 跳变），"
              f"留在新位置 {len(stays)} 次（飞机真的动了）")
        for label, lst in (("跳变", jumps), ("位移", stays)):
            if not lst:
                continue
            gs = [g for _, _, g in lst if g == g]
            gs.sort()
            med_g = gs[len(gs) // 2] if gs else float("nan")
            dmed = sorted(d for _, d, _ in lst)[len(lst) // 2]
            print(f"  {label}: 中位幅度 {dmed:.0f} cm，那一刻陀螺中位 {med_g:.0f}（raw）")


if __name__ == "__main__":
    main()

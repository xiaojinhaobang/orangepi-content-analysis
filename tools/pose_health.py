#!/usr/bin/env python3
"""One line per rec_*.log: how healthy was the map pose while the airframe was still?"""
import argparse
import glob
import math
import os
import re

RE_T = re.compile(r"t=\s*(-?[\d.]+)")
RE_XY = re.compile(r"x=\s*(-?[\d.]+|nan)\s+y=\s*(-?[\d.]+|nan)")
RE_G = re.compile(r"fcGyroXY=\(\s*(-?[\d.]+|nan),\s*(-?[\d.]+|nan)\)")
RE_FC = re.compile(r"fcVelX=\s*(-?[\d.]+|nan)\s+fcVelY=\s*(-?[\d.]+|nan)")
RE_THR = re.compile(r"fcThr=\s*(-?[\d.]+|nan)")
RE_PWM = re.compile(r"fcPwm=\(\s*(-?[\d.]+|nan),\s*(-?[\d.]+|nan),\s*(-?[\d.]+|nan),\s*(-?[\d.]+|nan)\)")


def num(s):
    try:
        return float(s)
    except ValueError:
        return float("nan")


def analyse(path):
    rows = []
    with open(path, "r", encoding="utf-8", errors="ignore") as fh:
        for line in fh:
            mt, mp = RE_T.search(line), RE_XY.search(line)
            if not (mt and mp):
                continue
            x, y = num(mp.group(1)), num(mp.group(2))
            if x != x or y != y:
                continue
            mg, mf = RE_G.search(line), RE_FC.search(line)
            g = math.hypot(num(mg.group(1)), num(mg.group(2))) if mg else float("nan")
            fv = math.hypot(num(mf.group(1)), num(mf.group(2))) if mf else float("nan")
            mthr, mpwm = RE_THR.search(line), RE_PWM.search(line)
            thr = num(mthr.group(1)) if mthr else float("nan")
            pwm = max((num(mpwm.group(i)) for i in range(1, 5)), default=float("nan")) if mpwm else float("nan")
            rows.append((num(mt.group(1)), x * 100, y * 100, g, fv, thr, pwm))
    if len(rows) < 200:
        return None

    # 只看“机体没动”的样本：陀螺小 且 飞控速度小
    still = [r for r in rows if r[3] == r[3] and r[4] == r[4] and r[3] < 10 and r[4] < 5]
    if len(still) < 100:
        return None
    idx = {r[0]: i for i, r in enumerate(rows)}
    speeds = []
    for t, x, y, _, _, _, _ in still:
        i = idx[t]
        a, b = max(0, i - 5), min(len(rows) - 1, i + 5)
        span = rows[b][0] - rows[a][0]
        if span >= 0.4:
            speeds.append(math.hypot(rows[b][1] - rows[a][1], rows[b][2] - rows[a][2]) / span)
    speeds.sort()
    if not speeds:
        return None
    med = speeds[len(speeds) // 2]
    p90 = speeds[int(len(speeds) * 0.9)]
    far = max(math.hypot(r[1], r[2]) for r in rows)

    # 飞行段：电机在转（PWM>50 或 thr>500）时的“地图速度 / 飞控速度”
    idx = {r[0]: i for i, r in enumerate(rows)}
    fly = [r for r in rows
           if (r[6] == r[6] and r[6] > 50) or (r[5] == r[5] and r[5] > 500)]
    mv, cv = [], []
    for r in fly:
        i = idx[r[0]]
        a, b = max(0, i - 5), min(len(rows) - 1, i + 5)
        span = rows[b][0] - rows[a][0]
        if span >= 0.4:
            mv.append(math.hypot(rows[b][1] - rows[a][1], rows[b][2] - rows[a][2]) / span)
            if r[4] == r[4]:
                cv.append(r[4])
    ratio = None
    if len(mv) > 50 and cv:
        mv.sort()
        cv.sort()
        m_med = mv[len(mv) // 2]
        c_med = cv[len(cv) // 2]
        ratio = (m_med, c_med, m_med / max(c_med, 1.0), len(mv))
    return (len(rows), len(still), med, p90, far, med > 5.0 or far > 300, ratio)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("paths", nargs="+")
    args = ap.parse_args()
    files = []
    for p in args.paths:
        files.extend(sorted(glob.glob(p)))
    print(f"{'文件':30s} {'飞行样本':>8s} {'地图v':>7s} {'飞控v':>7s} {'倍数':>6s}")
    for f in files:
        r = analyse(f)
        name = os.path.basename(f)[:38]
        if r is None:
            print(f"{name:38s} {'—':>7s}")
            continue
        n, ns, med, p90, far, bad, ratio = r
        if ratio is None:
            print(f"{name:30s} {'无飞行段':>8s}")
            continue
        m_med, c_med, q, cnt = ratio
        flag = " <<<" if q > 3.0 else ""
        print(f"{name:30s} {cnt:8d} {m_med:7.1f} {c_med:7.1f} {q:6.1f}{flag}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""分析 leg_test.py 的 CSV：每个直线航段的侧滑 / 速度跟踪 / 航向 / 高度。

机体系直线段里，理想运动是**纯沿指令轴**；垂直分量就是"侧滑"（最直接的直线质量指标）。
用法: python tools/leg_csv_report.py <leg_test.csv>
"""
import csv
import math
import sys


def wrap(d):
    return (d + 180.0) % 360.0 - 180.0


def unwrap_series(a):
    out = [a[0]]
    for v in a[1:]:
        out.append(out[-1] + wrap(v - out[-1]))
    return out


def main():
    rows = list(csv.DictReader(open(sys.argv[1])))
    groups = []
    cur = None
    for r in rows:
        key = (r["phase"], r["dir"])
        if cur is None or cur["key"] != key:
            cur = {"key": key, "rows": []}
            groups.append(cur)
        cur["rows"].append(r)

    print("方向  时长   指令  实际  跟踪比 | 位移(cm)  前进分量  侧滑分量  方向偏差 | 航向变化 峰峰 | 高度变化 范围")
    tot = []
    for g in groups:
        phase, d = g["key"]
        if phase != "leg":
            continue
        rs = g["rows"]
        t0, t1 = float(rs[0]["t"]), float(rs[-1]["t"])
        dur = t1 - t0
        x0, y0 = float(rs[0]["x"]), float(rs[0]["y"])
        x1, y1 = float(rs[-1]["x"]), float(rs[-1]["y"])
        dx, dy = x1 - x0, y1 - y0
        dist = math.hypot(dx, dy)
        psi0 = float(rs[0]["psi"])
        exp = {"f": psi0, "b": psi0 + 180.0, "r": psi0 - 90.0, "l": psi0 + 90.0}[d]
        act = math.degrees(math.atan2(dy, dx))
        err = wrap(act - exp)
        fwd = dist * math.cos(math.radians(err))
        lat = dist * math.sin(math.radians(err))
        vcmd = float(rs[0]["v_cmd"])
        vact = dist / dur if dur > 0 else math.nan
        psi = unwrap_series([float(r["psi"]) for r in rs])
        h = [float(r["h"]) for r in rs]
        print("%-4s %5.1fs %5.1f %5.1f %5.2f  | %6.0f   %+7.0f   %+8.0f  %+8.1f° | %+7.1f° %5.1f° | %+5.0f cm %3.0f~%3.0f"
              % (d, dur, vcmd, vact, (vact / vcmd if vcmd else math.nan),
                 dist, fwd, lat, err,
                 psi[-1] - psi[0], max(psi) - min(psi), h[-1] - h[0], min(h), max(h)))
        tot.append((vcmd, vact, dist, err, psi[-1] - psi[0], max(psi) - min(psi), h[-1] - h[0]))
    if tot:
        import statistics as st
        print("\n汇总：跟踪比 %.2f（%.2f~%.2f） | 段长 %.0f cm | 方向偏差 中位 %.1f° 最大 %.1f°"
              " | 航向变化 中位 %.1f° 最大 %.1f° | 高度变化 中位 %+.0f cm"
              % (st.mean(t[1] / t[0] for t in tot),
                 min(t[1] / t[0] for t in tot), max(t[1] / t[0] for t in tot),
                 st.median(t[2] for t in tot),
                 st.median(abs(t[3]) for t in tot), max(abs(t[3]) for t in tot),
                 st.median(abs(t[4]) for t in tot), max(abs(t[4]) for t in tot),
                 st.median(t[6] for t in tot)))


if __name__ == "__main__":
    main()

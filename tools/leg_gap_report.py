#!/usr/bin/env python3
"""看 leg_test CSV 里"换方向"的 gap 段：机头（psi）偏了多少、偏完有没有回来。

用法: python tools/leg_gap_report.py <leg_test.csv> [每行秒数，默认 0.5]
"""
import csv
import math
import sys


def wrap(d):
    return (d + 180.0) % 360.0 - 180.0


def main():
    rows = list(csv.DictReader(open(sys.argv[1])))
    step = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
    groups = []
    cur = None
    for r in rows:
        key = (r["phase"], r["dir"])
        if cur is None or cur["key"] != key:
            cur = {"key": key, "rows": []}
            groups.append(cur)
        cur["rows"].append(r)

    print("=== 所有 gap 段（换方向，指令速度=0）===")
    for g in groups:
        phase, d = g["key"]
        if phase != "gap":
            continue
        rs = g["rows"]
        t0 = float(rs[0]["t"])
        psi0 = float(rs[0]["psi"])
        psi = []
        acc = psi0
        prev = psi0
        for r in rs:
            acc += wrap(float(r["psi"]) - prev)
            prev = float(r["psi"])
            psi.append(acc)
        dev = [p - psi0 for p in psi]
        dev_max = max(dev, key=abs)
        t_max = float(rs[dev.index(dev_max)]["t"]) - t0
        h = [float(r["h"]) for r in rs]
        damp = [float(r["damp_q"]) for r in rs]
        vyaw = [float(r["vyaw_cmd"]) for r in rs]
        print("\n-- 上一段 %s → 下一段（gap %.1fs）--" % (d, float(rs[-1]["t"]) - t0))
        print("   机头偏: 峰值 %+.1f°（在 %.1fs 处）  结束时 %+.1f°  |  高度 %.0f~%.0fcm"
              "  |  阻尼命令 %+.1f~%+.1f（非零 %d/%d）"
              % (dev_max, t_max, dev[-1], min(h), max(h),
                 min(damp), max(damp), sum(1 for x in damp if x != 0), len(damp)))
        # 抽样打印
        n = len(rs)
        ts = sorted(set(float(r["t"]) for r in rs))
        dts = [b - a for a, b in zip(ts, ts[1:]) if b > a]
        dtm = min(dts) if dts else 0.02
        stride = max(1, int(round(step / dtm)))
        line = []
        for k in range(0, n, stride):
            line.append("%.1fs:%+.1f°" % (float(rs[k]["t"]) - t0, dev[k]))
        print("   " + "  ".join(line))


if __name__ == "__main__":
    main()

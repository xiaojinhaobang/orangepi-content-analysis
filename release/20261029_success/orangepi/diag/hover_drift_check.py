#!/usr/bin/env python3
"""悬停段"漂移方向 + 偏航力矩"检查（不用飞，直接读已有日志）。

对每个干净悬停段（unlock=1）算：
  · 机体坐标系的水平位移（前/左）—— 回答"是不是每次都往左飘"
  · 航向净变化 —— 回答"航向是不是一直往一个方向"
  · 陀螺 Z 的均值（°/s）：全段 / 前10秒 / 后段 —— 回答"有没有恒定偏航力矩、是不是越往后越明显"

用法: python3 ~/diag/hover_drift_check.py [扫最新 N 个日志, 默认 30]
"""
import glob
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dump_flight_segments import parse                                  # noqa: E402
from hover_report import clean_h, merged_runs                            # noqa: E402

GYRO_SCALE = 18.4          # LSB / (deg/s)
GYRO_NOTE = "18.4"


def unwrap(vals):
    out, acc, prev = [], 0.0, None
    for v in vals:
        if math.isnan(v):
            out.append(float('nan'))
            continue
        if prev is not None:
            d = (v - prev + 180.0) % 360.0 - 180.0
            acc += d
        prev = v
        out.append(acc)
    return out


def seg_report(seg):
    t0 = seg[0]["t"]
    xs = [(r["t"], r["x"], r["y"], r["psi"]) for r in seg
          if not math.isnan(r["x"]) and not math.isnan(r["y"]) and not math.isnan(r["psi"])]
    if len(xs) < 40:
        return None
    _, x0, y0, _ = xs[0]
    _, x1, y1, _ = xs[-1]
    psi = sum(r[3] for r in xs) / len(xs)
    r = math.radians(psi)
    dx, dy = x1 - x0, y1 - y0
    fwd = dx * math.cos(r) + dy * math.sin(r)
    left = -dx * math.sin(r) + dy * math.cos(r)
    ps = unwrap([v for _, _, _, v in xs])
    net_yaw = ps[-1] - ps[0]
    def gz(lo, hi):
        """[lo,hi) 秒窗口内的飞行陀螺 Z 均值，单位 LSB。"""
        v = [r_["gzz"] for r_ in seg
             if lo <= r_["t"] - t0 < hi and not math.isnan(r_.get("gzz", float('nan')))]
        return (sum(v) / len(v)) if v else float('nan')

    def yc(lo, hi):
        v = [r_["yawcmd"] for r_ in seg
             if lo <= r_["t"] - t0 < hi and not math.isnan(r_.get("yawcmd", float('nan')))]
        return (sum(v) / len(v)) if v else float('nan')

    inf = float('inf')
    return dict(dur=seg[-1]["t"] - t0, h=sum(r_["h"] for r_ in seg) / len(seg),
                fwd=fwd, left=left, mag=math.hypot(dx, dy), psi=psi,
                net_yaw=net_yaw,
                gz_all=gz(0.0, inf), gz_head=gz(0.0, 10.0), gz_tail=gz(10.0, inf),
                yc_head=yc(0.0, 10.0), yc_tail=yc(10.0, inf))


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 30
    cands = []
    for pat in ("rec_*.log",):
        cands += glob.glob(os.path.expanduser("~/traces/" + pat))
    cands.sort(key=os.path.getmtime, reverse=True)
    print('%-13s %6s %5s %8s %8s %8s %8s | %9s %9s'
          % ('时间', '时长s', '高cm', '前(cm)', '左(cm)', '航向°', '净航向°',
             '陀螺Z全段', '前10s/后段'))
    print('# fcGyroZ 单位 LSB，括号里按 %s LSB/(°/s) 折算成 °/s' % GYRO_NOTE)
    rows = []
    for p in cands[:n]:
        name = os.path.basename(p)
        try:
            rows_ = clean_h(parse(p))
        except Exception:                                                  # noqa: BLE001
            continue
        for seg in merged_runs(rows_):
            rep = seg_report(seg)
            if rep:
                rep['file'] = name
                rep['t0'] = seg[0]["t"]
                rows.append(rep)
    for d in rows:
        print('%-13s %6.1f %5.0f %+8.1f %+8.1f %8.1f %+8.1f | %+7.1f(%+5.2f)  %+6.1f/%+6.1f'
              % (d['file'][:13], d['dur'], d['h'], d['fwd'], d['left'], d['psi'],
                 d['net_yaw'], d['gz_all'], d['gz_all'] / GYRO_SCALE,
                 d['gz_head'], d['gz_tail']))
    lefts = [d['left'] for d in rows if not math.isnan(d['left'])]
    if lefts:
        nl = sum(1 for v in lefts if v > 0)
        print('\n共 %d 段：向左漂 %d 段，向右漂 %d 段（左漂占比 %.0f%%）'
              % (len(lefts), nl, len(lefts) - nl, 100.0 * nl / len(lefts)))
    gy = [d for d in rows if not math.isnan(d['gz_all'])]
    if gy:
        pos = sum(1 for d in gy if d['gz_all'] > 0)
        print('陀螺Z 恒为正 %d 段 / 恒为负 %d 段（正=左转）' % (pos, len(gy) - pos))
        w = [d for d in gy if not math.isnan(d['gz_tail']) and not math.isnan(d['gz_head'])]
        if w:
            bigger = sum(1 for d in w if abs(d['gz_tail']) > abs(d['gz_head']))
            print('后段|陀螺Z|>前10s 的段数：%d/%d（"越飞越歪"的量化）' % (bigger, len(w)))
    return 0


if __name__ == '__main__':
    sys.exit(main())

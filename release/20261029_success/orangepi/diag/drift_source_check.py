#!/usr/bin/env python3
"""左漂溯源：是"雷达报的速度有偏"还是"机体真被推偏"。

对每个悬停段算两个互相独立的量：

  A. SLAM 实测速度（机体系，cm/s）—— 用最小二乘拟合 x(t)/y(t) 的斜率，
     这是"飞机真的在往哪走"，和速度源无关。
  B. 雷达原始速度 raw（机体系均值）—— 雷达自己说飞机在往哪走。

判定：
  · A 大、B 也大且同向  → 雷达没瞎，是飞控没把速度压住（环/注入问题）
  · A 大、B ≈ 0         → 雷达看不见这个漂移（雷达偏置/标定）
  · A 大、B 反向        → 雷达速度符号/坐标有错

同时给前 10s / 后段的对比，回答"是不是十秒后才开始漂"。

用法: python3 ~/diag/drift_source_check.py [扫最新 N 个日志, 默认 20]
"""
import glob
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dump_flight_segments import parse                                  # noqa: E402
from hover_report import clean_h, merged_runs                            # noqa: E402


def lsq_slope(ts, vs):
    """最小二乘斜率（单位/秒）。点少于 2 个返回 nan。"""
    n = len(ts)
    if n < 2:
        return float('nan')
    mt = sum(ts) / n
    mv = sum(vs) / n
    den = sum((t - mt) ** 2 for t in ts)
    if den <= 1e-9:
        return float('nan')
    return sum((ts[i] - mt) * (vs[i] - mv) for i in range(n)) / den


def body_of(seg, lo, hi):
    """返回 (slam_fwd, slam_left, raw_fwd, raw_left, filt_fwd, filt_left, n, dur)。"""
    sub = [r for r in seg if lo <= r["t"] - seg[0]["t"] < hi]
    pos = [r for r in sub if not math.isnan(r["x"]) and not math.isnan(r["y"])
           and not math.isnan(r["psi"])]
    if len(pos) < 30:
        return None
    ts = [r["t"] for r in pos]
    xs = [r["x"] for r in pos]
    ys = [r["y"] for r in pos]
    psi = sum(r["psi"] for r in pos) / len(pos)
    cr, sr = math.cos(math.radians(psi)), math.sin(math.radians(psi))
    vx, vy = lsq_slope(ts, xs), lsq_slope(ts, ys)
    slam_f = vx * cr + vy * sr
    slam_l = -vx * sr + vy * cr

    def mean2(key_x, key_y):
        v = [(r[key_x], r[key_y]) for r in sub
             if not math.isnan(r.get(key_x, float('nan')))
             and not math.isnan(r.get(key_y, float('nan')))]
        if not v:
            return float('nan'), float('nan')
        mx = sum(a for a, _ in v) / len(v)
        my = sum(b for _, b in v) / len(v)
        return mx * cr + my * sr, -mx * sr + my * cr

    rf, rl = mean2("rx", "ry")
    ff, fl = mean2("fx", "fy")
    return dict(dur=ts[-1] - ts[0], n=len(pos), psi=psi,
                slam_f=slam_f, slam_l=slam_l, raw_f=rf, raw_l=rl,
                filt_f=ff, filt_l=fl)


def show(tag, d):
    if d is None:
        print("    %-6s 样本不足" % tag)
        return
    print("    %-6s %5.1fs | SLAM实测 前%+6.2f 左%+6.2f | 雷达raw 前%+6.2f 左%+6.2f"
          " | 注入 前%+6.2f 左%+6.2f cm/s"
          % (tag, d["dur"], d["slam_f"], d["slam_l"],
             d["raw_f"], d["raw_l"], d["filt_f"], d["filt_l"]))


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 20
    cands = glob.glob(os.path.expanduser("~/traces/rec_*.log"))
    cands.sort(key=os.path.getmtime, reverse=True)
    seg_i = 0
    tot = []
    for p in cands[:n]:
        name = os.path.basename(p)
        try:
            rows_ = clean_h(parse(p))
        except Exception:                                                  # noqa: BLE001
            continue
        for seg in merged_runs(rows_):
            if seg[-1]["t"] - seg[0]["t"] < 8.0:
                continue
            full = body_of(seg, 0.0, float('inf'))
            if full is None:
                continue
            seg_i += 1
            print("%s  段%d  psi=%+.1f°" % (name, seg_i, full["psi"]))
            show("全段", full)
            show("前10s", body_of(seg, 0.0, 10.0))
            show("后段", body_of(seg, 10.0, float('inf')))
            tot.append(full)
    if not tot:
        print("没有可用悬停段")
        return 0
    print("\n==== 全段汇总（%d 段） ====" % len(tot))
    for key, label in (("slam_f", "SLAM实测 前"), ("slam_l", "SLAM实测 左"),
                       ("raw_f", "雷达raw  前"), ("raw_l", "雷达raw  左"),
                       ("filt_f", "注入速度 前"), ("filt_l", "注入速度 左")):
        v = [d[key] for d in tot if not math.isnan(d[key])]
        if v:
            m = sum(v) / len(v)
            sd = (sum((x - m) ** 2 for x in v) / len(v)) ** 0.5
            pos = sum(1 for x in v if x > 0)
            print("  %s: 均值 %+6.2f ± %5.2f cm/s   为正 %d/%d"
                  % (label, m, sd, pos, len(v)))
    return 0


if __name__ == '__main__':
    sys.exit(main())

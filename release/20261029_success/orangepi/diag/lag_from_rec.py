#!/usr/bin/env python3
"""从记录器日志里量"雷达速度 -> 飞控手里"的延迟（互相关 raw vs fc回显）。

用法: python3 ~/diag/lag_from_rec.py <日志> [起始t] [结束t]
不写时间范围就自动找"有运动"的那一段（|raw| 明显大于静止底噪）。
"""
import math
import re
import sys

RE_T = re.compile(r't=\s*([-\d.]+)')
RE_RAW = re.compile(r'raw=\(\s*([-\d.]+|nan),\s*([-\d.]+|nan)\)')
RE_ECHO = re.compile(r'fc回显=\(\s*(-?[\d.]+|nan),\s*(-?[\d.]+|nan)\)')


def f(x):
    try:
        return float(x)
    except ValueError:
        return float('nan')


def load(path):
    rows = []
    with open(path, errors='ignore') as fh:
        for ln in fh:
            mt, mr, me = RE_T.search(ln), RE_RAW.search(ln), RE_ECHO.search(ln)
            if not (mt and mr and me):
                continue
            rows.append((f(mt.group(1)), f(mr.group(1)), f(mr.group(2)),
                         f(me.group(1)), f(me.group(2))))
    return rows


def resample(rows, idx, rate=20.0):
    pts = [(r[0], r[idx]) for r in rows if not math.isnan(r[idx])]
    if len(pts) < 10:
        return None
    t0, t1 = pts[0][0], pts[-1][0]
    dt = 1.0 / rate
    n = int((t1 - t0) / dt) + 1
    out, j = [], 0
    for i in range(n):
        t = t0 + i * dt
        while j + 1 < len(pts) and pts[j + 1][0] < t:
            j += 1
        out.append(pts[j][1])
    return t0, dt, out


def lag(dt, a, b, max_lag_s=1.0):
    n = min(len(a), len(b))
    ma, mb = sum(a[:n]) / n, sum(b[:n]) / n
    va = [x - ma for x in a[:n]]
    vb = [x - mb for x in b[:n]]
    best, bl = -1e18, 0
    for L in range(-int(max_lag_s / dt), int(max_lag_s / dt) + 1):
        s, c = 0.0, 0
        for i in range(max(0, L), min(n, n + L)):
            s += va[i] * vb[i - L]
            c += 1
        if c > 20 and s / c > best:
            best, bl = s / c, L
    return bl * dt, best


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    rows = load(sys.argv[1])
    if len(rows) < 50:
        print('样本太少')
        return 1
    if len(sys.argv) >= 4:
        lo, hi = float(sys.argv[2]), float(sys.argv[3])
    else:
        # 自动找运动段：|raw| 的中位数 + 5 倍
        mags = sorted(math.hypot(r[1], r[2]) for r in rows if not math.isnan(r[1]))
        med = mags[len(mags) // 2]
        thr = max(4.0, med * 5.0)
        act = [r[0] for r in rows if not math.isnan(r[1]) and math.hypot(r[1], r[2]) > thr]
        if len(act) < 40:
            print('没找到明显运动段（静止中位 |raw|=%.1f，阈值 %.1f）' % (med, thr))
            return 1
        lo, hi = act[0], act[-1]
        print('自动找到运动段: t=%.1f..%.1f（%.1f 秒，静止底噪 %.1f）'
              % (lo, hi, hi - lo, med))
    sub = [r for r in rows if lo <= r[0] <= hi]
    print('窗口样本: %d' % len(sub))
    for tag, ia, ib in (('x 分量', 1, 3), ('y 分量', 2, 4)):
        ra = resample(sub, ia)
        rb = resample(sub, ib)
        if not (ra and rb):
            continue
        n = min(len(ra[2]), len(rb[2]))
        # 去均值后的相关系数
        a, b = ra[2][:n], rb[2][:n]
        ma, mb = sum(a) / n, sum(b) / n
        num = sum((a[i] - ma) * (b[i] - mb) for i in range(n))
        da = math.sqrt(sum((x - ma) ** 2 for x in a))
        db = math.sqrt(sum((x - mb) ** 2 for x in b))
        r0 = num / (da * db) if da > 0 and db > 0 else float('nan')
        L, _ = lag(ra[1], a, b)
        print('  %s: 零延迟相关 %.2f   最佳延迟 %+.0f ms（>0 = 飞控手里的值领先）'
              % (tag, r0, L * 1000))
    return 0


if __name__ == '__main__':
    sys.exit(main())

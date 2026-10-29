#!/usr/bin/env python3
"""分析 fast_log.py 的记录：速度噪声谱 / 延迟 / 台阶。

用法: python3 ~/diag/fast_analyze.py /tmp/fast_xxxx.csv
"""
import math
import sys
from collections import defaultdict


def load(path):
    d = defaultdict(list)
    with open(path, errors='ignore') as fh:
        for ln in fh:
            if ln.startswith('#') or not ln.strip():
                continue
            p = ln.strip().split(',')
            if len(p) < 4:
                continue
            try:
                t, v1, v2 = float(p[0]), float(p[2]), float(p[3])
            except ValueError:
                continue
            v3 = float('nan')
            if len(p) > 4 and p[4] not in ('', 'nan'):
                try:
                    v3 = float(p[4])
                except ValueError:
                    pass
            d[p[1]].append((t, v1, v2, v3))
    return d


def stats(v):
    n = len(v)
    if n == 0:
        return 0, 0.0, 0.0
    m = sum(v) / n
    sd = math.sqrt(sum((x - m) ** 2 for x in v) / n) if n > 1 else 0.0
    return n, m, sd


def resample(rows, col, rate=100.0):
    """线性插值到均匀 rate Hz 网格 -> (t0, dt, [...])"""
    pts = [(r[0], r[col]) for r in rows if not math.isnan(r[col])]
    if len(pts) < 4:
        return None
    t0, t1 = pts[0][0], pts[-1][0]
    dt = 1.0 / rate
    n = int((t1 - t0) / dt) + 1
    out, j = [], 0
    for i in range(n):
        t = t0 + i * dt
        while j + 1 < len(pts) and pts[j + 1][0] < t:
            j += 1
        a, b = pts[j], pts[min(j + 1, len(pts) - 1)]
        if b[0] > a[0]:
            w = (t - a[0]) / (b[0] - a[0])
            out.append(a[1] + w * (b[1] - a[1]))
        else:
            out.append(a[1])
    return t0, dt, out


def band_amp(dt, v, fmin, fmax, step=0.05):
    """在 fmin..fmax 扫正弦，返回（平均幅度, 最大幅度）"""
    n = len(v)
    if n < 20:
        return 0.0, 0.0
    mean = sum(v) / n
    best = 0.0
    acc = 0.0
    cnt = 0
    f = fmin
    while f <= fmax + 1e-9:
        a = b = 0.0
        w = 2.0 * math.pi * f
        for i in range(n):
            x = v[i] - mean
            a += x * math.sin(w * i * dt)
            b += x * math.cos(w * i * dt)
        amp = 2.0 * math.hypot(a, b) / n
        acc += amp
        cnt += 1
        if amp > best:
            best = amp
        f += step
    return (acc / cnt if cnt else 0.0), best


def best_lag(dt, a, b, max_lag_s=0.6):
    """互相关找 a 相对 b 的延迟（秒）。>0 表示 a 落后 b。"""
    n = min(len(a), len(b))
    max_lag = int(max_lag_s / dt)
    ma, mb = sum(a[:n]) / n, sum(b[:n]) / n
    va = [x - ma for x in a[:n]]
    vb = [x - mb for x in b[:n]]
    best, bl = -1e18, 0
    for lag in range(-max_lag, max_lag + 1):
        s = 0.0
        c = 0
        for i in range(max(0, lag), min(n, n + lag)):
            s += va[i] * vb[i - lag]
            c += 1
        if c < 20:
            continue
        if s / c > best:
            best, bl = s / c, lag
    return bl * dt


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    pos = [a for a in sys.argv[1:] if not a.startswith('--')]
    d = load(pos[0])
    if len(pos) >= 3:                     # 可选：只分析 [lo, hi] 秒这段
        lo, hi = float(pos[1]), float(pos[2])
        for k in list(d):
            d[k] = [r for r in d[k] if lo <= r[0] <= hi]
        print('只分析 t=%.1f..%.1f s' % (lo, hi))
    if not d:
        print('没读到数据')
        return 1
    print('=== 各话题统计 ===')
    print('%-8s %6s %8s %9s %11s %11s' %
          ('话题', '条数', '时长s', '频率Hz', '均值1', '均值2'))
    for k in ('vmap', 'vfilt', 'vfus', 'vimu', 'gyro', 'echo'):
        rows = d.get(k, [])
        if not rows:
            print('%-8s %6d' % (k, 0))
            continue
        dur = rows[-1][0] - rows[0][0]
        _, m1, _ = stats([r[1] for r in rows])
        _, m2, _ = stats([r[2] for r in rows])
        print('%-8s %6d %8.1f %9.1f %+11.3f %+11.3f' %
              (k, len(rows), dur, (len(rows) - 1) / dur if dur > 0 else 0, m1, m2))

    vmap = d.get('vmap', [])
    if vmap:
        _, m1, s1 = stats([r[1] for r in vmap])
        _, m2, s2 = stats([r[2] for r in vmap])
        print('\n=== /velocity_map 噪声（1=x, 2=y，单位 cm/s）===')
        print('  均值  : x=%+.3f  y=%+.3f' % (m1, m2))
        print('  标准差: x=%.2f   y=%.2f' % (s1, s2))
        rs = resample(vmap, 1, 100.0)
        if rs:
            _, dt, v = rs
            print('\n=== x 分量的噪声谱（静止时应接近 0）===')
            for lo, hi, name in ((0.05, 0.2, '0.05-0.2Hz 慢漂'),
                                 (0.2, 1.0, '0.2-1Hz'),
                                 (1.0, 3.0, '1-3Hz'),
                                 (3.0, 10.0, '3-10Hz'),
                                 (10.0, 20.0, '10-20Hz')):
                avg, mx = band_amp(dt, v, lo, hi)
                print('  %-16s 平均 %.3f  最大 %.3f' % (name, avg, mx))
        same = sum(1 for i in range(1, len(vmap)) if vmap[i][1] == vmap[i - 1][1])
        print('  台阶（相邻完全相同）比例: %.0f%%'
              % (100.0 * same / max(1, len(vmap) - 1)))

    echo = d.get('echo', [])
    vfus = d.get('vfus', [])
    vimu = d.get('vimu', [])
    if vmap and vimu:
        ra = resample(vmap, 1, 100.0)
        ri = resample(vimu, 1, 100.0)
        if ra and ri:
            n = min(len(ra[2]), len(ri[2]))
            lag = best_lag(ra[1], ra[2][:n], ri[2][:n])
            print('\n=== 雷达速度相对 IMU 积分速度的延迟（这就是要补偿的 L）===')
            print('  延迟 ≈ %+.0f ms  （>0 = 雷达速度落后 IMU 这么多）' % (lag * 1000))
    if vmap and vfus:
        ra = resample(vmap, 1, 100.0)
        rf = resample(vfus, 1, 100.0)
        if ra and rf:
            n = min(len(ra[2]), len(rf[2]))
            lag = best_lag(ra[1], ra[2][:n], rf[2][:n])
            print('\n=== 延迟（互相关: /velocity_map 相对 /velocity_fused）===')
            print('  x 分量延迟 ≈ %+.0f ms   （>0 = 融合速度领先雷达速度这么多）'
                  % (lag * 1000))
    if vmap and echo:
        ra = resample(vmap, 1, 100.0)
        rb = resample(echo, 1, 100.0)
        if ra and rb:
            n = min(len(ra[2]), len(rb[2]))
            lag = best_lag(ra[1], ra[2][:n], rb[2][:n])
            print('\n=== 延迟（互相关: /velocity_map 相对 /fc_vel_echo）===')
            print('  延迟 ≈ %+.0f ms  （>0 = 香橙派的速度比飞控手里的晚）' % (lag * 1000))
    return 0


if __name__ == '__main__':
    sys.exit(main())

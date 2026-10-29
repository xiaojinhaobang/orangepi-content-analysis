#!/usr/bin/env python3
"""汇总 ~/traces/*.log 的飞行段：高度、水平漂移、航向、速度。

日志行格式：
t= 23.54 x= -2.87 y= -8.36 psi= -176.8 | filt=( 0.0, 0.0) raw=( 1.2, 1.0) tvd=( -1.2, -2.9) z= 30.0 yaw= -2.4 | fc回显=( 0, 0) h=3 mode=2 unlock=0
"""
import glob
import math
import os
import re

import numpy as np

TRACE_DIR = os.path.expanduser('~/traces')

NUM = r'([-\d\.]+|nan)'
RE_T = re.compile(r't=\s*' + NUM)
RE_XY = re.compile(r'x=\s*' + NUM + r'\s+y=\s*' + NUM)
RE_PSI = re.compile(r'psi=\s*' + NUM)
RE_RAW = re.compile(r'raw=\(\s*' + NUM + r',\s*' + NUM + r'\)')
RE_TVD = re.compile(r'tvd=\(\s*' + NUM + r',\s*' + NUM + r'\)')
RE_Z = re.compile(r'z=\s*' + NUM)
RE_YAW = re.compile(r'yaw=\s*' + NUM)
RE_H = re.compile(r'h=(\S+)\s+mode=(\S+)\s+unlock=(\S+)')


def f(m, i=1):
    try:
        return float(m.group(i))
    except (AttributeError, ValueError):
        return math.nan


def parse(path):
    rows = []
    with open(path, errors='ignore') as fh:
        for line in fh:
            mt, mxy = RE_T.search(line), RE_XY.search(line)
            if not mt or not mxy:
                continue
            mh = RE_H.search(line)
            h = math.nan
            unlock = None
            mode = None
            if mh:
                try:
                    h = float(mh.group(1))
                except ValueError:
                    h = math.nan
                mode, unlock = mh.group(2), mh.group(3)
            mraw, mtvd = RE_RAW.search(line), RE_TVD.search(line)
            rows.append({
                't': float(mt.group(1)),
                'x': f(mxy, 1), 'y': f(mxy, 2),
                'psi': f(RE_PSI.search(line)),
                'raw': math.hypot(f(mraw, 1), f(mraw, 2)) if mraw else math.nan,
                'tvd': math.hypot(f(mtvd, 1), f(mtvd, 2)) if mtvd else math.nan,
                'z': f(RE_Z.search(line)),
                'h': h,
                'unlock': unlock,
            })
    return rows


def summarize(path):
    rows = parse(path)
    if not rows:
        return None
    # 飞行段 = 已解锁 且 高度有效
    air = [r for r in rows
           if r['unlock'] == '1' and not math.isnan(r['h']) and r['h'] > 20]
    if len(air) < 20:
        air = [r for r in rows if not math.isnan(r['h']) and r['h'] > 20]
    if len(air) < 20:
        return {'file': os.path.basename(path), 'rows': len(rows), 'air': len(air),
                'note': '无飞行段（地面上/未连接）'}

    t = np.array([r['t'] for r in air])
    h = np.array([r['h'] for r in air])
    x = np.array([r['x'] for r in air])
    y = np.array([r['y'] for r in air])
    psi = np.array([r['psi'] for r in air])
    raw = np.array([r['raw'] for r in air])
    tvd = np.array([r['tvd'] for r in air])

    ok = ~np.isnan(x) & ~np.isnan(y)
    drift = math.nan
    sx = sy = math.nan
    if ok.sum() > 10:
        xm, ym = x[ok].mean(), y[ok].mean()
        d = np.hypot(x[ok] - xm, y[ok] - ym)
        drift = float(d.max())
        sx, sy = float((x[ok] - xm).std()), float((y[ok] - ym).std())

    yaw_pp = math.nan
    pok = ~np.isnan(psi)
    if pok.sum() > 10:
        u = np.unwrap(np.radians(psi[pok]))
        yaw_pp = float(np.degrees(u.max() - u.min()))

    return {
        'file': os.path.basename(path), 'rows': len(rows), 'air': len(air),
        'dur': float(t.max() - t.min()),
        'h_mean': float(h.mean()), 'h_std': float(h.std()),
        'h_min': float(h.min()), 'h_max': float(h.max()),
        'drift': drift, 'sx': sx, 'sy': sy, 'yaw_pp': yaw_pp,
        'raw_max': float(np.nanmax(raw)) if not np.all(np.isnan(raw)) else math.nan,
        'tvd_max': float(np.nanmax(tvd)) if not np.all(np.isnan(tvd)) else math.nan,
    }


def main():
    files = sorted(glob.glob(os.path.join(TRACE_DIR, '*.log')),
                   key=os.path.getmtime, reverse=True)
    print('==== 悬停稳定窗口（自动截取 h 稳定段，用于验证"悬停精度"）====')
    for p in files:
        rows = parse(p)
        air = [r for r in rows
               if r['unlock'] == '1' and not math.isnan(r['h']) and r['h'] > 20]
        if len(air) < 100:
            continue
        h = np.array([r['h'] for r in air])
        med = float(np.median(h))
        mask = np.abs(h - med) <= 5.0
        best = cur = 0
        for v in mask:
            cur = cur + 1 if v else 0
            best = max(best, cur)
        if best < 60:
            continue
        # 找最长连续段
        start = end = 0
        cur = 0
        for i, v in enumerate(mask):
            cur = cur + 1 if v else 0
            if cur > (end - start):
                end, start = i + 1, i + 1 - cur
        seg = air[start:end]
        t = np.array([r['t'] for r in seg])
        hs = np.array([r['h'] for r in seg])
        x = np.array([r['x'] for r in seg])
        y = np.array([r['y'] for r in seg])
        psi = np.array([r['psi'] for r in seg])
        ok = ~np.isnan(x) & ~np.isnan(y)
        drift = sx = sy = math.nan
        if ok.sum() > 20:
            xm, ym = x[ok].mean(), y[ok].mean()
            drift = float(np.hypot(x[ok] - xm, y[ok] - ym).max())
            sx, sy = float((x[ok] - xm).std()), float((y[ok] - ym).std())
        yaw_pp = math.nan
        pok = ~np.isnan(psi)
        if pok.sum() > 20:
            u = np.unwrap(np.radians(psi[pok]))
            yaw_pp = float(np.degrees(u.max() - u.min()))
        print('%-32s 窗口%5.1fs 高度%7.1f±%-4.1f cm | 最大漂移%5.1f  x±%4.1f y±%4.1f | 航向峰峰%5.1f°' % (
            os.path.basename(p).replace('.log', ''), float(t.max() - t.min()),
            float(hs.mean()), float(hs.std()), drift, sx, sy, yaw_pp))

    print()
    print('==== 全部记录总览 ====')
    for p in files:
        s = summarize(p)
        if s is None:
            continue
        if 'note' in s:
            print('%-32s %6d行  %s' % (s['file'].replace('.log', ''), s['rows'], s['note']))
            continue
        print('%-32s %6d行 飞行%5.1fs 高度%6.1f±%-5.1f 漂移%7.1f 航向%6.1f' % (
            s['file'].replace('.log', ''), s['rows'], s['dur'],
            s['h_mean'], s['h_std'], s['drift'], s['yaw_pp']))


if __name__ == '__main__':
    main()

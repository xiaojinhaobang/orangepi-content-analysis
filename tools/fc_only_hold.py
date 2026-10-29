#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""统计"单飞控定点"段（= 空中且我们的位置环没在发指令 tvd=nan）的水平漂移。

用途：回答"只靠飞控自己的定点 + 雷达速度注入，稳不稳"。
"""
import glob
import math
import os
import re

RE_TVD = re.compile(r'tvd=\(\s*(-?[\d.]+|nan),\s*(-?[\d.]+|nan)\)')


def load(path):
    rows = []
    with open(path, errors='ignore') as fh:
        for line in fh:
            if 'psi=' not in line:
                continue
            m = re.match(r't=\s*([\d.]+)\s+x=\s*(-?[\d.]+)\s+y=\s*(-?[\d.]+)', line)
            if not m:
                continue
            t, x, y = float(m.group(1)), float(m.group(2)), float(m.group(3))
            mh = re.search(r'\bh=(-?[\d.]+)', line)
            h = float(mh.group(1)) if mh else 0.0
            mt = RE_TVD.search(line)
            cmd_on = bool(mt) and mt.group(1) != 'nan' and mt.group(2) != 'nan'
            rows.append((t, x, y, h, cmd_on))
    return rows


def main():
    print('%-26s %-6s %7s %10s %11s' % ('记录', '段', '时长s', '水平漂移cm', '速率cm/s'))
    for path in sorted(glob.glob('flight_data/console_2026*.log')):
        rows = load(path)
        segs, cur = [], []
        for r in rows:
            if r[3] > 20 and not r[4]:
                cur.append(r)
            else:
                if len(cur) > 30:            # 至少 1.5 秒
                    segs.append(cur)
                cur = []
        if len(cur) > 30:
            segs.append(cur)
        for i, s in enumerate(segs[:3]):
            dur = s[-1][0] - s[0][0]
            drift = max(math.hypot(r[1] - s[0][1], r[2] - s[0][2]) for r in s)
            print('%-26s %-6s %7.1f %10.1f %11.1f'
                  % (os.path.basename(path).replace('console_', '').replace('.log', ''),
                     '第%d段' % (i + 1), dur, drift, drift / max(dur, 0.1)))


if __name__ == '__main__':
    main()

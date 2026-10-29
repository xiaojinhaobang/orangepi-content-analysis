# -*- coding: utf-8 -*-
"""列出 fcstat CSV 里所有 cmd1=6（IMU 板跑"一键降落"）的时段，并标出后果。

用途：回答"以前的自降是什么样、悬停自测有没有这个问题"——
      把每次 cmd1=6 的持续时间、期间高度变化、之后是否上锁列成一张表。

列定义: t,h,mode_sta,unlock_sta,mode_cmd,unlock_cmd,cid,cmd0,cmd1,thr

用法: python tools/cmd1_episodes.py flight_data/2026-10-12/*.csv
"""
import csv
import glob
import os
import sys


def load(path):
    with open(path, encoding='utf-8') as fh:
        rows = [r for r in csv.reader(fh) if len(r) >= 10]
    if not rows or rows[0][0].strip() == 't':
        rows = rows[1:]
    out = []
    for r in rows:
        try:
            out.append((float(r[0]), float(r[1]), int(r[3]), int(r[8]),
                        float(r[9])))
        except ValueError:
            continue
    return out


def episodes(rows):
    """返回 [(t0, t1, dur, h0, h_end, h_min, locked_after)]"""
    eps = []
    i = 0
    n = len(rows)
    while i < n:
        if rows[i][3] != 6:
            i += 1
            continue
        j = i
        while j + 1 < n and rows[j + 1][3] == 6:
            j += 1
        t0, t1 = rows[i][0], rows[j][0]
        hs = [rows[k][1] for k in range(i, j + 1)]
        k = j + 1
        locked = 0
        while k < n and rows[k][0] - t1 <= 5.0:
            if rows[k][2] == 0:
                locked = 1
                break
            k += 1
        eps.append((t0, t1, t1 - t0, hs[0], hs[-1], min(hs), locked))
        i = j + 1
    return eps


def main():
    pats = sys.argv[1:] or ['flight_data/**/fcstat*.csv']
    paths = []
    for p in pats:
        paths.extend(sorted(glob.glob(p, recursive=True)))
    for path in paths:
        rows = load(path)
        if not rows:
            continue
        eps = episodes(rows)
        name = os.path.basename(path)
        if not eps:
            print('%-28s 无 cmd1=6' % name)
            continue
        print('=== %s  （时长 %.0fs，%d 个 cmd1=6 事件）' % (name, rows[-1][0],
                                                          len(eps)))
        for e in eps:
            kind = '瞬跳' if e[2] < 0.6 else ('短降' if e[2] < 3 else '持续降落')
            print('   t=%-8.1f 持续 %-5.2fs  %-8s h: %.0f→%.0f (min %.0f)  %s'
                  % (e[0], e[2], kind, e[3], e[4], e[5],
                     '之后上锁' if e[6] else ''))


if __name__ == '__main__':
    main()

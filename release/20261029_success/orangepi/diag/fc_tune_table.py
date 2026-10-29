#!/usr/bin/env python3
"""把"干净悬停段"汇总成 参数 -> 航向 对照表（自动带绝对时间 + 中位数）。

只统计 unlock=1 的真飞行段 —— 人拿着飞机走的那段（高度也在 1m 左右）会被排除。
每行给出绝对时间、高度、时长、前10s/后段、航向峰峰，并贴上参数标签。
最后按参数分组给中位数：**同一档要连飞 3 发，看中位数**，单发不可比。

用法:
  python3 ~/diag/fc_tune_table.py              # 扫最新 30 个日志
  python3 ~/diag/fc_tune_table.py 60           # 扫 60 个
  python3 ~/diag/fc_tune_table.py --params     # 只打印参数记录表

记参数（改完飞控参数、**起飞之前**就记一行，必须带时间，否则贴不到飞行上）:
  echo "$(date '+%Y-%m-%d %H:%M:%S') ID65=1050 PID3=1200/300/30" >> ~/diag/fc_params.txt

补记历史（比如今天这两发）:
  echo "2026-10-08 20:50:00 ID65=908 PID3=1200/100/10" >> ~/diag/fc_params.txt
"""
import datetime as dt
import glob
import math
import os
import re
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dump_flight_segments import parse                                 # noqa: E402
from hover_report import clean_h, merged_runs, seg_psi_pp, split_pp    # noqa: E402

PARAM_FILE = os.path.expanduser('~/diag/fc_params.txt')
DROP_FILE = os.path.expanduser('~/diag/fc_drop.txt')
RE_STAMP = re.compile(r'^\s*(\d{4}-\d{2}-\d{2})[ _](\d{2}:\d{2}:\d{2})\s+(.*)$')
RE_NAME = re.compile(r'(\d{4})(\d{2})(\d{2})_(\d{2})(\d{2})(\d{2})')


def load_drops():
    """fc_drop.txt：要剔除的场次，每行 `YYYY-MM-DD HH:MM[:SS]`（匹配 ±90 秒内）。"""
    out = []
    if not os.path.exists(DROP_FILE):
        return out
    for ln in open(DROP_FILE, encoding='utf-8', errors='ignore'):
        ln = ln.strip()
        if not ln or ln.startswith('#'):
            continue
        for fmt in ('%Y-%m-%d %H:%M:%S', '%Y-%m-%d %H:%M'):
            try:
                out.append(dt.datetime.strptime(ln, fmt))
                break
            except ValueError:
                continue
    return out


def load_params():
    """读参数记录。只有**带时间戳**的行能按时序贴到飞行上。"""
    out = []
    if not os.path.exists(PARAM_FILE):
        return out
    for ln in open(PARAM_FILE, encoding='utf-8', errors='ignore'):
        ln = ln.strip()
        if not ln or ln.startswith('#'):
            continue
        m = RE_STAMP.match(ln)
        if not m:
            continue                      # 老格式（没时间）没法定位，忽略
        try:
            when = dt.datetime.strptime('%s %s' % (m.group(1), m.group(2)),
                                        '%Y-%m-%d %H:%M:%S')
        except ValueError:
            continue
        out.append((when, m.group(3).strip()))
    out.sort()
    return out


def label_for(params, when, max_gap_min=45.0):
    """这场飞行用的是哪次记的参数。返回 (标签, 是否只是"沿用上一条")。

    沿用（离上一条参数记录超过 max_gap_min 分钟）的打 `*`，**不参与中位数分组**
    —— 否则"上一场改过、这场没记"会把别的档位的数值算进来。
    """
    if when is None or not params:
        return '?', False
    best = None
    for w, t in params:
        if w <= when:
            best = (w, t)
        else:
            break
    if not best:
        return '?', False
    gap_min = (when - best[0]).total_seconds() / 60.0
    return best[1], gap_min > max_gap_min


def file_start(path):
    """从 rec_YYYYMMDD_HHMMSS(.log / -N.log) 取记录开始时刻。"""
    m = RE_NAME.search(os.path.basename(path))
    if not m:
        return None
    try:
        return dt.datetime(*[int(x) for x in m.groups()])
    except ValueError:
        return None


def collect(paths):
    rows = []
    for p in paths:
        st = file_start(p)
        segs = merged_runs(clean_h([r for r in parse(p)
                                    if not math.isnan(r["x"]) and not math.isnan(r["y"])]))
        for i, seg in enumerate(segs, 1):
            hs = [r["h"] for r in seg]
            hd, tl = split_pp(seg)
            when = (st + dt.timedelta(seconds=seg[0]["t"])) if st else None
            rows.append({
                'when': when, 'file': os.path.basename(p), 'seg': i,
                't': seg[0]["t"], 'dur': seg[-1]["t"] - seg[0]["t"],
                'h': sum(hs) / len(hs), 'pp': seg_psi_pp(seg), 'head': hd, 'tail': tl,
            })
    rows.sort(key=lambda r: (r['when'] is None, r['when'] or dt.datetime.max,
                             0 if r['file'].startswith('rec_') else 1))
    return dedupe(rows)


def dedupe(rows, tol=3.0):
    """同一场飞行会被 rec_*.log 和 console_*.log 各记一份 —— 去重，优先留 rec_。"""
    out = []
    for r in rows:
        hit = None
        for o in out:
            if (r['when'] and o['when']
                    and abs((r['when'] - o['when']).total_seconds()) <= tol
                    and abs(r['h'] - o['h']) <= 5.0):
                hit = o
                break
        if hit is None:
            out.append(r)
        elif r['file'].startswith('rec_') and not hit['file'].startswith('rec_'):
            out[out.index(hit)] = r
    return out


def main():
    positional = [a for a in sys.argv[1:] if not a.startswith('--')]
    n = int(positional[0]) if positional else 30
    max_dur = 60.0
    if '--max-dur' in sys.argv:
        try:
            max_dur = float(sys.argv[sys.argv.index('--max-dur') + 1])
        except (IndexError, ValueError):
            pass
    params = load_params()

    if '--params' in sys.argv:
        if not params:
            print('（~/diag/fc_params.txt 里没有"带时间戳"的记录 —— 贴不到飞行上）')
        for w, t in params:
            print('%s  %s' % (w.strftime('%m-%d %H:%M:%S'), t))
        return 0

    cands = []
    for pat in ("rec_*.log", "console_*.log", "fc_*.log", "ourloop*.log"):
        cands += glob.glob(os.path.expanduser("~/traces/" + pat))
    cands.sort(key=os.path.getmtime, reverse=True)
    rows = collect(cands[:n])
    if not rows:
        print('没找到任何干净悬停段（unlock=1）')
        return 1
    # 悬停段一般在 10~40 秒；更长的多半是"长航段/航向漂移"，会把中位数带偏
    long_drop = [r for r in rows if r['dur'] > max_dur]
    rows = [r for r in rows if r['dur'] <= max_dur]
    if long_drop:
        print('（剔除 %d 条 >%.0f 秒的长段，避免航向漂移污染中位数）'
              % (len(long_drop), max_dur))

    drops = load_drops()
    if drops:
        kept = [r for r in rows
                if not (r['when'] and any(abs((r['when'] - d).total_seconds()) <= 90
                                          for d in drops))]
        if len(kept) != len(rows):
            print('（按 fc_drop.txt 剔除 %d 场）' % (len(rows) - len(kept)))
        rows = kept

    print('=== 干净悬停段（unlock=1；时间 = 记录开始 + t）===')
    print('%-15s %-26s %4s %7s %6s %6s %8s  %s'
          % ('时间', '文件·段', '高', '时长', '前10s', '后段', '航向峰峰', '参数'))
    for r in rows:
        w = r['when'].strftime('%m-%d %H:%M:%S') if r['when'] else '   ?（无时间） '
        lbl, inh = label_for(params, r['when'])
        print('%-15s %-26s %4.0f %6.1fs %6.1f %6.1f %8.1f  %s'
              % (w, '%s·段%d' % (r['file'][:20], r['seg']), r['h'], r['dur'],
                 r['head'], r['tail'], r['pp'], lbl + ('*' if inh else '')))

    groups = {}
    for r in rows:
        lbl, inh = label_for(params, r['when'])
        if not inh:                      # 只统计"确认过"的场次
            groups.setdefault(lbl, []).append(r['pp'])
    print('\n=== 按参数分组（同一档连飞 3 发看中位数）===')
    print('%-30s %3s %9s %9s %9s' % ('参数', 'n', '中位数', '最小', '最大'))
    for k in sorted(groups, key=lambda k: (k == '?', k)):
        v = groups[k]
        print('%-30s %3d %9.1f %9.1f %9.1f'
              % (k, len(v), statistics.median(v), min(v), max(v)))
    return 0


if __name__ == '__main__':
    sys.exit(main())

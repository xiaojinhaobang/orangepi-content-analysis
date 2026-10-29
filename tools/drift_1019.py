#!/usr/bin/env python3
"""临时诊断：解析 2026-10-19 的 rec / hover 日志，看漂移与指令的关系。

用法:
  python tools/drift_1019.py rec <log> <t0> <t1> [step_s]
  python tools/drift_1019.py hover <log> [step_s]
"""
import math
import re
import sys

NUM = r'[-+]?\d+(?:\.\d+)?'


def f(pattern, text, default=float('nan')):
    m = re.search(pattern, text)
    return float(m.group(1)) if m else default


def parse_rec(path):
    rows = []
    for line in open(path, encoding='utf-8', errors='ignore'):
        if not line.startswith('t='):
            continue
        t = f(r'^t=\s*(' + NUM + ')', line)
        rows.append({
            't': t,
            'x': f(r'\bx=\s*(' + NUM + ')', line),
            'y': f(r'\by=\s*(' + NUM + ')', line),
            'psi': f(r'\bpsi=\s*(' + NUM + ')', line),
            'tvx': f(r'tvd=\(\s*(' + NUM + ')', line),
            'tvy': f(r'tvd=\(\s*' + NUM + r',\s*(' + NUM + ')', line),
            'ftx': f(r'fcTgtVx=\s*(' + NUM + ')', line),
            'fty': f(r'fcTgtVy=\s*(' + NUM + ')', line),
            'fvx': f(r'fcVelX=\s*(' + NUM + ')', line),
            'fvy': f(r'fcVelY=\s*(' + NUM + ')', line),
            'ax': f(r'fcAccX=\s*(' + NUM + ')', line),
            'ay': f(r'fcAccY=\s*(' + NUM + ')', line),
            'rol': f(r'fcRoll=\s*(' + NUM + ')', line),
            'pit': f(r'fcPitch=\s*(' + NUM + ')', line),
            'pvx': f(r'fcTarV=\(\s*(' + NUM + ')', line),
            'pvy': f(r'fcTarV=\(\s*' + NUM + r',\s*(' + NUM + ')', line),
            'h': f(r'\bh=(' + NUM + ')', line),
            'unlock': f(r'unlock=(' + NUM + ')', line),
            'pwm': [float(v) for v in re.findall(NUM, re.search(r'fcPwm=\(([^)]*)\)', line).group(1))] if 'fcPwm=(' in line else [],
        })
    return rows


def show_rec(path, t0, t1, step):
    rows = [r for r in parse_rec(path) if t0 <= r['t'] <= t1]
    if not rows:
        print('该时间窗没有数据')
        return
    print('t      x       y      psi    tvx    tvy   fcTx  fcTy  fcVx  fcVy  fcPx  fcPy   rol   pit    h  ulk  pwm')
    next_print = t0
    for r in rows:
        if r['t'] >= next_print:
            print('%7.2f %7.1f %7.1f %6.1f %6.1f %6.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.1f %5.2f %5.2f %4.0f %3.0f  %s'
                  % (r['t'], r['x'], r['y'], r['psi'], r['tvx'], r['tvy'],
                     r['ftx'], r['fty'], r['fvx'], r['fvy'], r['pvx'], r['pvy'],
                     r['rol'], r['pit'], r['h'], r['unlock'],
                     ','.join('%.0f' % v for v in r['pwm'])))
            next_print += step
    # 汇总：每个步长窗口的位移与指令
    print('\n窗口  dt  位移   平均速度  平均|tvd|  平均|fcTgt|  最大单步')
    for i in range(0, len(rows) - 1, max(1, int(step * 10))):
        a, b = rows[i], rows[min(i + int(step * 10), len(rows) - 1)]
        dt = b['t'] - a['t']
        if dt <= 0:
            continue
        d = math.hypot(b['x'] - a['x'], b['y'] - a['y'])
        seg = rows[i:min(i + int(step * 10), len(rows) - 1) + 1]
        tv = sum(math.hypot(r['tvx'], r['tvy']) for r in seg) / len(seg)
        ft = sum(math.hypot(r['ftx'], r['fty']) for r in seg) / len(seg)
        mx = max(math.hypot(seg[k + 1]['x'] - seg[k]['x'], seg[k + 1]['y'] - seg[k]['y'])
                 for k in range(len(seg) - 1))
        print('%6.1f %4.1f %6.1f %7.2f %9.1f %11.1f %10.1f'
              % (a['t'], dt, d, d / dt, tv, ft, mx))


def show_hover(path, step):
    rows = []
    for line in open(path, encoding='utf-8', errors='ignore'):
        if not line.startswith('t='):
            continue
        rows.append((f(r'^t=\s*(' + NUM + ')', line),
                     f(r'\bx=\s*(' + NUM + ')', line),
                     f(r'\by=\s*(' + NUM + ')', line),
                     f(r'drift=\s*(' + NUM + ')', line),
                     f(r'\bh=(' + NUM + ')', line),
                     f(r'unlock=(' + NUM + ')', line),
                     f(r'raw=\(\s*(' + NUM + ')', line),
                     f(r'raw=\(' + NUM + r',\s*(' + NUM + ')', line)))
    print('t      x       y   drift     h ulk   raw_vx raw_vy  步长')
    nxt = 0.0
    max_step = 0.0
    for i, r in enumerate(rows):
        if r[0] >= nxt:
            st = math.hypot(r[1] - rows[i - 1][1], r[2] - rows[i - 1][2]) if i else 0.0
            print('%6.1f %7.1f %7.1f %6.1f %5.0f %3.0f %8.1f %6.1f %6.1f'
                  % (r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], st))
            nxt += step
        if i:
            max_step = max(max_step, math.hypot(r[1] - rows[i - 1][1], r[2] - rows[i - 1][2]))
    print('最大单样本跳变 %.1f cm' % max_step)


def scan_rec(path, win=2.0, vmin=4.0):
    """列出“SLAM 说在动、机身却是平的”窗口，用来区分真漂移和位姿漂移。"""
    rows = parse_rec(path)
    print('t0      dt   SLAM速度  平均|roll| 平均|pitch|  FC速度  |tvd|  h')
    i = 0
    while i < len(rows) - 1:
        a = rows[i]
        j = i
        while j + 1 < len(rows) and rows[j + 1]['t'] - a['t'] < win:
            j += 1
        b = rows[j]
        dt = b['t'] - a['t']
        if dt > 0 and a['h'] >= 20 and b['h'] >= 20:
            sp = math.hypot(b['x'] - a['x'], b['y'] - a['y']) / dt
            seg = rows[i:j + 1]
            roll = sum(abs(r['rol']) for r in seg) / len(seg)
            pit = sum(abs(r['pit']) for r in seg) / len(seg)
            fv = sum(math.hypot(r['fvx'], r['fvy']) for r in seg) / len(seg)
            tv = sum(math.hypot(r['tvx'], r['tvy']) for r in seg) / len(seg)
            if sp >= vmin:
                print('%7.1f %4.1f %8.1f %10.2f %10.2f %8.1f %6.1f %4.0f'
                      % (a['t'], dt, sp, roll, pit, fv, tv, a['h']))
        i = j + 1


if __name__ == '__main__':
    mode = sys.argv[1]
    if mode == 'rec':
        show_rec(sys.argv[2], float(sys.argv[3]), float(sys.argv[4]),
                 float(sys.argv[5]) if len(sys.argv) > 5 else 2.0)
    elif mode == 'scan':
        scan_rec(sys.argv[2], float(sys.argv[3]) if len(sys.argv) > 3 else 2.0,
                 float(sys.argv[4]) if len(sys.argv) > 4 else 4.0)
    else:
        show_hover(sys.argv[2], float(sys.argv[3]) if len(sys.argv) > 3 else 2.0)

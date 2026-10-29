#!/usr/bin/env python3
"""判断"慢摆"是机体真在转，还是位姿估计在摆。

原理:
  fcGyroZ = 凌霄IMU 的原始偏航角速度(LSB)。它不经过飞控、不经过 SLAM —— 独立证人。
  把 fcGyroZ 积分起来 = 一条独立的"陀螺航向"曲线，可以和 psi(SLAM航向) 直接比。

结论规则:
  psi 在某个低频上摆得很大，而陀螺航向在同频上几乎不动  -> 是位姿估计在摆
  两条曲线同频同幅                                     -> 机体真在转

用法:
  python3 analyze_fc_attitude.py <日志>                  # 自动挑最长的一段动作
  python3 analyze_fc_attitude.py <日志> 100 130          # 只看 t=100..130 秒
  python3 analyze_fc_attitude.py <日志> --scale 18.4     # 陀螺标定 LSB/(deg/s)
"""
import math
import re
import sys

# 台面标定值（2026-10-08 用 SLAM 累计转角做参照得到）
DEFAULT_SCALE = 18.4

LINE = re.compile(
    r't=\s*(?P<t>-?[\d.]+|nan).*?psi=\s*(?P<psi>-?[\d.]+|nan).*?'
    r'\sh=(?P<h>\S+)\smode=(?P<mode>\S+)\sunlock=(?P<unlock>\S+).*?'
    r'fcYaw=\s*(?P<fcy>-?[\d.]+|nan)\s+fcYawEuler=\s*(?P<fce>-?[\d.]+|nan)\s+'
    r'fcGyroZ=\s*(?P<gz>-?[\d.]+|nan)(?:\s+fcYawCmd=\s*(?P<cmd>-?[\d.]+|nan))?')


def num(s):
    return float('nan') if s == 'nan' else float(s)


def load(path):
    rows = []
    with open(path, 'r', encoding='utf-8', errors='replace') as fh:
        for ln in fh:
            m = LINE.search(ln)
            if m:
                try:
                    h = float(m.group('h'))
                except ValueError:
                    h = float('nan')
                if not math.isnan(h) and (h < 0.0 or h > 1000.0):
                    h = float('nan')      # 飞控高度偶尔跳变（实测有 5152 这种假值）
                try:
                    unlock = float(m.group('unlock'))
                except ValueError:
                    unlock = float('nan')
                rows.append((num(m.group('t')), num(m.group('psi')), num(m.group('fcy')),
                             num(m.group('fce')), num(m.group('gz')), h,
                             num(m.group('cmd')) if m.group('cmd') else float('nan'),
                             unlock))
    return rows  # t, psi, fc_yaw, fc_yaw_eu, fc_gyro_z, h, cmd_yaw_dps, unlock


def wrap180(d):
    return (d + 180.0) % 360.0 - 180.0


def unwrap(vals):
    out, prev, acc = [], None, 0.0
    for v in vals:
        if math.isnan(v):
            out.append(float('nan'))
            continue
        if prev is not None:
            acc += wrap180(v - prev)
        prev = v
        out.append(acc)
    return out


def ptp(vals):
    v = [x for x in vals if not math.isnan(x)]
    return (max(v) - min(v)) if len(v) >= 2 else float('nan')


def corr(a, b):
    pairs = [(x, y) for x, y in zip(a, b) if not math.isnan(x) and not math.isnan(y)]
    if len(pairs) < 5:
        return float('nan')
    n = len(pairs)
    ma = sum(p[0] for p in pairs) / n
    mb = sum(p[1] for p in pairs) / n
    va = sum((p[0] - ma) ** 2 for p in pairs)
    vb = sum((p[1] - mb) ** 2 for p in pairs)
    if va <= 0 or vb <= 0:
        return float('nan')
    return sum((p[0] - ma) * (p[1] - mb) for p in pairs) / math.sqrt(va * vb)


def diff_rate(t, v):
    out = [float('nan')] * len(v)
    for i in range(1, len(v)):
        dt = t[i] - t[i - 1]
        if dt > 0 and not math.isnan(v[i]) and not math.isnan(v[i - 1]):
            out[i] = (v[i] - v[i - 1]) / dt
    return out


def integrate(t, rate, scale=1.0):
    """把角速度积成角度"""
    out, acc = [], 0.0
    for i in range(len(t)):
        if i > 0 and not math.isnan(rate[i]):
            dt = t[i] - t[i - 1]
            if dt > 0:
                acc += rate[i] * dt / scale
        out.append(acc)
    return out


def detrend(t, y):
    """减去线性拟合，去掉积分漂移"""
    pairs = [(x, v) for x, v in zip(t, y) if not math.isnan(v)]
    if len(pairs) < 3:
        return list(y)
    n = len(pairs)
    mt = sum(p[0] for p in pairs) / n
    my = sum(p[1] for p in pairs) / n
    cu = sum((p[0] - mt) * (p[1] - my) for p in pairs)
    de = sum((p[0] - mt) ** 2 for p in pairs)
    k = cu / de if de > 0 else 0.0
    return [(v - (my + k * (x - mt))) if not math.isnan(v) else v
            for x, v in zip(t, y)]


def dft_scan(t, y, fmin=0.03, fmax=0.6, step=0.005):
    """扫低频，返回 (最强频率, 振幅)。最小二乘投影，采样基本均匀。"""
    pairs = [(x, v) for x, v in zip(t, y) if not math.isnan(v)]
    if len(pairs) < 20:
        return (float('nan'), float('nan'))
    n = len(pairs)
    mean = sum(p[1] for p in pairs) / n
    best = (float('nan'), -1.0)
    f = fmin
    while f <= fmax + 1e-9:
        a = b = 0.0
        for x, v in pairs:
            w = 2.0 * math.pi * f * x
            a += (v - mean) * math.sin(w)
            b += (v - mean) * math.cos(w)
        amp = 2.0 * math.hypot(a, b) / n
        if amp > best[1]:
            best = (f, amp)
        f += step
    return best


def windows(t, gz, thresh, gap=1.5):
    wins, cur = [], None
    for i, (ti, g) in enumerate(zip(t, gz)):
        if (not math.isnan(g)) and abs(g) > thresh:
            if cur is None:
                cur = [i, i]
            elif ti - t[cur[1]] > gap:
                wins.append(cur)
                cur = [i, i]
            else:
                cur[1] = i
    if cur:
        wins.append(cur)
    return wins


def main():
    args = list(sys.argv[1:])
    scale = DEFAULT_SCALE
    if '--scale' in args:
        k = args.index('--scale')
        scale = float(args[k + 1])
        del args[k:k + 2]
    if not args:
        print(__doc__)
        return 1
    rows = load(args[0])
    if not rows:
        print('没解析到任何行 —— 日志格式不对？')
        return 1

    t = [r[0] for r in rows]
    psi = [r[1] for r in rows]
    fcyaw = [r[2] for r in rows]
    gz = [r[4] for r in rows]
    hh = [r[5] for r in rows]
    cmd = [r[6] for r in rows]
    ul = [r[7] for r in rows]
    print('解析到 %d 行，t = %.1f .. %.1f s，陀螺标定 %.1f LSB/(deg/s)' %
          (len(rows), t[0], t[-1], scale))

    if len(args) >= 3:
        lo, hi = float(args[1]), float(args[2])
        idx = [i for i in range(len(rows)) if lo <= t[i] <= hi]
        tag = '指定窗口 t=%.1f..%.1f' % (lo, hi)
    else:
        good = sorted(abs(g) for g in gz if not math.isnan(g))
        if not good:
            print('fcGyroZ 全是 nan —— 记录器没收到 /fc_gyro')
            return 1
        med = good[len(good) // 2]
        idx = tag = None

        # ① 最靠谱：unlock=1（飞控已解锁）且离地的一段 —— 手拿着走/台面测试都会被排除
        armed = [i for i in range(len(rows))
                 if ul[i] == 1.0 and (not math.isnan(hh[i])) and hh[i] >= 50.0]
        if len(armed) >= 40:
            segs, cur = [], None
            for i in armed:
                if cur is None:
                    cur = [i, i]
                elif i - cur[1] <= 40:      # 允许 2 秒空洞（遥测偶尔丢）
                    cur[1] = i
                else:
                    segs.append(cur)
                    cur = [i, i]
            if cur is not None:
                segs.append(cur)
            s = max(segs, key=lambda w: w[1] - w[0])
            hm = max(hh[i] for i in range(s[0], s[1] + 1) if not math.isnan(hh[i]))
            sel = [i for i in range(s[0], s[1] + 1)
                   if (not math.isnan(hh[i])) and hh[i] >= hm * 0.85]
            if len(sel) >= 40:
                idx = sel
                tag = '飞行悬停段（unlock=1，最高 %.0f cm） t=%.1f..%.1f' % (
                    hm, t[idx[0]], t[idx[-1]])
                print('按 unlock=1 + 高度找到飞行段: t=%.1f..%.1f（%.1f 秒）' %
                      (t[idx[0]], t[idx[-1]], t[idx[-1]] - t[idx[0]]))

        # ② 退一步：没有解锁信息时，用"离地且在最高高度附近"当悬停段
        hv = [v for v in hh if not math.isnan(v)]
        if idx is None and len(hv) > 20 and max(hv) >= 50:
            segs, cur = [], None
            for i, v in enumerate(hh):
                if (not math.isnan(v)) and v >= 50:
                    cur = [i, i] if cur is None else [cur[0], i]
                elif cur is not None:
                    segs.append(cur)
                    cur = None
            if cur is not None:
                segs.append(cur)
            best = None
            for s in segs:
                hm = max(hh[i] for i in range(s[0], s[1] + 1) if not math.isnan(hh[i]))
                cut = hm * 0.85
                sel = [i for i in range(s[0], s[1] + 1)
                       if not math.isnan(hh[i]) and hh[i] >= cut]
                if sel and (best is None or len(sel) > len(best)):
                    best = sel
            if best and len(best) >= 40:
                idx = best
                seg_max = max(hh[i] for i in idx if not math.isnan(hh[i]))
                tag = '悬停段（按高度自动裁，最高 %.0f cm） t=%.1f..%.1f' % (
                    seg_max, t[idx[0]], t[idx[-1]])
                print('按高度列找到悬停段: t=%.1f..%.1f（%.1f 秒）' %
                      (t[idx[0]], t[idx[-1]], t[idx[-1]] - t[idx[0]]))

        if idx is None:      # 没有高度信息（比如手动转台面）→ 退回到"最长动作段"
            thresh = max(15.0, med * 6.0)
            wins = windows(t, gz, thresh)
            if not wins:
                print('\n整段没有超阈值的转动 —— 没飞起来/没手动转，或者 /fc_gyro 没数据。')
                return 0
            print('中位 |fcGyroZ| = %.1f，阈值 %.1f，找到 %d 段动作:' %
                  (med, thresh, len(wins)))
            for w in wins[:8]:
                print('   t=%8.1f..%-8.1f  %5.1f 秒' % (t[w[0]], t[w[1]], t[w[1]] - t[w[0]]))
            w = max(wins, key=lambda w: w[1] - w[0])   # 挑最长的一段
            idx = list(range(w[0], w[1] + 1))
            tag = '最长动作段 t=%.1f..%.1f' % (t[w[0]], t[w[1]])

    if not idx:
        print('窗口里没数据')
        return 1

    tt = [t[i] for i in idx]
    pp, yy, gg = [psi[i] for i in idx], [fcyaw[i] for i in idx], [gz[i] for i in idx]
    up, uy = unwrap(pp), unwrap(yy)
    ghead = detrend(tt, integrate(tt, gg, scale))     # 陀螺积分航向（去线性漂移）
    pd = detrend(tt, up)                              # SLAM 航向（去线性漂移）
    cc = [cmd[i] for i in idx]
    has_cmd = sum(1 for v in cc if not math.isnan(v)) > 0.5 * len(cc)
    chead = detrend(tt, integrate(tt, cc)) if has_cmd else None   # 飞控命令的航向

    print('\n=== %s（%.1f 秒，%d 点）===' % (tag, tt[-1] - tt[0], len(idx)))
    print('psi       峰峰值 = %7.1f deg' % ptp(up))
    print('fcYaw     峰峰值 = %7.1f deg' % ptp(uy))
    print('陀螺航向  峰峰值 = %7.1f deg   <- 由 fcGyroZ 积分，去线性漂移' % ptp(ghead))
    print('fcGyroZ   峰值   = %7.0f LSB  (静止中位 %.0f)' %
          (max(abs(g) for g in gg), sorted(abs(g) for g in gg)[len(gg) // 2]))
    print()
    print('相关性 fcGyroZ vs d(fcYaw)/dt = %+.2f' % corr(gg, diff_rate(tt, uy)))
    print('相关性 fcGyroZ vs d(psi)/dt   = %+.2f' % corr(gg, diff_rate(tt, pd)))

    fp, ap = dft_scan(tt, pd)
    fg, ag = dft_scan(tt, ghead)
    print('\n低频扫描（0.03-0.6 Hz）:')
    print('  psi(SLAM)  最强 %.3f Hz，振幅 %.1f deg' % (fp, ap))
    print('  陀螺航向   最强 %.3f Hz，振幅 %.1f deg' % (fg, ag))
    fc_, ac = (dft_scan(tt, chead) if has_cmd else (float('nan'), float('nan')))
    if has_cmd:
        print('  命令航向   最强 %.3f Hz，振幅 %.1f deg   <- 积分 fcYawCmd(yaw_dps)' % (fc_, ac))

    print('\n=== 结论 ===')
    if not (ap > 1.0):
        print('这段 psi 没有明显低频摆动（振幅 %.1f deg）—— 没有慢摆可判。' % ap)
    elif ag < ap / 3.0:
        print('psi 在 %.2f Hz 摆 %.1f deg，陀螺航向同频只有 %.1f deg —— '
              '**机体没转，是位姿估计在摆**。' % (fp, ap, ag))
    elif ag < ap / 1.5:
        print('psi %.1f deg vs 陀螺航向 %.1f deg（%.2f Hz）—— 陀螺只解释了不到三分之二，'
              '**部分真转、部分来自估计**，看原始波形。' % (ap, ag, fp))
    else:
        print('psi %.1f deg vs 陀螺航向 %.1f deg（%.2f Hz）—— 两条独立曲线同频同幅，'
              '**机体真在转**，走机械排查。' % (ap, ag, fp))

    if has_cmd:
        print()
        if ac < ag / 3.0:
            print('但**命令航向同频只有 %.1f deg**（实际 %.1f deg）—— 摆动不是飞控下命令的，'
                  '指向外部扰动 / 机械。' % (ac, ag))
        elif ac > ag * 1.5:
            print('而**命令航向同频 %.1f deg 比实际 %.1f deg 还大** —— 飞控在主动命令这个摆动，'
                  '查 YAW 环 / 航向估计（IMU 融合）。' % (ac, ag))
        else:
            print('而**命令航向 %.1f deg ≈ 实际 %.1f deg** —— 飞控在命令它、机体也跟上了，'
                  '查 YAW 环 / 航向估计（IMU 融合）。' % (ac, ag))

    print('\n采样（每 %d 点取一个）:' % max(1, len(idx) // 14))
    print('%8s %9s %9s %9s %9s %7s %7s' %
          ('t', 'psi', 'fcYaw', 'gyroHDG', 'fcGyroZ', 'h', 'yawCmd'))
    step = max(1, len(idx) // 14)
    for k in range(0, len(idx), step):
        print('%8.2f %9.1f %9.1f %9.1f %9.0f %7.0f %7.0f' %
              (tt[k], pp[k], yy[k], ghead[k], gg[k], hh[idx[k]], cc[k]))
    return 0


if __name__ == '__main__':
    sys.exit(main())

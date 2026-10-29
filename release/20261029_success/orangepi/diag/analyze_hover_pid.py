#!/usr/bin/env python3
"""悬停段的 PID 相位分析：偏航/水平振荡的频率、幅值、指令-响应滞后。

用法: python tools/analyze_hover_pid.py flight_data/xxx.log [h_lo h_hi]
"""
import math
import sys

from dump_flight_segments import parse, wrap_deg


def unwrap(vals):
    out, acc = [], 0.0
    for i, v in enumerate(vals):
        if i:
            acc += wrap_deg(v - vals[i - 1])
        out.append(acc)
    return out


def detrend(t, y):
    n = len(t)
    mt, my = sum(t) / n, sum(y) / n
    sxx = sum((a - mt) ** 2 for a in t)
    b = sum((a - mt) * (c - my) for a, c in zip(t, y)) / sxx if sxx else 0.0
    a = my - b * mt
    return [v - (a + b * u) for u, v in zip(t, y)]


def crossings(t, y):
    """返回过均值次数与平均频率。"""
    n = 0
    for i in range(1, len(y)):
        if y[i - 1] * y[i] < 0:
            n += 1
    dur = t[-1] - t[0]
    return n, (n / 2.0 / dur if dur > 0 else float("nan"))


def xcorr_lag(t, cmd, resp, max_lag=2.0):
    """找让 corr(cmd(t), resp(t+lag)) 最大的 lag（秒）。lag>0 表示响应滞后指令。"""
    dt = (t[-1] - t[0]) / max(len(t) - 1, 1)
    best, best_lag = -2.0, float("nan")
    steps = int(max_lag / max(dt, 1e-6))
    for k in range(0, steps + 1):
        a = cmd[: len(cmd) - k] if k else cmd
        b = resp[k:]
        if len(a) < 20:
            break
        ma, mb = sum(a) / len(a), sum(b) / len(b)
        sa = sum((v - ma) ** 2 for v in a) ** 0.5
        sb = sum((v - mb) ** 2 for v in b) ** 0.5
        if sa < 1e-9 or sb < 1e-9:
            continue
        c = sum((u - ma) * (v - mb) for u, v in zip(a, b)) / (sa * sb)
        if c > best:
            best, best_lag = c, k * dt
    return best_lag, best


def main():
    path = sys.argv[1]
    lo = float(sys.argv[2]) if len(sys.argv) > 2 else 108.0
    hi = float(sys.argv[3]) if len(sys.argv) > 3 else 130.0
    rows = [r for r in parse(path)
            if not math.isnan(r["x"]) and not math.isnan(r["h"]) and lo <= r["h"] <= hi]
    if len(rows) < 50:
        print("窗口样本太少：%d" % len(rows))
        return
    t = [r["t"] - rows[0]["t"] for r in rows]
    dur = t[-1]
    print("窗口 %.1fs  %d 样本（高度 %.0f~%.0fcm）" % (dur, len(rows), lo, hi))

    # 位置残余（去掉整体线性漂移）
    for name, key in (("x", "x"), ("y", "y")):
        v = [r[key] for r in rows]
        res = detrend(t, v)
        n, f = crossings(t, res)
        print("  %s: 峰峰 %5.1fcm  过零 %2d 次(%.2fHz)  线性漂移 %+.1fcm"
              % (name, max(res) - min(res), n, f, v[-1] - v[0]))

    # 偏航
    psi = unwrap([r["psi"] for r in rows])
    res = detrend(t, psi)
    n, f = crossings(t, res)
    print("  psi: 峰峰 %5.1f°  过零 %2d 次(%.2fHz)  线性漂移 %+.1f°"
          % (max(res) - min(res), n, f, psi[-1] - psi[0]))

    # 偏航指令 vs 实际角速度
    rate = []
    for i in range(1, len(psi)):
        dt = t[i] - t[i - 1]
        rate.append((psi[i] - psi[i - 1]) / dt if dt > 1e-6 else 0.0)
    cmd = [r["yawv"] for r in rows[1:]]
    tt = t[1:]
    lag, c = xcorr_lag(tt, cmd, rate)
    print("  偏航: 指令 %+.2f±%.2f deg/s   实际 %+.2f±%.2f deg/s   响应滞后 %.2fs (corr %.2f)"
          % (sum(cmd) / len(cmd),
             (sum((v - sum(cmd) / len(cmd)) ** 2 for v in cmd) / len(cmd)) ** 0.5,
             sum(rate) / len(rate),
             (sum((v - sum(rate) / len(rate)) ** 2 for v in rate) / len(rate)) ** 0.5,
             lag, c))

    # 水平指令 vs 实际速度（导航系，按 psi 旋转 tvd）
    vx, vy = [], []
    for i in range(1, len(rows)):
        dt = t[i] - t[i - 1]
        vx.append((rows[i]["x"] - rows[i - 1]["x"]) / dt if dt > 1e-6 else 0.0)
        vy.append((rows[i]["y"] - rows[i - 1]["y"]) / dt if dt > 1e-6 else 0.0)
    cx, cy = [], []
    for r in rows[1:]:
        c_, s_ = math.cos(math.radians(r["psi"])), math.sin(math.radians(r["psi"]))
        cx.append(r["tvx"] * c_ - r["tvy"] * s_)      # 机体系 -> 导航系
        cy.append(r["tvx"] * s_ + r["tvy"] * c_)
    for nm, cv, rv in (("vx", cx, vx), ("vy", cy, vy)):
        lag, c = xcorr_lag(tt, cv, rv)
        mc, mr = sum(cv) / len(cv), sum(rv) / len(rv)
        print("  %s: 指令 %+.2f  实测 %+.2f cm/s  响应滞后 %.2fs (corr %.2f)"
              % (nm, mc, mr, lag, c))


if __name__ == "__main__":
    main()

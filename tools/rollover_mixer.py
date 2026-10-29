# -*- coding: utf-8 -*-
"""Resolve the FC mixer into collective + roll/pitch/yaw demand per unlock window.

Motor layout per 飞控手册: 1=右前 2=左前 3=左后 4=右后 (从右前逆时针 1,2,3,4);
/fc_pwm 的第 i 个元素 = 第 (i+1) 号电机, 单位 %.

    C      = mean(4)                    集体油门 (推力代理量)
    roll   = ((m2+m3) - (m1+m4)) / 4    右滚为正
    pitch  = ((m1+m2) - (m3+m4)) / 4    低头为正
    yaw    = ((m1+m3) - (m2+m4)) / 4    偏航

Usage: python tools/rollover_mixer.py <log> [<log> ...] [--step 10]
"""
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze_rollover import parse, find_events  # noqa: E402


def pwm4(r):
    p = r.get("pwm") or []
    if len(p) < 4 or any(v != v for v in p):
        return None
    if max(p) <= 0:          # 上锁/未输出
        return None
    return p


def demand(p):
    m1, m2, m3, m4 = p
    return (
        (m1 + m2 + m3 + m4) / 4.0,
        ((m2 + m3) - (m1 + m4)) / 4.0,
        ((m1 + m2) - (m3 + m4)) / 4.0,
        ((m1 + m3) - (m2 + m4)) / 4.0,
    )


def run(path, step):
    rows = parse(path)
    print(f"\n===== {os.path.basename(path)}")
    for a, b in find_events(rows):
        win = [r for r in rows if a - 0.3 <= r["t"] <= b + 0.3]
        live = [r for r in win if pwm4(r)]
        if not live:
            print(f"  unlock {a:.2f}..{b:.2f}: 电机未升速 (pwm 恒 0/18)")
            continue
        t_first = live[0]["t"]
        t_last = live[-1]["t"]
        hmax = max((r["h"] for r in live if r["h"] == r["h"]), default=float("nan"))
        print(
            f"  unlock {a:.2f}..{b:.2f}  电机升速 {t_first:.2f}..{t_last:.2f}"
            f"  h_max={hmax:.0f}cm"
        )
        print(
            "      t   |  C   roll  pitch  yaw | meas_roll meas_pitch | gXmean gYmean | h"
        )
        idx = list(range(0, len(live), step))
        if idx[-1] != len(live) - 1:
            idx.append(len(live) - 1)
        for i in idx:
            r = live[i]
            C, ro, pi, ya = demand(pwm4(r))
            lo = max(0, i - step // 2)
            hi = min(len(live), i + step // 2 + 1)
            seg = live[lo:hi]
            gx = [x for x in (rr["gyroXY"][0] if rr["gyroXY"] else float("nan") for rr in seg)]
            gy = [x for x in (rr["gyroXY"][1] if len(rr["gyroXY"]) > 1 else float("nan") for rr in seg)]
            gxm = sum(v for v in gx if v == v) / max(1, sum(1 for v in gx if v == v))
            gym = sum(v for v in gy if v == v) / max(1, sum(1 for v in gy if v == v))
            print(
                f"{r['t']:9.2f} |{C:5.1f}{ro:6.1f}{pi:6.1f}{ya:6.1f} |"
                f"{r['roll']:9.2f}{r['pitch']:11.2f} |{gxm:6.0f}{gym:6.0f} |{r['h']:3.0f}"
            )


def main():
    files = [a for a in sys.argv[1:] if not a.startswith("--")]
    step = 10
    if "--step" in sys.argv:
        step = int(sys.argv[sys.argv.index("--step") + 1])
    for f in files:
        run(f, step)
    return 0


if __name__ == "__main__":
    sys.exit(main())

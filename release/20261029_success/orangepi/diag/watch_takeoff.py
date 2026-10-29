# -*- coding: utf-8 -*-
"""打印最近一次"电机升速"窗口的油门/高度/命令回显，用来判定起飞有没有被受理。

读 /tmp/fcstat_pwm.csv（fcstat_log.py 写），列:
    t,h,mode_sta,unlock_sta,mode_cmd,unlock_cmd,cid,cmd0,cmd1,thr,ms,pwm,pwm0,pwm1,pwm2,pwm3

判据：
    mean(pwm0..3) 一路爬到 60%+（拆桨会到 90%+）  → 飞控进了"一键起飞爬升"
    停在 40~43% 不动                              → 飞控留在定点悬停档，起飞没被执行
    ms   = F407 收到的最后一条 0x66 子命令（1=起飞 / 2=降落 / 4=锁定）
    cmd1 = IMU 核回显的最后一条 CMD[1]（5=起飞 / 4=锁定 / 6=降落）

用法: python3 watch_takeoff.py [文件] [窗口数,默认1]
"""
import sys

PATH = sys.argv[1] if len(sys.argv) > 1 else "/tmp/fcstat_pwm.csv"
NWIN = int(sys.argv[2]) if len(sys.argv) > 2 else 1


def f(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


rows = []
with open(PATH, "r", errors="replace") as fh:
    for ln in fh:
        p = ln.rstrip("\n").split(",")
        if len(p) < 16 or not f(p[0]):
            continue
        rows.append([f(x) for x in p])

if not rows:
    print("没有数据")
    raise SystemExit(1)

# 找"电机升速"的连续窗口
wins = []
cur = None
for r in rows:
    live = all(v is not None for v in r[12:16]) and max(r[12:16]) > 20
    if live and cur is None:
        cur = [r]
    elif live:
        cur.append(r)
    elif cur is not None:
        if len(cur) >= 10:
            wins.append(cur)
        cur = None
if cur is not None and len(cur) >= 10:
    wins.append(cur)

if not wins:
    print("最近没有电机升速的窗口（只有怠速 18% 或全部为 0）")
    raise SystemExit(0)

for w in wins[-NWIN:]:
    print(
        f"=== 窗口 t={w[0][0]:.2f}..{w[-1][0]:.2f}（{w[-1][0] - w[0][0]:.1f}s，"
        f"{len(w)} 帧）"
    )
    print("     t | h  unl | thr | ms cmd1 cid | pwm0 pwm1 pwm2 pwm3 | 均值 极差")
    step = max(1, len(w) // 40)
    for r in w[::step]:
        pm = [v for v in r[12:16]]
        mean = sum(pm) / 4.0
        spread = max(pm) - min(pm)
        print(
            f"{r[0]:7.2f} |{r[1]:3.0f} {r[3]:3.0f} |{r[9]:4.0f} |"
            f"{r[10]:2.0f} {r[11]:4.0f} {r[6]:3.0f} |"
            f"{pm[0]:5.1f}{pm[1]:5.1f}{pm[2]:5.1f}{pm[3]:5.1f} |{mean:5.1f}{spread:5.1f}"
        )
    mmax = max(sum(r[12:16]) / 4.0 for r in w)
    rmax = max(max(r[12:16]) - min(r[12:16]) for r in w)
    hmax = max(r[1] for r in w)
    ms = w[0][10]
    cmd1 = w[0][11]
    print(
        f"--- 汇总: 集体油门峰值 {mmax:.1f}%  最大极差 {rmax:.1f}%   h_max={hmax:.0f}cm"
        f"   ms={ms:.0f} cmd1={cmd1:.0f}"
    )
    if mmax >= 55:
        print("    → 飞控进了爬升档（拆桨会顶到 90%+）：起飞命令被执行")
    elif mmax >= 30:
        print("    → 停在 40% 上下：飞控留在地面保持档，起飞没被执行")
print()
print(f"（文件 {PATH}，共 {len(rows)} 行，t 最大 {rows[-1][0]:.1f}s）")

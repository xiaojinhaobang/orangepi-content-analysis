#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""offline_heading_fuse — 离线验证「雷达 + IMU(陀螺) 航向融合」能否压住雷达航向的跳变/卡顿。

背景（2026-10-16 实测）：
  · 起飞点悬停时雷达航向 1 秒内跳 42°（4.6°→47°→60°），同一时刻飞控陀螺显示飞机只平滑转 −3.7°/s
  · 手转 180° 时雷达航向卡住 5~6 秒不动，飞控陀螺两次给出 196.9°/198.4°（一致）
  → 结论：那个点上，雷达航向不可信，陀螺可信；而我们的偏航环拿雷达航向当真值去追 → 自转 + 漂移

融合原理（本脚本离线实现，验证通过后才上机）：
  d          = 雷达航向 − 飞控航向      （零点偏差，物理上缓慢变化）
  psi_fused  = fcYaw + d                （实时航向：陀螺提供动态，雷达提供长期零点）
  仅当 |innovation| = |wrap(psi_radar − psi_fused)| < GATE 时，才允许 d 慢速更新；
  超出 GATE（跳变/卡顿）→ 冻结 d，该帧的雷达航向被拒绝。

判据（最要紧的一条）：
  在跳变窗口里，用「原始雷达航向」算出的偏航误差 vs 用「融合航向」算出的偏航误差。
  原始会顶到 20~40°+（我们就是这么被带偏的），融合应该只有几度。

用法：python3 tools/offline_heading_fuse.py flight_data/offline/fz_*.txt
"""
import glob
import math
import os
import re
import sys

RE_T = re.compile(r"^t=\s*([-\d.]+)")
RE_PSI = re.compile(r"psi=\s*([-\d.]+)")
RE_FY = re.compile(r"fcYaw=\s*([-\d.]+)")
RE_GZ = re.compile(r"fcGyroZ=\s*([-\d.]+)")
RE_H = re.compile(r"h=(\d+)")
RE_UN = re.compile(r"unlock=(\d)")
RE_X = re.compile(r"x=\s*([-\d.]+)")
RE_Y = re.compile(r"y=\s*([-\d.]+)")

GATE = float(os.environ.get("FUSE_GATE", "8.0"))       # deg，超过即认为是跳变
K = float(os.environ.get("FUSE_K", "0.05"))            # 1/s，d 的慢速更新增益（约 20s 时间常数）
MAX_RATE = float(os.environ.get("FUSE_MAX_RATE", "0.5"))  # deg/s，d 的更新速率上限
GZ_SCALE = float(os.environ.get("FUSE_GZ_SCALE", "16.0"))  # LSB per deg/s（实测约 16）
STILL_DPS = float(os.environ.get("FUSE_STILL_DPS", "1.0"))  # deg/s，小于此视为"飞机没在转"
REACQ_S = float(os.environ.get("FUSE_REACQ_S", "3.0"))      # s，超门限持续多久后允许重新捕获


def wrap(d):
    while d > 180.0:
        d -= 360.0
    while d < -180.0:
        d += 360.0
    return d


def parse(path):
    rows = []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            if not line.startswith("t="):
                continue
            mt, mp, mf = RE_T.search(line), RE_PSI.search(line), RE_FY.search(line)
            if not (mt and mp and mf):
                continue
            mh, mu, mg = RE_H.search(line), RE_UN.search(line), RE_GZ.search(line)
            rows.append({
                "t": float(mt.group(1)),
                "psi": float(mp.group(1)),
                "fcy": float(mf.group(1)),
                "gz": float(mg.group(1)) if mg else float("nan"),
                "h": int(mh.group(1)) if mh else -1,
                "un": int(mu.group(1)) if mu else -1,
                "x": float(RE_X.search(line).group(1)) if RE_X.search(line) else float("nan"),
                "y": float(RE_Y.search(line).group(1)) if RE_Y.search(line) else float("nan"),
            })
    return rows


def fuse(rows):
    d = None
    prev_t = None
    rej = 0
    reacq = 0
    bad_run = 0.0
    still_run = 0.0
    for r in rows:
        if d is None:
            d = wrap(r["psi"] - r["fcy"])
        fused = wrap(r["fcy"] + d)
        innov = wrap(r["psi"] - fused)
        dt = 0.05 if prev_t is None else max(1e-3, min(0.5, r["t"] - prev_t))
        gz_dps = r["gz"] / GZ_SCALE if r["gz"] == r["gz"] else 0.0
        air = (r["un"] == 1 and r["h"] >= 60)     # 在空中：d 一律冻结，绝不重新捕获
        still_run = still_run + dt if abs(gz_dps) < STILL_DPS else 0.0
        if abs(innov) < GATE:
            step = K * innov * dt
            step = max(-MAX_RATE * dt, min(MAX_RATE * dt, step))
            d = wrap(d + step)
            r["acc"] = 1
            bad_run = 0.0
        elif (not air) and bad_run >= REACQ_S and still_run >= REACQ_S:
            # 超门限很久 + 陀螺说飞机没在转 → 雷达此时是可信的绝对参考，重新锁定 d
            d = wrap(r["psi"] - r["fcy"])
            r["acc"] = 1
            r["reacq"] = 1
            reacq += 1
            bad_run = 0.0
            still_run = 0.0
        else:
            rej += 1
            r["acc"] = 0
            bad_run += dt
        r["d"] = d
        r["psi_fused"] = wrap(r["fcy"] + d)
        r["innov"] = innov
        prev_t = r["t"]
    return rej, reacq


def hover_windows(rows):
    """空中窗口：unlock=1 且 h>=100（悬停高度）"""
    wins = []
    cur = []
    for r in rows:
        if r["un"] == 1 and r["h"] >= 100:
            cur.append(r)
        else:
            if len(cur) >= 40:
                wins.append(cur)
            cur = []
    if len(cur) >= 40:
        wins.append(cur)
    return wins


def report(path, rows, rej, reacq):
    print("=" * 78)
    print("%s   样本 %d" % (os.path.basename(path), len(rows)))
    if not rows:
        return
    ds = [r["d"] for r in rows]
    print("  d(雷达−飞控) 起点 %+.1f°  终点 %+.1f°  范围 %+.1f° ~ %+.1f°   "
          "被拒绝帧 %d (%.1f%%)  重新捕获 %d 次"
          % (wrap(ds[0]), wrap(ds[-1]), min(ds), max(ds), rej,
             100.0 * rej / len(rows), reacq))

    wins = hover_windows(rows)
    if not wins:
        print("  （没有 unlock=1 且 h>=100 的窗口）")
    for w in wins:
        # 关键判据：雷达航向相对"融合真值(陀螺)"的最大偏离 = max|innov|
        dev = max(abs(r["innov"]) for r in w)
        ref = w[0]["psi"]
        e_raw = max(abs(wrap(r["psi"] - ref)) for r in w)
        e_fus = max(abs(wrap(r["psi_fused"] - ref)) for r in w)
        rj = sum(1 for r in w if not r["acc"])
        dur = w[-1]["t"] - w[0]["t"]
        print("\n  ── 悬停窗口 %.1fs（%d 帧，拒绝 %d 帧）" % (dur, len(w), rj))
        print("     ★ 雷达相对融合真值的最大偏离 %6.1f°   （空地正常时应 <3°）" % dev)
        print("     参考：窗口内航向变化量 原始雷达 %6.1f°  vs  融合 %6.1f°" % (e_raw, e_fus))
        # 位置跳变检查：决定第 2 层（位置校验门）需不需要
        spikes = 0
        mx = 0.0
        for i in range(1, len(w)):
            dt = w[i]["t"] - w[i - 1]["t"]
            if dt <= 0 or w[i]["x"] != w[i]["x"]:
                continue
            v = math.hypot(w[i]["x"] - w[i - 1]["x"], w[i]["y"] - w[i - 1]["y"]) / dt
            mx = max(mx, v)
            if v > 100.0:
                spikes += 1
        print("     位置跳变(>100cm/s)：%d 帧，峰值 %6.0f cm/s   ← 第2层需不需要看这个"
              % (spikes, mx))
        print("     t        psi_radar   fcYaw    psi_fused    d      innov   ")
        step = max(1, len(w) // 12)
        for r in w[::step]:
            print("     %7.1f  %+9.1f  %+8.1f  %+10.1f  %+7.1f  %+7.1f %s"
                  % (r["t"], r["psi"], r["fcy"], r["psi_fused"], r["d"], r["innov"],
                     "" if r["acc"] else "  <-拒绝"))


def main():
    paths = sys.argv[1:] or sorted(glob.glob("flight_data/offline/fz_*.txt"))
    print("GATE=%.1f°  K=%.2f/s  MAX_RATE=%.1f°/s" % (GATE, K, MAX_RATE))
    for p in paths:
        rows = parse(p)
        if not rows:
            print("== %s 无数据 ==" % p)
            continue
        rej, reacq = fuse(rows)
        report(p, rows, rej, reacq)


if __name__ == "__main__":
    main()

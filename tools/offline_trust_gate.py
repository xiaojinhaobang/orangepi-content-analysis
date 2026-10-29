#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""offline_trust_gate — 2a 的离线回放：用已有飞行日志判断「位姿可信度门控」什么时候会触发。

这份脚本就是"影子模式"的离线版：只做判断、输出触发时刻与占比，不改任何控制。

可用判据（20Hz rec 日志能算的两条）：
  D1 速度可实现性：|v_雷达| > |v_指令| + V_MARGIN 持续 V_HOLD 秒
      （飞机不可能比我们发的指令快；实测起点失控时雷达报 42cm/s，我们最多发 12）
  D2 位置可实现性：|p_雷达(t) − p_雷达(0)| > ∫|v_指令|dt + P_MARGIN
      （雷达报的位移，超出"按指令应该走的路"太多 → 位姿不可信）

注：振动判据（IMU 高频 RMS > 1.6×基线）需要 200Hz 采样，rec 日志只有 20Hz，
    离线无法回放；上机时用 /fc_gyro 实时算。

用法：python3 tools/offline_trust_gate.py
环境变量：V_MARGIN=10(cm/s) V_HOLD=0.3(s) P_MARGIN=12(cm) P_HOLD=0.3(s)
"""
import glob
import math
import os
import re
import sys

RE_T = re.compile(r"^t=\s*([-\d.]+)")
RE_X = re.compile(r"x=\s*([-\d.]+)")
RE_Y = re.compile(r"y=\s*([-\d.]+)")
RE_TVD = re.compile(r"tvd=\(\s*([-\d.]+),\s*([-\d.]+)\)")
RE_H = re.compile(r"h=(\d+)")
RE_UN = re.compile(r"unlock=(\d)")

V_MARGIN = float(os.environ.get("V_MARGIN", "10.0"))
V_HOLD = float(os.environ.get("V_HOLD", "0.3"))
P_MARGIN = float(os.environ.get("P_MARGIN", "12.0"))
P_HOLD = float(os.environ.get("P_HOLD", "0.3"))


def parse(path):
    rows = []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            if not line.startswith("t="):
                continue
            m_t, m_x, m_y = RE_T.search(line), RE_X.search(line), RE_Y.search(line)
            if not (m_t and m_x and m_y):
                continue
            m_v, m_h, m_u = RE_TVD.search(line), RE_H.search(line), RE_UN.search(line)
            rows.append({
                "t": float(m_t.group(1)), "x": float(m_x.group(1)), "y": float(m_y.group(1)),
                "vx": float(m_v.group(1)) if m_v else 0.0,
                "vy": float(m_v.group(2)) if m_v else 0.0,
                "h": int(m_h.group(1)) if m_h else -1,
                "un": int(m_u.group(1)) if m_u else -1,
            })
    return rows


def windows(rows, min_n=40):
    out, cur = [], []
    for r in rows:
        if r["un"] == 1 and r["h"] >= 100:
            cur.append(r)
        else:
            if len(cur) >= min_n:
                out.append(cur)
            cur = []
    if len(cur) >= min_n:
        out.append(cur)
    return out


def replay(w):
    """返回 (首触发信息, 判不可信帧占比, 最大横移, 帧数)"""
    x0, y0 = w[0]["x"], w[0]["y"]
    path_cmd = 0.0           # ∫|v_cmd|dt
    v_bad = 0.0
    p_bad = 0.0
    bad_frames = 0
    first = None
    raw_max = 0.0
    for i, r in enumerate(w):
        dist = math.hypot(r["x"] - x0, r["y"] - y0)
        raw_max = max(raw_max, dist)
        vcmd = math.hypot(r["vx"], r["vy"])
        if i > 0:
            p = w[i - 1]
            dt = max(1e-3, min(0.5, r["t"] - p["t"]))
            v_slam = math.hypot(r["x"] - p["x"], r["y"] - p["y"]) / dt
            path_cmd += vcmd * dt
            # D1
            v_bad = v_bad + dt if v_slam > vcmd + V_MARGIN else 0.0
            # D2
            p_bad = p_bad + dt if dist > path_cmd + P_MARGIN else 0.0
            if v_bad >= V_HOLD or p_bad >= P_HOLD:
                why = "速度" if v_bad >= V_HOLD else "位置"
                if first is None:
                    first = (r["t"] - w[0]["t"], dist, why)
                bad_frames += 1
                v_bad = p_bad = 0.0
    return first, bad_frames, raw_max, len(w)


def main():
    paths = sorted(glob.glob("flight_data/offline/pg_*.txt")) + \
            sorted(glob.glob("flight_data/offline/fz_*.txt"))
    if len(sys.argv) > 1:
        paths = sys.argv[1:]
    print("判据: D1 |v_雷达|>|v_指令|+%.0fcm/s 持续%.1fs | D2 里程超出指令路径+%.0fcm 持续%.1fs"
          % (V_MARGIN, V_HOLD, P_MARGIN, P_HOLD))
    for p in paths:
        rows = parse(p)
        ws = windows(rows)
        name = os.path.basename(p)
        if not ws:
            print("  %-34s (无悬停窗口)" % name)
            continue
        for w in ws:
            first, bad, raw_max, n = replay(w)
            if first:
                print("  %-30s %.1fs窗口: **首次触发 %.1fs（横移%.1fcm，判据=%s）**  不可信占比 %.0f%%  最大横移 %.1fcm"
                      % (name, w[-1]["t"] - w[0]["t"], first[0], first[1], first[2],
                         100.0 * bad / n, raw_max))
            else:
                print("  %-30s %.1fs窗口: 从未触发  最大横移 %.1fcm  ✓"
                      % (name, w[-1]["t"] - w[0]["t"], raw_max))


if __name__ == "__main__":
    main()

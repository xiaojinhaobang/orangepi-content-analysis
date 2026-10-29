#!/usr/bin/env python3
"""悬停调参一条命令出报告。

自动取最新的 ~/traces/rec_*.log，抠出悬停段（高度在窗口内），
打印 高度/水平/航向/响应滞后 四类指标，并与 2026-10-06 定稿基线对比。

用法:
    python3 ~/diag/hover_report.py                # 最新的 rec_*.log / fc_*.log
    python3 ~/diag/hover_report.py <log>           # 指定文件（自动识别悬停段）
    python3 ~/diag/hover_report.py <log> lo hi     # 强制用固定高度窗口
"""
import glob
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dump_flight_segments import parse, wrap_deg               # noqa: E402
from analyze_hover_pid import detrend, crossings, xcorr_lag    # noqa: E402

# ---- 判定参考值 ----
# ★ 2026-10-07 重定：原来照 10-06 文档设的"航向峰峰 <=5.5°"是错的。
#   把历史记录全查了一遍（含位姿冻结检测）后，凡是**位姿正常**的记录：
#     9/28 = 24.1° / 24.0°、9/30 = 23.9°、10-01 09:52 = 14.9°、今天 = 11.4~25.7°
#   而那些"0.0°/3.8°"的漂亮数字，全部出在位姿冻结（x/y/psi 纹丝不动）的数据上。
#   结论：这台机器的真实量级是 11~24°，今天最好的 11.4° 就是历史最好。
#   所以下面的阈值按"实测真值"重设，不要再拿 5.5° 当靶子。
BASE = {
    "h_sigma": 3.5,        # cm，高度标准差
    "xy_pp": 25.0,         # cm，水平峰峰（纯飞控本来就会漂；我们环参与时应 <=8）
    "xy_drift": 40.0,      # cm，首末线性漂移（同上）
    "psi_pp": 12.0,        # deg，航向峰峰 —— 历史真值 11~24°，目标先做到 <=12
    "psi_drift": 8.0,      # deg，航向线性漂移（实测 1.6~8.1）
    "yaw_lag": 0.80,       # s，偏航响应滞后（只在我们环参与时有意义）
    "freq": 2.0,           # Hz，主频上限（10-01 定的"无 >2Hz 高频抖"）
    "yaw_jitter": 1.6,     # deg/s，偏航高频抖（>2Hz 成分）——实测真值 0.91~1.52
}


def unwrap(vals):
    out, acc = [], 0.0
    for i, v in enumerate(vals):
        if i:
            acc += wrap_deg(v - vals[i - 1])
        out.append(acc)
    return out


def clean_h(rows):
    """把 /height 的串口野值清成 nan：>500cm 丢、单次跳变 >100cm 丢。

    ★ 2026-10-07 踩坑：实测 t=6765.42 出现 h=3720 的单点野值，
      它让"高度变化率"瞬间变成 3000cm/s，把 20 秒的悬停硬生生劈成两段。
      /height 是 20Hz 的，真实变化每帧不到 1cm，>100cm 的跳变一定是野值。
    """
    out, last = [], None
    for r in rows:
        h = r["h"]
        if not math.isnan(h):
            if h < 0 or h > 500 or (last is not None and abs(h - last) > 100):
                h = math.nan
            else:
                last = h
        rr = dict(r)
        rr["h"] = h
        out.append(rr)
    return out


def longest_run(rows, lo, hi, min_len=5.0):
    """最长的、高度落在 [lo,hi] 的连续段。"""
    best, cur = None, []
    for r in rows:
        h = r["h"]
        ok = (not math.isnan(h)) and lo <= h <= hi
        if ok:
            cur.append(r)
        else:
            if cur and (cur[-1]["t"] - cur[0]["t"]) >= min_len and (best is None or len(cur) > len(best)):
                best = cur
            cur = []
    if cur and (cur[-1]["t"] - cur[0]["t"]) >= min_len and (best is None or len(cur) > len(best)):
        best = cur
    return best


def longest_steady(rows, min_len=5.0, rate_max=10.0, h_min=60.0, base_sec=1.0):
    """自动找悬停段：离地 >h_min，且高度变化率 <rate_max cm/s 的最长连续段。

    变化率用 **1 秒基线** 算（h(t) vs h(t-1s)）——记录是 20Hz，相邻两帧的
    高度噪声就有 ±2cm，逐帧算会得到 40cm/s 的假变化率。
    这样不用写死高度窗口：150cm / 118cm、纯飞控 / 我们环，都能自动认出来。
    """
    best, cur, hist = None, [], []
    for r in rows:
        h = r["h"]
        if math.isnan(h):
            hist = []
            if cur and (cur[-1]["t"] - cur[0]["t"]) >= min_len and (best is None or len(cur) > len(best)):
                best = cur
            cur = []
            continue
        while hist and r["t"] - hist[0][0] > base_sec * 2.5:
            hist.pop(0)
        ref = None
        for tt, hh in reversed(hist):
            if r["t"] - tt >= base_sec:
                ref = (tt, hh)
                break
        rate = 999.0
        if ref is not None:
            rate = abs(h - ref[1]) / (r["t"] - ref[0])
        if h > h_min and rate <= rate_max:
            cur.append(r)
        else:
            if cur and (cur[-1]["t"] - cur[0]["t"]) >= min_len and (best is None or len(cur) > len(best)):
                best = cur
            cur = []
        hist.append((r["t"], h))
    if cur and (cur[-1]["t"] - cur[0]["t"]) >= min_len and (best is None or len(cur) > len(best)):
        best = cur
    return best


def longest_still(rows, min_len=5.0, speed_max=6.0, base_sec=1.0):
    """没有高度数据时的替代判据：水平几乎不动的连续段 = 悬停。

    ★ 2026-10-07：飞控-香橙派串口线被拔掉后，记录里 h/mode/unlock 全是 None，
      但位姿来自我们自己的 SLAM，照样有效 —— 这时就靠"1 秒内位移 <6cm"找悬停段。
    """
    best, cur, hist = None, [], []
    for r in rows:
        x, y = r["x"], r["y"]
        if math.isnan(x) or math.isnan(y):
            hist = []
            if cur and (cur[-1]["t"] - cur[0]["t"]) >= min_len and (best is None or len(cur) > len(best)):
                best = cur
            cur = []
            continue
        while hist and r["t"] - hist[0][0] > base_sec * 2.5:
            hist.pop(0)
        ref = None
        for tt, xx, yy in reversed(hist):
            if r["t"] - tt >= base_sec:
                ref = (tt, xx, yy)
                break
        spd = 999.0
        if ref is not None:
            spd = math.hypot(x - ref[1], y - ref[2]) / (r["t"] - ref[0])
        if spd <= speed_max:
            cur.append(r)
        else:
            if cur and (cur[-1]["t"] - cur[0]["t"]) >= min_len and (best is None or len(cur) > len(best)):
                best = cur
            cur = []
        hist.append((r["t"], x, y))
    if cur and (cur[-1]["t"] - cur[0]["t"]) >= min_len and (best is None or len(cur) > len(best)):
        best = cur
    return best


def all_steady(rows, min_len=5.0, rate_max=10.0, h_min=60.0, base_sec=1.0):
    """所有"离地且高度稳定"的连续段（含长度/均高），用于 --scan 找场次。"""
    runs, cur, hist = [], [], []
    for r in rows:
        h = r["h"]
        if math.isnan(h):
            hist = []
            if cur and (cur[-1]["t"] - cur[0]["t"]) >= min_len:
                runs.append(cur)
            cur = []
            continue
        while hist and r["t"] - hist[0][0] > base_sec * 2.5:
            hist.pop(0)
        ref = None
        for tt, hh in reversed(hist):
            if r["t"] - tt >= base_sec:
                ref = (tt, hh)
                break
        rate = 999.0
        if ref is not None:
            rate = abs(h - ref[1]) / (r["t"] - ref[0])
        if h > h_min and rate <= rate_max:
            cur.append(r)
        else:
            if cur and (cur[-1]["t"] - cur[0]["t"]) >= min_len:
                runs.append(cur)
            cur = []
        hist.append((r["t"], h))
    if cur and (cur[-1]["t"] - cur[0]["t"]) >= min_len:
        runs.append(cur)
    return runs


def mark(value, limit):
    if value is None or math.isnan(value):
        return "?"
    return "OK" if abs(value) <= limit else "差"


def seg_psi_pp(rows):
    """一段里的航向峰峰（去趋势）。"""
    if len(rows) < 20:
        return float('nan')
    tt = [r["t"] - rows[0]["t"] for r in rows]
    res = detrend(tt, unwrap([r["psi"] for r in rows]))
    return max(res) - min(res)


def split_pp(seg, t_split=10.0):
    """前 t_split 秒 / 之后 的航向峰峰 —— 用来区分"一直摆"和"慢慢长起来"。"""
    if not seg:
        return float('nan'), float('nan')
    t0 = seg[0]["t"]
    head = [r for r in seg if r["t"] - t0 <= t_split]
    tail = [r for r in seg if r["t"] - t0 > t_split]
    return seg_psi_pp(head), seg_psi_pp(tail)


def merged_runs(rows, gap=3.0, **kw):
    """在 all_steady 的基础上，把间隔 <gap 秒的两段合并成一段。

    位姿（TF）短暂丢失或高度野值会把一段悬停切成几块，这里并回去。
    """
    runs = all_steady(rows, **kw)
    out = []
    for seg in runs:
        if out and seg[0]["t"] - out[-1][-1]["t"] <= gap:
            out[-1].extend(seg)
        else:
            out.append(list(seg))
    return out


def main():
    fixed = len(sys.argv) > 3
    lo = float(sys.argv[2]) if fixed else 0.0
    hi = float(sys.argv[3]) if fixed else 0.0

    if '--scan' in sys.argv:
        cands = []
        for pat in ("rec_*.log", "console_*.log", "fc_*.log", "ourloop*.log"):
            cands += glob.glob(os.path.expanduser("~/traces/" + pat))
        cands.sort(key=os.path.getmtime, reverse=True)
        print("=== 悬停段扫描（离地 >60cm 且高度稳定 >=5s）===")
        for p in cands[:10]:
            rows = clean_h([r for r in parse(p)
                            if not math.isnan(r["x"]) and not math.isnan(r["y"])])
            runs = merged_runs(rows)
            name = os.path.basename(p)
            if not runs:
                print("%-34s %5d 有效样本   无悬停段" % (name, len(rows)))
                continue
            for i, seg in enumerate(runs, 1):
                hs = [r["h"] for r in seg]
                hd, tl = split_pp(seg)
                print("%-30s 段%d t=%7.1f s %5.1fs 高%3.0fcm | 航向 %5.1f° 前10s/后段 %5.1f/%5.1f"
                      % (name, i, seg[0]["t"], seg[-1]["t"] - seg[0]["t"],
                         sum(hs) / len(hs), seg_psi_pp(seg), hd, tl))
        return

    if len(sys.argv) > 1:
        paths = [sys.argv[1]]
    else:
        cands = []
        for pat in ("rec_*.log", "fc_*.log", "ourloop*.log"):
            cands += glob.glob(os.path.expanduser("~/traces/" + pat))
        cands.sort(key=os.path.getmtime, reverse=True)
        if not cands:
            print("没找到 ~/traces/rec_*.log / fc_*.log")
            return
        paths = cands[:8]

    # ★ 2026-10-07：换电池会重启板子，记录器随即开一个**新文件**。
    #   如果只取"最新那个"，就会去分析一个还没飞过的空文件。
    #   这里按 mtime 从新到旧找，取第一个**真的含悬停段**的文件。
    path, rows, seg, fallback = None, [], None, None
    for p in paths:
        r = clean_h([x for x in parse(p)
                     if not math.isnan(x["x"]) and not math.isnan(x["y"])])
        # 自动模式取**最近**的一段（不是最长）——你刚飞完的那场才是要看的
        segs = merged_runs(r) if not fixed else [x for x in (longest_run(r, lo, hi),) if x]
        if not segs and not fixed:
            st = longest_still(r)          # 没有高度数据（串口线拔了）时按"几乎不动"找
            segs = [st] if st else []
        s = segs[-1] if segs else None
        if fallback is None:
            fallback = (p, r)
        if s:
            path, rows, seg = p, r, s
            break
    if path is None:
        path, rows = fallback

    print("=== 悬停报告 ===")
    print("记录: %s" % path)
    # ★ 2026-10-07：日志里不记录飞控参数，ID65 / PID-3 的取值只能靠"人记"，
    #   一旦记错整张对照表就有个点是错的。约定：每次改完参数，在板上写一行：
    #     echo "ID65=1030 PID3=1200/100/10" >> ~/diag/fc_params.txt
    #   这里就把最后一行打出来，作为本次分析对应的参数取值。
    try:
        _pl = os.path.expanduser('~/diag/fc_params.txt')
        if os.path.exists(_pl):
            _ls = [x.strip() for x in open(_pl, encoding='utf-8', errors='ignore') if x.strip()]
            if _ls:
                print("参数标签: %s" % _ls[-1])
    except Exception:                     # noqa: BLE001
        pass
    if len(rows) < 50:
        print("解析到的有效样本太少（%d）" % len(rows))
        return
    print("数据范围: t=%.1f~%.1f s（%d 样本）" % (rows[0]["t"], rows[-1]["t"], len(rows)))

    if not seg:
        heights = sorted(r["h"] for r in rows if not math.isnan(r["h"]))
        if heights:
            print("没找到悬停段（要离地 >60cm 且高度变化率 <10cm/s、连续 >=5s）；"
                  "本次高度 中位 %.0f / 最大 %.0f cm"
                  % (heights[len(heights) // 2], heights[-1]))
        else:
            print("全程没读到高度（/height 有没有在发？）")
        return

    t = [r["t"] - seg[0]["t"] for r in seg]
    dur = t[-1]
    print("悬停窗口: %.1fs  %d 样本  (t=%.1f~%.1f s)%s"
          % (dur, len(seg), seg[0]["t"], seg[-1]["t"],
             "（固定窗口 %.0f~%.0fcm）" % (lo, hi) if fixed else "（自动识别）"))
    if dur < 8.0:
        print("!! 窗口不到 8 秒，指标仅供参考")
    if all(math.isnan(r["h"]) for r in seg):
        print("!! 本窗口没有飞控高度数据（串口线断了？）→ 悬停段是按\"水平几乎不动\"识别的")

    # ★ 位姿冻结检测：x/y/psi 连续完全相同 = 位姿停止更新，记录器在重复写旧值。
    #   2026-10-07 实测踩坑：10-01 那场"好成绩"后半段就是这样冻住的，
    #   冻结段贡献 0 摆动 → 峰峰被严重低估（看着像"锁死航向"，其实是没有数据）。
    frozen = sum(1 for i in range(1, len(seg))
                 if seg[i]["x"] == seg[i - 1]["x"] and seg[i]["y"] == seg[i - 1]["y"]
                 and seg[i]["psi"] == seg[i - 1]["psi"])
    frozen_ratio = frozen / max(len(seg) - 1, 1)
    if frozen_ratio > 0.2:
        print("!! 位姿冻结 %.0f%%（位姿停止更新）→ 下面的幅度指标偏乐观，只当参考"
              % (frozen_ratio * 100))

    hs = [r["h"] for r in seg]
    hm = sum(hs) / len(hs)
    h_sigma = (sum((v - hm) ** 2 for v in hs) / len(hs)) ** 0.5

    res_all = []
    for key in ("x", "y"):
        res_all.append(detrend(t, [r[key] for r in seg]))
    xy_pp = math.hypot(max(res_all[0]) - min(res_all[0]),
                       max(res_all[1]) - min(res_all[1]))
    drift = math.hypot(seg[-1]["x"] - seg[0]["x"], seg[-1]["y"] - seg[0]["y"])
    _, f_x = crossings(t, res_all[0])
    _, f_y = crossings(t, res_all[1])
    f_xy = max(f_x, f_y)

    psi = unwrap([r["psi"] for r in seg])
    psi_res = detrend(t, psi)
    psi_pp = max(psi_res) - min(psi_res)
    psi_drift = psi[-1] - psi[0]
    _, f_psi = crossings(t, psi_res)

    # ★ 前 10 秒 vs 后段：实测现象是"前 10s 稳，之后开始左右摆并越来越"。
    #   整段平均会把"还在长"的那段抹平 —— 分开看才知道是"一直摆"还是"长起来的"。
    pp_head = seg_psi_pp([r for r, tt in zip(seg, t) if tt <= 10.0])
    pp_tail = seg_psi_pp([r for r, tt in zip(seg, t) if tt > 10.0])

    rate = []
    for i in range(1, len(psi)):
        dt = t[i] - t[i - 1]
        rate.append((psi[i] - psi[i - 1]) / dt if dt > 1e-6 else 0.0)
    cmd = [r["yawv"] for r in seg[1:]]
    yaw_lag, yaw_corr = xcorr_lag(t[1:], cmd, rate)
    mc = sum(cmd) / len(cmd)
    sc = (sum((v - mc) ** 2 for v in cmd) / len(cmd)) ** 0.5
    mr = sum(rate) / len(rate)
    sr = (sum((v - mr) ** 2 for v in rate) / len(rate)) ** 0.5
    # 高频抖动：变化率做 3 点平滑后取残差的标准差 —— 只看 >2Hz 的成分。
    # 机架/桨/减震变差时这里会明显变大，是"机械问题"最直接的指纹。
    if len(rate) > 6:
        sm = [(rate[i - 1] + rate[i] + rate[i + 1]) / 3.0 for i in range(1, len(rate) - 1)]
        resid = [rate[i + 1] - sm[i] for i in range(len(sm))]
        mres = sum(resid) / len(resid)
        yaw_jitter = (sum((v - mres) ** 2 for v in resid) / len(resid)) ** 0.5
    else:
        yaw_jitter = float('nan')

    vx, vy, cx, cy = [], [], [], []
    for i in range(1, len(seg)):
        dt = t[i] - t[i - 1]
        vx.append((seg[i]["x"] - seg[i - 1]["x"]) / dt if dt > 1e-6 else 0.0)
        vy.append((seg[i]["y"] - seg[i - 1]["y"]) / dt if dt > 1e-6 else 0.0)
        c_, s_ = math.cos(math.radians(seg[i]["psi"])), math.sin(math.radians(seg[i]["psi"]))
        cx.append(seg[i]["tvx"] * c_ - seg[i]["tvy"] * s_)
        cy.append(seg[i]["tvx"] * s_ + seg[i]["tvy"] * c_)
    xlag, _ = xcorr_lag(t[1:], cx, vx)
    ylag, _ = xcorr_lag(t[1:], cy, vy)

    print("")
    print("%-12s %-11s %-13s %s" % ("指标", "本次", "基线(10-06)", "判定"))
    print("%-12s %-11s %-13s %s" % ("高度 sigma", "%.1f cm" % h_sigma, "<= %.1f cm" % BASE["h_sigma"],
                                    mark(h_sigma, BASE["h_sigma"])))
    print("%-12s %-11s %-13s %s" % ("水平峰峰", "%.1f cm" % xy_pp, "<= %.1f cm" % BASE["xy_pp"],
                                    mark(xy_pp, BASE["xy_pp"])))
    print("%-12s %-11s %-13s %s" % ("水平漂移", "%+.1f cm" % drift, "<= %.1f cm" % BASE["xy_drift"],
                                    mark(drift, BASE["xy_drift"])))
    print("%-12s %-11s %-13s %s" % ("航向峰峰", "%.1f deg" % psi_pp, "<= %.1f deg" % BASE["psi_pp"],
                                    mark(psi_pp, BASE["psi_pp"])))
    print("%-12s %-11s %-13s %s" % ("前10s→后段", "%.1f/%.1f" % (pp_head, pp_tail), "看增长", ""))
    print("%-12s %-11s %-13s %s" % ("航向漂移", "%+.1f deg" % psi_drift, "<= %.1f deg" % BASE["psi_drift"],
                                    mark(psi_drift, BASE["psi_drift"])))
    print("%-12s %-11s %-13s %s" % ("航向主频", "%.2f Hz" % f_psi, "<= %.2f Hz" % BASE["freq"],
                                    mark(f_psi, BASE["freq"])))
    print("%-12s %-11s %-13s %s" % ("航向高频抖", "%.2f deg/s" % yaw_jitter,
                                    "<= %.2f deg/s" % BASE["yaw_jitter"],
                                    mark(yaw_jitter, BASE["yaw_jitter"])))
    print("%-12s %-11s %-13s %s" % ("水平主频", "%.2f Hz" % f_xy, "<= %.2f Hz" % BASE["freq"],
                                    mark(f_xy, BASE["freq"])))
    if math.isnan(yaw_lag):
        # 指令恒定 -> 相关系数无意义：说明我们的位置环根本没参与（纯飞控那一场）
        print("%-12s %-11s %-13s %s" % ("偏航响应滞后", "无法计算", "<= %.2f s" % BASE["yaw_lag"],
                                        "纯飞控"))
    else:
        print("%-12s %-11s %-13s %s" % ("偏航响应滞后", "%.2f s" % yaw_lag, "<= %.2f s" % BASE["yaw_lag"],
                                        mark(yaw_lag, BASE["yaw_lag"])))
    print("%-12s %-11s %-13s %s" % ("水平响应滞后", "%.2f s" % max(xlag, ylag), "-", ""))
    print("")
    print("偏航指令 %+.2f±%.2f deg/s   实际 %+.2f±%.2f deg/s   corr %.2f"
          % (mc, sc, mr, sr, yaw_corr))
    print("高度 均值 %.0f cm   范围 %.0f~%.0f cm" % (hm, min(hs), max(hs)))


if __name__ == "__main__":
    main()

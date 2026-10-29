#!/usr/bin/env python3
"""分析 flight_console 悬停自测记录（~/traces/console_*.log）。

行格式:
  t=  0.05 x=   3.73 y=   0.92 psi=  164.5 | filt=( 10.3,  4.5) raw=( 10.4,  4.8)
  tvd=( nan, nan) z= nan yaw= nan | fc回显=( 0, 0) h=None mode=None unlock=None

用法: python tools/analyze_console.py flight_data/console_20260928_181239.log
"""
import math
import re
import sys


def parse(path):
    rows = []
    with open(path, errors="ignore") as fh:
        for line in fh:
            if "t=" not in line or "psi=" not in line:
                continue
            g = lambda k: (float(m.group(1)) if (m := re.search(k + r"=\s*(-?[\d.]+)", line)) else None)
            h = g(r"\bh")            # 不能写 h= 会撞上路径里的字符，这里用 \bh 加空格前的字段名
            rec = {"t": g(r"t"), "x": g(r"x"), "y": g(r"y"), "psi": g(r"psi"),
                   "h": h, "mode": g(r"mode"), "unlock": g(r"unlock")}
            if rec["t"] is None or rec["x"] is None or rec["h"] is None:
                continue
            rows.append(rec)
    return rows


def stats(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return (float("nan"),) * 3
    m = sum(vals) / len(vals)
    var = sum((v - m) ** 2 for v in vals) / len(vals)
    return m, math.sqrt(var), max(abs(v - m) for v in vals)


def longest_hover(rows, lo=105.0, hi=135.0, min_len=20):
    best, cur = [], []
    for r in rows:
        if lo <= r["h"] <= hi:
            cur.append(r)
        else:
            if len(cur) > len(best):
                best = cur
            cur = []
    if len(cur) > len(best):
        best = cur
    return best if len(best) >= min_len else []


def main():
    path = sys.argv[1]
    rows = parse(path)
    if not rows:
        print("没解析到数据"); return
    t0, t1 = rows[0]["t"], rows[-1]["t"]
    hs = [r["h"] for r in rows]
    print(f"记录: {path}")
    print(f"时长 {t1 - t0:.1f}s  样本 {len(rows)} 行（{len(rows) / max(t1 - t0, 1e-9):.0f} Hz）")
    print(f"高度全程 {min(hs):.0f}~{max(hs):.0f} cm   最高点 {max(hs):.0f} cm")

    hv = longest_hover(rows)
    if hv:
        mh, sh, dh = stats([r["h"] for r in hv])
        xs, ys = [r["x"] for r in hv], [r["y"] for r in hv]
        mx, sx, dx = stats(xs)
        my, sy, dy = stats(ys)
        mx0, my0 = xs[0], ys[0]
        drift = max(math.hypot(r["x"] - mx0, r["y"] - my0) for r in hv)
        print()
        print(f"悬停段 t={hv[0]['t'] - t0:.1f}~{hv[-1]['t'] - t0:.1f}s"
              f"（{hv[-1]['t'] - hv[0]['t']:.1f}s）")
        print(f"  高度   {mh:.1f} ± {sh:.1f} cm（最大偏离 {dh:.1f}）")
        print(f"  x 位置 {mx:.1f} ± {sx:.1f} cm（最大偏离 {dx:.1f}）")
        print(f"  y 位置 {my:.1f} ± {sy:.1f} cm（最大偏离 {dy:.1f}）")
        print(f"  水平漂移（相对悬停起点）最大 {drift:.1f} cm")
        psi = [r["psi"] for r in hv if r["psi"] is not None]
        if psi:
            mp, sp, dp = stats(psi)
            cross = sum(1 for a, b in zip(psi, psi[1:])
                        if (a - mp) * (b - mp) < 0)
            dur = hv[-1]["t"] - hv[0]["t"]
            print(f"  航向   {mp:.1f} ± {sp:.1f}°（峰峰值 {2 * dp:.1f}°，"
                  f"{cross} 次过均值，约 {cross / dur:.2f} Hz）")
    else:
        print("没找到稳定的悬停段")

    # 起飞/落地时刻
    air = [r for r in rows if r["h"] > 20]
    if air:
        print()
        print(f"离地 {air[0]['t'] - t0:.1f}s → 落地 {air[-1]['t'] - t0:.1f}s"
              f"（空中 {air[-1]['t'] - air[0]['t']:.1f}s）")
        hx = [r["x"] for r in air]
        hy = [r["y"] for r in air]
        span = max(math.hypot(r["x"] - air[0]["x"], r["y"] - air[0]["y"]) for r in air)
        print(f"空中水平最大偏离起点 {span:.1f} cm；x 范围 {min(hx):.0f}~{max(hx):.0f}，"
              f"y 范围 {min(hy):.0f}~{max(hy):.0f}")


if __name__ == "__main__":
    main()

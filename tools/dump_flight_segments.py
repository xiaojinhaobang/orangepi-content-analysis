#!/usr/bin/env python3
"""把 flight_data/*.log（flight_console 记录格式）拆成飞行段并逐秒重放。

用法: python tools/dump_flight_segments.py flight_data/last_fc910.log [--every 1.0]
"""
import math
import re
import sys

NUM = r"([-\d.]+|nan)"
RE_T = re.compile(r"t=\s*" + NUM)
RE_XY = re.compile(r"x=\s*" + NUM + r"\s+y=\s*" + NUM)
RE_PSI = re.compile(r"psi=\s*" + NUM)
RE_FILT = re.compile(r"filt=\(\s*" + NUM + r",\s*" + NUM + r"\)")
RE_RAW = re.compile(r"raw=\(\s*" + NUM + r",\s*" + NUM + r"\)")
RE_ECHO = re.compile(r"fc回显=\(\s*" + NUM + r",\s*" + NUM + r"\)")
RE_TVD = re.compile(r"tvd=\(\s*" + NUM + r",\s*" + NUM + r"\)")
RE_ZV = re.compile(r"z=\s*" + NUM)
RE_YAWV = re.compile(r"yaw=\s*" + NUM)
RE_H = re.compile(r"h=(\S+)\s+mode=(\S+)\s+unlock=(\S+)")
RE_FCGYROZ = re.compile(r"fcGyroZ=\s*" + NUM)
RE_FCYAWCMD = re.compile(r"fcYawCmd=\s*" + NUM)


def g(m, i=1):
    try:
        return float(m.group(i))
    except (AttributeError, ValueError):
        return math.nan


def parse(path):
    rows = []
    with open(path, errors="ignore", encoding="utf-8") as fh:
        for line in fh:
            mt, mxy = RE_T.search(line), RE_XY.search(line)
            if not mt or not mxy:
                continue
            mh = RE_H.search(line)
            h = math.nan
            mode = unlock = None
            if mh:
                try:
                    h = float(mh.group(1))
                except ValueError:
                    h = math.nan
                mode, unlock = mh.group(2), mh.group(3)
            mf, mr, me = RE_FILT.search(line), RE_RAW.search(line), RE_ECHO.search(line)
            mtv = RE_TVD.search(line)
            rows.append({
                "t": g(mt), "x": g(mxy, 1), "y": g(mxy, 2),
                "psi": g(RE_PSI.search(line)),
                "fx": g(mf, 1), "fy": g(mf, 2),
                "rx": g(mr, 1), "ry": g(mr, 2),
                "ex": g(me, 1), "ey": g(me, 2),
                "tvx": g(mtv, 1), "tvy": g(mtv, 2),
                "zv": g(RE_ZV.search(line)), "yawv": g(RE_YAWV.search(line)),
                "gzz": g(RE_FCGYROZ.search(line)),      # 飞控陀螺 Z 原始 LSB（恒定偏航力矩的硬证据）
                "yawcmd": g(RE_FCYAWCMD.search(line)),  # 飞控偏航角速度命令 deg/s
                "h": h, "mode": mode, "unlock": unlock,
            })
    return rows


def segments(rows, h_lo=20.0, min_len=10):
    segs, cur = [], []
    for r in rows:
        air = (not math.isnan(r["h"])) and r["h"] > h_lo
        if air:
            cur.append(r)
        else:
            if len(cur) >= min_len:
                segs.append(cur)
            cur = []
    if len(cur) >= min_len:
        segs.append(cur)
    return segs


def wrap_deg(d):
    return (d + 180.0) % 360.0 - 180.0


def report_segment(idx, seg):
    t0 = seg[0]["t"]
    dur = seg[-1]["t"] - t0
    hs = [r["h"] for r in seg if not math.isnan(r["h"])]
    print(f"\n--- 飞行段{idx}: t={t0:.1f}~{seg[-1]['t']:.1f}s  时长 {dur:.1f}s  {len(seg)} 样本")
    print(f"    高度 {min(hs):.0f}~{max(hs):.0f} cm（末段中位 {sorted(hs)[len(hs)//2]:.0f}）")

    # 水平漂移相对本段最早有效位姿
    ref = next(((r["x"], r["y"]) for r in seg
                if not math.isnan(r["x"]) and not math.isnan(r["y"])), None)
    if ref:
        dmax, tmax = 0.0, 0.0
        for r in seg:
            if math.isnan(r["x"]) or math.isnan(r["y"]):
                continue
            d = math.hypot(r["x"] - ref[0], r["y"] - ref[1])
            if d > dmax:
                dmax, tmax = d, r["t"] - t0
        print(f"    水平最大偏离起飞点 {dmax:.1f} cm（发生在 {tmax:.1f}s）")

    # 航向：累计漂移 + 分段角速度
    ps = [(r["t"] - t0, r["psi"]) for r in seg if not math.isnan(r["psi"])]
    if len(ps) > 20:
        unw, acc = [], 0.0
        for i, (_, p) in enumerate(ps):
            if i:
                acc += wrap_deg(p - ps[i - 1][1])
            unw.append(acc)
        print(f"    航向累计变化 {unw[-1]:+.1f}°   峰峰 {max(unw) - min(unw):.1f}°")
        half = len(ps) // 2
        for name, lo, hi in (("前半", 0, half), ("后半", half, len(ps))):
            dt = ps[hi - 1][0] - ps[lo][0]
            if dt > 0.5:
                print(f"      {name} {ps[lo][0]:5.1f}~{ps[hi - 1][0]:5.1f}s 航向变化 "
                      f"{unw[hi - 1] - unw[lo]:+7.1f}°  均值 {abs(unw[hi - 1] - unw[lo]) / dt:5.2f} °/s")
        # 角速度的振荡幅度（后半段）
        rates = []
        for i in range(max(1, half), len(ps)):
            dt = ps[i][0] - ps[i - 1][0]
            if dt > 1e-6:
                rates.append(wrap_deg(ps[i][1] - ps[i - 1][1]) / dt)
        if rates:
            mr = sum(rates) / len(rates)
            print(f"      后半段角速度 {mr:+.2f} ± {(sum((x - mr) ** 2 for x in rates) / len(rates)) ** 0.5:.2f} °/s")

    raw = [math.hypot(r["rx"], r["ry"]) for r in seg if not math.isnan(r["rx"])]
    filt = [math.hypot(r["fx"], r["fy"]) for r in seg if not math.isnan(r["fx"])]
    for name, v in (("雷达原始速度", raw), ("注入速度", filt)):
        if v:
            m = sum(v) / len(v)
            print(f"    {name}: 均值 {m:5.1f}  峰值 {max(v):6.1f} cm/s  "
                  f">25cm/s 占比 {100 * sum(1 for x in v if x > 25) / len(v):4.1f}%")
    modes = sorted({r["mode"] for r in seg if r["mode"] is not None})
    print(f"    模式 {modes}")


def main():
    path = sys.argv[1]
    every = 1.0
    if "--every" in sys.argv:
        every = float(sys.argv[sys.argv.index("--every") + 1])
    rows = parse(path)
    if not rows:
        print("没解析到数据")
        return
    print(f"记录 {path}: {rows[0]['t']:.1f}~{rows[-1]['t']:.1f}s, {len(rows)} 行")
    segs = segments(rows)
    print(f"共 {len(segs)} 个疑似飞行段（h>20cm 连续段）")
    for i, seg in enumerate(segs, 1):
        report_segment(i, seg)

    print("\n==== 逐秒重放（h>20 的段内） ====")
    for i, seg in enumerate(segs, 1):
        t0 = seg[0]["t"]
        print(f"\n[段{i}]  t     h   x     y     psi   |raw| |filt| echo_vx echo_vy mode lock")
        nxt = 0.0
        for r in seg:
            rel = r["t"] - t0
            if rel < nxt:
                continue
            nxt = rel + every
            print(f"  {rel:6.1f} {r['h']:4.0f} {r['x']:7.1f} {r['y']:7.1f} {r['psi']:7.1f} "
                  f"{math.hypot(r['rx'], r['ry']):5.1f} {math.hypot(r['fx'], r['fy']):6.1f} "
                  f"{r['ex']:6.0f} {r['ey']:6.0f} {str(r['mode']):>4} {str(r['unlock']):>4}")


if __name__ == "__main__":
    main()

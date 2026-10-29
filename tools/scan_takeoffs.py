# -*- coding: utf-8 -*-
"""Scan many rec_*.log files and tabulate every unlock window that spun the motors up.

Usage: python3 tools/scan_takeoffs.py <glob...>
"""
import glob
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze_rollover import parse, find_events  # noqa: E402


def pwm4(r):
    p = r.get("pwm") or []
    if len(p) < 4 or any(v != v for v in p):
        return None
    return p


def collective(r):
    p = pwm4(r)
    if not p or max(p) <= 0:
        return None
    return sum(p) / 4.0


def spread(p):
    if not p:
        return None
    return max(p) - min(p)


def scan(path):
    rows = parse(path)
    if not rows:
        return []
    out = []
    for a, b in find_events(rows):
        win = [r for r in rows if a - 0.5 <= r["t"] <= b + 0.5]
        live = [r for r in win if collective(r) is not None and collective(r) > 20]
        if not live:
            continue
        gnd = [r for r in live if not (r["h"] == r["h"]) or r["h"] <= 10]
        hmax = max((r["h"] for r in live if r["h"] == r["h"]), default=float("nan"))
        lifted = hmax == hmax and hmax > 25
        c_gnd_max = max((collective(r) for r in gnd), default=float("nan"))
        c_at_lift = None
        if lifted:
            for r in live:
                if r["h"] == r["h"] and r["h"] > 8:
                    c_at_lift = collective(r)
                    break
        spreads = [spread(pwm4(r)) for r in gnd]
        spreads = [s for s in spreads if s is not None]
        thr0 = live[0]["thr"]
        t_up = live[0]["t"] - a
        cmd1 = sorted({int(r["cmd1"]) for r in live[:60] if r["cmd1"] == r["cmd1"]})
        out.append(
            dict(
                file=os.path.basename(path),
                t=a,
                dur=b - a,
                thr=thr0,
                t_motor=t_up,
                c_max_gnd=c_gnd_max,
                c_lift=c_at_lift,
                h_max=hmax,
                lifted=lifted,
                dmx=(max(spreads) if spreads else float("nan")),
                cmd1=cmd1,
            )
        )
    return out


HDR = (
    f"{'file':<26}{'t_unl':>9}{'dur':>6}{'thr':>6}{'tMot':>6}"
    f"{'C@gnd':>7}{'C@lift':>8}{'h_max':>7}{'lift':>6}{'dmxGnd':>8}  cmd1"
)


def main():
    pats = sys.argv[1:] or ["rec_*.log"]
    print(HDR)
    for pat in pats:
        for f in sorted(glob.glob(pat)):
            for e in scan(f):
                print(
                    f"{e['file']:<26}{e['t']:>9.2f}{e['dur']:>6.1f}{e['thr']:>6.0f}"
                    f"{e['t_motor']:>6.1f}"
                    f"{(e['c_max_gnd'] if e['c_max_gnd'] == e['c_max_gnd'] else 0):>7.1f}"
                    f"{(e['c_lift'] if e['c_lift'] is not None else 0):>8.1f}"
                    f"{(e['h_max'] if e['h_max'] == e['h_max'] else 0):>7.0f}"
                    f"{('Y' if e['lifted'] else '.'):>6}{e['dmx']:>8.1f}"
                    f"  {e['cmd1']}"
                )
    return 0


if __name__ == "__main__":
    sys.exit(main())

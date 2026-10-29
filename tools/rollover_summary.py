# -*- coding: utf-8 -*-
"""One-line-per-unlock-window summary across many rec_*.log files.

Usage: python tools/rollover_summary.py <log> [<log> ...]
"""
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze_rollover import parse, find_events  # noqa: E402


def pwm_of(r):
    p = r.get("pwm") or []
    return p + [float("nan")] * (4 - len(p)) if len(p) < 4 else p[:4]


def summarize(path):
    rows = parse(path)
    if not rows:
        return []
    name = os.path.basename(path)
    out = []
    for a, b in find_events(rows):
        win = [r for r in rows if a <= r["t"] <= b]
        if not win:
            continue
        # ground phase = before h exceeds 10
        gnd = []
        air = []
        for r in win:
            (air if (r["h"] == r["h"] and r["h"] > 10) else gnd).append(r)
        top = max((r["thr"] for r in gnd if r["thr"] == r["thr"]), default=float("nan"))
        # widest differential while still on the ground
        widest = None
        for r in gnd:
            p = pwm_of(r)
            if any(v != v for v in p) or max(p) == 0:
                continue
            d = max(p) - min(p)
            if widest is None or d > widest[0]:
                widest = (d, r)
        imax = imin = -1
        if widest:
            p = pwm_of(widest[1])
            imax, imin = p.index(max(p)), p.index(min(p))
        rl = [r["roll"] for r in win if r["roll"] == r["roll"]]
        pt = [r["pitch"] for r in win if r["pitch"] == r["pitch"]]
        hmax = max((r["h"] for r in win if r["h"] == r["h"]), default=float("nan"))
        end_u = win[-1]["unlock"]
        out.append(
            (
                name,
                a,
                round(b - a, 1),
                round(top),
                round(hmax),
                round(widest[0], 1) if widest else None,
                widest[1]["t"] if widest else None,
                imax,
                imin,
                round(min(rl), 2) if rl else None,
                round(max(rl), 2) if rl else None,
                round(max(abs(v) for v in pt), 2) if pt else None,
                int(end_u) if end_u == end_u else None,
                len(air),
            )
        )
    return out


HDR = (
    f"{'file':<28}{'t_unlock':>9}{'dur':>6}{'thr_top':>8}{'h_max':>6}"
    f"{'dmx_max':>8}{'t@dmx':>8}{'hi':>3}{'lo':>3}"
    f"{'roll_min':>9}{'roll_max':>9}{'|pit|max':>9}{'unl_end':>8}{'air_rows':>9}"
)


def main():
    files = sys.argv[1:]
    print(HDR)
    for f in files:
        for row in summarize(f):
            (name, a, dur, top, hmax, dmx, tdmx, imax, imin, rlo, rhi, pmax, uend, nair) = row
            print(
                f"{name:<28}{a:>9.2f}{dur:>6.1f}{top:>8.0f}{hmax:>6.0f}"
                f"{dmx:>8.1f}{tdmx:>8.2f}{imax:>3}{imin:>3}"
                f"{rlo:>9.2f}{rhi:>9.2f}{pmax:>9.2f}{uend:>8}{nair:>9}"
            )
    return 0


if __name__ == "__main__":
    sys.exit(main())

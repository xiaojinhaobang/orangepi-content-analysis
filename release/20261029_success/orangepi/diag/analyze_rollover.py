# -*- coding: utf-8 -*-
"""Parse ~/traces/rec_*.log (20 Hz wide recorder) and dump takeoff/rollover windows.

Usage:
    python tools/analyze_rollover.py <log> [--before 3] [--after 6]
    python tools/analyze_rollover.py <log> --events      # list unlock/lock events only
"""
import re
import sys
import math

NUM = r"[-+0-9.eE]+"

RE_T = re.compile(r"t=\s*(" + NUM + r")")
RE_XY = re.compile(r"x=\s*(" + NUM + r")\s+y=\s*(" + NUM + r")")
RE_PSI = re.compile(r"psi=\s*(" + NUM + r")")
RE_Z = re.compile(r"\bz=\s*(" + NUM + r")")
RE_ECHO = re.compile(r"fc回显=\(\s*(" + NUM + r"),\s*(" + NUM + r")\)")
RE_HMU = re.compile(r"h=(\S+)\s+mode=(\S+)\s+unlock=(\S+)")


def _f(x):
    try:
        v = float(x)
        return v
    except (TypeError, ValueError):
        return float("nan")


def _nums(text, key):
    """Grab the parenthesised number list that follows `key=(`."""
    i = text.find(key + "=(")
    if i < 0:
        return []
    j = text.find(")", i)
    return [_f(v) for v in re.findall(NUM, text[i + len(key) + 2:j])]


def _scalar(text, key):
    m = re.search(key + r"=(?:\s*)(" + NUM + r")", text)
    return _f(m.group(1)) if m else float("nan")


def parse(path):
    rows = []
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            if not line.startswith("t="):
                continue
            r = {}
            m = RE_T.search(line)
            if not m:
                continue
            r["t"] = _f(m.group(1))
            m = RE_XY.search(line)
            r["x"], r["y"] = (_f(m.group(1)), _f(m.group(2))) if m else (float("nan"),) * 2
            m = RE_PSI.search(line)
            r["psi"] = _f(m.group(1)) if m else float("nan")
            m = RE_Z.search(line)
            r["z"] = _f(m.group(1)) if m else float("nan")
            m = re.search(r"\byaw=\s*(" + NUM + r")", line)
            r["yaw"] = _f(m.group(1)) if m else float("nan")
            m = RE_ECHO.search(line)
            r["cmd0"], r["cmd1"] = (_f(m.group(1)), _f(m.group(2))) if m else (float("nan"),) * 2
            m = RE_HMU.search(line)
            if m:
                r["h"] = _f(m.group(1))
                r["mode"] = _f(m.group(2))
                r["unlock"] = _f(m.group(3))
            else:
                r["h"] = r["mode"] = r["unlock"] = float("nan")
            r["yawE"] = _scalar(line, "fcYawEuler")
            r["gyroZ"] = _scalar(line, "fcGyroZ")
            r["yawCmd"] = _scalar(line, "fcYawCmd")
            r["thr"] = _scalar(line, "fcThr")
            r["tgtVx"] = _scalar(line, "fcTgtVx")
            r["tgtVy"] = _scalar(line, "fcTgtVy")
            r["roll"] = _scalar(line, "fcRoll")
            r["pitch"] = _scalar(line, "fcPitch")
            r["accX"] = _scalar(line, "fcAccX")
            r["accY"] = _scalar(line, "fcAccY")
            r["velX"] = _scalar(line, "fcVelX")
            r["velY"] = _scalar(line, "fcVelY")
            r["pwm"] = _nums(line, "fcPwm")
            r["tarA"] = _nums(line, "fcTarA")
            r["tarV"] = _nums(line, "fcTarV")
            r["ctlA"] = _nums(line, "fcCtlA")
            r["gyroXY"] = _nums(line, "fcGyroXY")
            rows.append(r)
    return rows


def _g(row, key, idx=0):
    seq = row.get(key) or []
    return seq[idx] if idx < len(seq) else float("nan")


HEADER = (
    "   t    | h  mode unl | thr  roll  pitch | pwm0 pwm1 pwm2 pwm3 | d13  dmx |"
    " gyroX gyroY | accX accY | vX   vY   tgtVx tgtVy | tarR tarP | ctl0 ctl1 ctl2 ctl3"
)

HEADER_C = (
    "   t    | h  unl | thr  roll  pitch | pwm0 pwm1 pwm2 pwm3 | d13  dmx |"
    " gyroX gyroY | accX accY"
)

HEADER_D = (
    "   t    |h  unl|thr |roll pitch|pwm0 pwm1 pwm2 pwm3| dmx| gX  gY  gZ|"
    " fcYaw ywCmd| vX    vY  | tVx  tVy |cmd0 cmd1|psi  h_ros| x     y"
)


def fmt_d(r):
    p = r["pwm"] + [float("nan")] * 4
    dmx = max(p[:4]) - min(p[:4]) if all(v == v for v in p[:4]) else float("nan")
    g = r["gyroXY"] + [float("nan")] * 2
    return (
        f"{r['t']:9.2f} |{r['h']:3.0f}{r['unlock']:3.0f}|{r['thr']:4.0f}|"
        f"{r['roll']:5.2f}{r['pitch']:6.2f}|"
        f"{p[0]:6.1f}{p[1]:6.1f}{p[2]:6.1f}{p[3]:6.1f}|{dmx:4.1f}|"
        f"{g[0]:4.0f}{g[1]:4.0f}{r['gyroZ']:4.0f}|"
        f"{r['yaw'] if 'yaw' in r else float('nan'):6.1f}{r['yawCmd']:6.1f}|"
        f"{r['velX']:6.1f}{r['velY']:6.1f}|{r['tgtVx']:5.1f}{r['tgtVy']:5.1f}|"
        f"{r['cmd0']:4.0f}{r['cmd1']:5.0f}|{r['psi']:5.1f}{r['z']:6.1f}|"
        f"{r['x']:7.1f}{r['y']:8.1f}"
    )



def fmt_c(r):
    p = r["pwm"] + [float("nan")] * 4
    d13 = p[1] - p[3]
    dmx = max(p[:4]) - min(p[:4]) if all(v == v for v in p[:4]) else float("nan")
    g = r["gyroXY"] + [float("nan")] * 2
    return (
        f"{r['t']:9.2f} |{r['h']:4.0f} {r['unlock']:3.0f} |"
        f"{r['thr']:5.0f} {r['roll']:6.2f} {r['pitch']:6.2f} |"
        f"{p[0]:5.1f}{p[1]:5.1f}{p[2]:5.1f}{p[3]:5.1f} |{d13:5.1f}{dmx:5.1f} |"
        f"{g[0]:6.0f}{g[1]:6.0f} |{r['accX']:5.0f}{r['accY']:5.0f}"
    )



def fmt(r):
    p = r["pwm"] + [float("nan")] * 4
    d13 = p[1] - p[3]
    dmx = max(p[:4]) - min(p[:4]) if all(v == v for v in p[:4]) else float("nan")
    g = r["gyroXY"] + [float("nan")] * 2
    a = r["tarA"] + [float("nan")] * 3
    c = r["ctlA"] + [float("nan")] * 4
    return (
        f"{r['t']:9.2f} |{r['h']:4.0f} {r['mode']:4.0f} {r['unlock']:3.0f} |"
        f"{r['thr']:5.0f} {r['roll']:6.2f} {r['pitch']:6.2f} |"
        f"{p[0]:5.1f}{p[1]:5.1f}{p[2]:5.1f}{p[3]:5.1f} |{d13:5.1f}{dmx:5.1f} |"
        f"{g[0]:6.0f}{g[1]:6.0f} |{r['accX']:5.0f}{r['accY']:5.0f} |"
        f"{r['velX']:5.1f}{r['velY']:5.1f}{r['tgtVx']:6.1f}{r['tgtVy']:6.1f} |"
        f"{a[0]:5.1f}{a[1]:5.1f} |{c[0]:6.2f}{c[1]:6.2f}{c[2]:6.2f}{c[3]:6.2f}"
    )


def find_events(rows):
    """Return [(t_unlock, t_lock)] pairs."""
    events = []
    last = float("nan")
    t_on = None
    for r in rows:
        u = r["unlock"]
        if u != u:
            continue
        if u == 1 and last != 1:
            t_on = r["t"]
        elif u == 0 and last == 1 and t_on is not None:
            events.append((t_on, r["t"]))
            t_on = None
        last = u
    return events


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    path = sys.argv[1]
    before = 3.0
    after = 6.0
    args = sys.argv[2:]
    if "--before" in args:
        before = float(args[args.index("--before") + 1])
    if "--after" in args:
        after = float(args[args.index("--after") + 1])
    rows = parse(path)
    if not rows:
        print("no rows parsed")
        return 1
    t0, t1 = rows[0]["t"], rows[-1]["t"]
    print(f"== {path}: {len(rows)} rows, t={t0:.2f}..{t1:.2f} ({t1 - t0:.0f}s)")
    events = find_events(rows)
    print(f"== unlock windows: {[(round(a, 2), round(b, 2)) for a, b in events]}")
    if "--events" in args:
        return 0
    if not events:
        print("no unlock window; dumping rows with thr>200")
        for r in rows:
            if r["thr"] == r["thr"] and r["thr"] > 200:
                print(fmt(r))
        return 0
    for a, b in events:
        print(f"\n--- window t={a - before:.2f}..{b + after:.2f} (unlock {a:.2f}..{b:.2f})")
        win = [r for r in rows if a - before <= r["t"] <= b + after]
        step = int(args[args.index("--step") + 1]) if "--step" in args else 1
        if "--diag" in args:
            print(HEADER_D)
            for r in win[::step]:
                print(fmt_d(r))
        elif step > 1:
            print(HEADER_C)
            for r in win[::step]:
                print(fmt_c(r))
        else:
            print(HEADER)
            for r in win:
                print(fmt(r))
    return 0


if __name__ == "__main__":
    sys.exit(main())

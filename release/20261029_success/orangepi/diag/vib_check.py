#!/usr/bin/env python3
"""算指定时间窗内的滚转/俯仰角速度 sd（振动指标），用于跨配置对比。

trace 每行含 `fcGyroXY=(x, y)`（飞控原始陀螺 LSB，18.4 LSB = 1 deg/s）。
悬停段的 sd 越小 = 振动越小。

用法: python3 vib_check.py <log> <t0> <t1>
"""
import math
import re
import sys

RE_T = re.compile(r"t=\s*([0-9.]+)")
RE_GXY = re.compile(r"fcGyroXY=\(\s*(-?\d+),\s*(-?\d+)\)")
GYRO_SCALE = 18.4


def main():
    if len(sys.argv) < 4:
        print(__doc__)
        return 1
    path, t0, t1 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
    xs, ys = [], []
    with open(path, errors='ignore', encoding='utf-8') as fh:
        for line in fh:
            mt = RE_T.search(line)
            if not mt:
                continue
            t = float(mt.group(1))
            if t < t0 or t > t1:
                continue
            mg = RE_GXY.search(line)
            if mg:
                xs.append(float(mg.group(1)))
                ys.append(float(mg.group(2)))
    if len(xs) < 2:
        print('样本太少: %d' % len(xs))
        return 1

    def sd(a):
        m = sum(a) / len(a)
        return math.sqrt(sum((v - m) ** 2 for v in a) / (len(a) - 1))

    print('n=%d  (%.1f~%.1f s)' % (len(xs), t0, t1))
    print('滚转 sd = %.1f LSB = %.2f deg/s' % (sd(xs), sd(xs) / GYRO_SCALE))
    print('俯仰 sd = %.1f LSB = %.2f deg/s' % (sd(ys), sd(ys) / GYRO_SCALE))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""vib_fft — 对 gyro_log 记的原始 IMU 做频谱分析，判断振动来源。

判据：
  · 桨不平衡      → 在"转速基频 f0"处出现**窄带尖峰**，且常有 2f0/3f0 谐波（间隔= f0）
  · 桨叶/桨夹问题 → 主峰在"桨叶数 × f0"（两叶桨 = 2f0）
  · 气流湍流      → **宽频抬升**，没有突出窄峰
输出：各轴的前几个峰 + 峰/中位数比（窄带程度）+ 推得的 f0 与转速

用法：python3 tools/vib_fft.py flight_data/vib/*.csv
"""
import glob
import os
import sys

import numpy as np


def load(path):
    d = np.genfromtxt(path, delimiter=",", names=True)
    return d


def trim(d, t0=None, t1=None):
    """可选：只看某个时间段（比如跳过"桨还没转"的部分）"""
    if t0 is None and t1 is None:
        return d
    t = np.asarray(d["t"], dtype=float)
    m = np.ones(len(t), dtype=bool)
    if t0 is not None:
        m &= t >= t0
    if t1 is not None:
        m &= t <= t1
    return d[m]


def spectrum(x, fs):
    x = np.asarray(x, dtype=float)
    x = x - x.mean()
    w = np.hanning(len(x))
    X = np.fft.rfft(x * w)
    f = np.fft.rfftfreq(len(x), 1.0 / fs)
    mag = np.abs(X) * 2.0 / (w.sum())
    return f, mag


def top_peaks(f, mag, n=6, fmin=15.0):
    m = f >= fmin
    fm, mm = f[m], mag[m]
    if len(mm) < 5:
        return []
    idx = []
    for i in range(1, len(mm) - 1):
        if mm[i] > mm[i - 1] and mm[i] >= mm[i + 1]:
            idx.append(i)
    idx.sort(key=lambda i: -mm[i])
    out = []
    for i in idx[:n * 3]:
        if not out or all(abs(fm[i] - o[0]) > 5.0 for o in out):
            out.append((fm[i], mm[i]))
        if len(out) >= n:
            break
    return out


def main():
    paths = sys.argv[1:] or sorted(glob.glob("flight_data/vib/*.csv"))
    if not paths:
        print("没有输入文件。先拷回 /tmp/vib_*.csv")
        return
    for p in paths:
        t0 = float(os.environ.get("VIB_T0", "0") or 0)
        t1 = float(os.environ.get("VIB_T1", "0") or 0)
        try:
            d = load(p)
        except Exception as e:
            print("== %s 读取失败: %s" % (p, e))
            continue
        if t1 > t0 > 0:
            d = trim(d, t0, t1)
            print("（只看 %.1f~%.1f s）" % (t0, t1))
        t = np.asarray(d["t"], dtype=float)
        fs = 1.0 / np.median(np.diff(t))
        print("=" * 76)
        print("%s   样本 %d  采样率 %.1f Hz  时长 %.1f s"
              % (os.path.basename(p), len(t), fs, t[-1] - t[0]))
        for axis in ("gx", "gy", "gz", "ax", "ay"):
            if axis not in d.dtype.names:
                continue
            x = np.asarray(d[axis], dtype=float)
            std = x.std()
            f, mag = spectrum(x, fs)
            med = np.median(mag)
            peaks = top_peaks(f, mag)
            ratio = (peaks[0][1] / med) if (peaks and med > 0) else float("nan")
            head = "  %-3s std=%7.1f  峰/中位=%6.1f  " % (axis, std, ratio)
            tail = "   ".join("%.0fHz(%.0f)" % (fr, mg) for fr, mg in peaks[:4])
            print(head + tail)
        # 用 gy 的峰间隔推基频
        f, mag = spectrum(np.asarray(d["gy"], dtype=float), fs)
        peaks = top_peaks(f, mag, n=4)
        if len(peaks) >= 2:
            f0 = peaks[1][0] - peaks[0][0]
            print("  峰间隔（若为谐波则=转速基频）≈ %.1f Hz → 约 %.0f 转/分"
                  % (f0, f0 * 60))


if __name__ == "__main__":
    main()

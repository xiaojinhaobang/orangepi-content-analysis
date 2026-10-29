#!/usr/bin/env python3
"""Is the lidar itself steady while the airframe is static?

Reads a rosbag2 dir with /scan, then reports frame-to-frame range noise and
per-sector behaviour. A crashed lidar shows a dead or noisy sector; a healthy
one stays within a few cm everywhere.
"""
import argparse
import math

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan


def read_scans(path, topic="/scan"):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=path, storage_id="sqlite3"),
                ConverterOptions("", ""))
    out = []
    while reader.has_next():
        name, data, _ = reader.read_next()
        if name == topic:
            out.append(deserialize_message(data, LaserScan))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--topic", default="/scan")
    ap.add_argument("--sectors", type=int, default=12)
    args = ap.parse_args()

    scans = read_scans(args.bag, args.topic)
    if len(scans) < 5:
        print(f"只有 {len(scans)} 帧，跳过")
        return
    n = len(scans[0].ranges)
    print(f"帧数 {len(scans)}  每帧点数 {n}  角度步长 {math.degrees(scans[0].angle_increment):.2f}°")

    # 帧间差：同一索引的 range 差（两帧都有效才算）
    diffs = []
    for a, b in zip(scans, scans[1:]):
        ds = [abs(x - y) for x, y in zip(a.ranges, b.ranges)
              if math.isfinite(x) and math.isfinite(y) and 0.05 < x < 8.0 and 0.05 < y < 8.0]
        if ds:
            diffs.append(sum(ds) / len(ds))
    diffs.sort()
    med = diffs[len(diffs) // 2]
    print(f"相邻帧平均距离差: 中位 {med * 100:.2f} cm  p90 {diffs[int(len(diffs) * 0.9)] * 100:.2f} cm  "
          f"最大 {diffs[-1] * 100:.2f} cm")

    # 分扇区：时间上的标准差（静止时应该都很小）+ 有效率
    width = n // args.sectors
    print("\n 扇区    方位角范围      有效率   时间标准差  中位距离")
    for s in range(args.sectors):
        lo, hi = s * width, (s + 1) * width
        valid = 0
        total = 0
        series = {}
        for fi, sc in enumerate(scans):
            for i in range(lo, hi):
                r = sc.ranges[i]
                total += 1
                if math.isfinite(r) and 0.05 < r < 8.0:
                    valid += 1
                    series.setdefault(i, []).append(r)
        stds = []
        meds = []
        for vals in series.values():
            if len(vals) > len(scans) * 0.5:
                mu = sum(vals) / len(vals)
                stds.append(math.sqrt(sum((v - mu) ** 2 for v in vals) / len(vals)))
                meds.append(mu)
        a0 = math.degrees(scans[0].angle_min) + math.degrees(scans[0].angle_increment) * lo
        a1 = math.degrees(scans[0].angle_min) + math.degrees(scans[0].angle_increment) * hi
        s_std = sorted(stds)[len(stds) // 2] if stds else float("nan")
        s_med = sorted(meds)[len(meds) // 2] if meds else float("nan")
        print(f" {s:2d}   {a0:7.1f}~{a1:7.1f}°  {100 * valid / max(1, total):6.1f}%  "
              f"{s_std * 100:8.2f} cm  {s_med:6.2f} m")


if __name__ == "__main__":
    main()

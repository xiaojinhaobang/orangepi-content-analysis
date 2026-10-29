#!/usr/bin/env python3
"""Summarize raw LaserScan data in a ROS2 bag (no flight, no control)."""
import argparse
import math
import os
import statistics
import sys

from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import LaserScan, PointCloud2


def wrap_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def summarize(msg, name):
    ranges = list(msg.ranges)
    n = len(ranges)
    valid = []
    for i, r in enumerate(ranges):
        if r is not None and math.isfinite(r) and msg.range_min <= r <= msg.range_max:
            valid.append((i, r))
    rs = [r for _, r in valid]
    # A segment is a run of valid samples separated by >= 5 invalid samples.
    segs = []
    cur = 0
    gap = 0
    for i, r in enumerate(ranges):
        ok = math.isfinite(r) and msg.range_min <= r <= msg.range_max
        if ok:
            if gap >= 5 and cur:
                segs.append(cur)
                cur = 0
            cur += 1
            gap = 0
        else:
            gap += 1
    if cur:
        segs.append(cur)
    print("[%s] samples=%d valid=%d coverage=%.1f%% angle=[%.1f,%.1f] step=%.2f deg" % (
        name, n, len(rs), 100.0 * len(rs) / max(1, n),
        math.degrees(msg.angle_min), math.degrees(msg.angle_max),
        math.degrees(msg.angle_increment)))
    if rs:
        print("  range min=%.2f median=%.2f mean=%.2f max=%.2f m | segments=%d max_seg=%d" % (
            min(rs), statistics.median(rs), sum(rs) / len(rs), max(rs),
            len(segs), max(segs)))
        bins = [0] * 8
        for r in rs:
            bins[min(7, int(r // 1.0))] += 1
        print("  histogram 0-1..7+ m: " + " ".join(str(x) for x in bins))
    return ranges, msg.angle_min, msg.angle_increment


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--csv', default='/tmp/lidar_scan.csv')
    a = ap.parse_args()
    reader = SequentialReader()
    reader.open(StorageOptions(uri=a.bag, storage_id='sqlite3'),
                ConverterOptions(input_serialization_format='cdr',
                                 output_serialization_format='cdr'))
    scans = []
    clouds = 0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/scan':
            scans.append(deserialize_message(data, LaserScan))
        elif topic == '/scan_matched_points2':
            m = deserialize_message(data, PointCloud2)
            clouds = max(clouds, int(m.width) * int(m.height))
    print("scan_frames=%d matched_cloud_max_points=%d" % (len(scans), clouds))
    if not scans:
        return 2
    picks = [0] if len(scans) == 1 else [0, len(scans) // 2, len(scans) - 1]
    for i in picks:
        ranges, amin, inc = summarize(scans[i], 'frame%d' % i)
    ranges, amin, inc = summarize(scans[len(scans) // 2], 'csv')
    with open(a.csv, 'w') as f:
        f.write('i,angle_deg,range_m\n')
        for i, r in enumerate(ranges):
            if r is not None and math.isfinite(r):
                f.write('%d,%.4f,%.4f\n' % (i, math.degrees(wrap_angle(amin + i * inc)), r))
    print("csv=%s" % a.csv)
    return 0


if __name__ == '__main__':
    sys.exit(main())

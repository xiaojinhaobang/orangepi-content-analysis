#!/usr/bin/env python3
"""Replay /scan as if the robot moved with a known constant velocity.

Points are shifted opposite to the commanded motion (world appears to move back),
then re-binned onto the original angular grid and small holes are interpolated.
Truth = v * t along `dir_deg` in the initial lidar frame.
"""
import argparse
import math
import time

import rclpy
from sensor_msgs.msg import LaserScan


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--v", type=float, default=20.0, help="cm/s")
    ap.add_argument("--dir", type=float, default=0.0, help="deg, 0 = +x of the scan frame")
    a = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("scan_shift")
    pub = node.create_publisher(LaserScan, "/scan_moved", 10)
    st = {"t0": None}

    def on_scan(msg):
        now = time.monotonic()
        if st["t0"] is None:
            st["t0"] = now
        dt = now - st["t0"]
        th = math.radians(a.dir)
        # 机器人走了 d，点云看起来反向走 d
        sx = a.v * dt * math.cos(th)
        sy = a.v * dt * math.sin(th)
        n = len(msg.ranges)
        out = [float("nan")] * n
        amin, inc = msg.angle_min, msg.angle_increment
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            ang = amin + i * inc
            x = r * math.cos(ang) - sx
            y = r * math.sin(ang) - sy
            r2 = math.hypot(x, y)
            if r2 < msg.range_min or r2 > msg.range_max:
                continue
            a2 = math.atan2(y, x)
            j = int(round((a2 - amin) / inc)) % n
            if out[j] != out[j] or r2 < out[j]:
                out[j] = r2
        # 补洞：用左右最近有效值线性插值（最多 6 格）
        idx = [i for i, v in enumerate(out) if v == v]
        if len(idx) > 10:
            for k in range(len(idx) - 1):
                i0, i1 = idx[k], idx[k + 1]
                if 1 < i1 - i0 <= 7:
                    for m in range(1, i1 - i0):
                        out[i0 + m] = out[i0] + (out[i1] - out[i0]) * m / (i1 - i0)
        msg.ranges = out
        msg.header.frame_id = "laser_link"
        pub.publish(msg)

    node.create_subscription(LaserScan, "/scan", on_scan, 5)
    rclpy.spin(node)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Record map->laser_link pose together with /height, for correlating drift with altitude."""
import argparse
import math
import time

import rclpy
import tf2_ros
from std_msgs.msg import Int16


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=85.0)
    ap.add_argument("--out", default="/tmp/tf_height.csv")
    a = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("tf_height_rec")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    state = {"h": float("nan")}

    def on_h(m):
        state["h"] = float(m.data)

    node.create_subscription(Int16, "/height", on_h, 10)
    t0 = time.monotonic()
    with open(a.out, "w", buffering=1) as fh:
        fh.write("t,x,y,yaw,h\n")
        last = 0.0
        while time.monotonic() - t0 < a.duration:
            rclpy.spin_once(node, timeout_sec=0.01)
            now = time.monotonic()
            if now - last < 0.05:
                continue
            last = now
            try:
                tr = buf.lookup_transform("map", "laser_link", rclpy.time.Time())
                p = tr.transform.translation
                q = tr.transform.rotation
                yaw = math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                              1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
                fh.write("%.3f,%.2f,%.2f,%.2f,%.1f\n"
                         % (now - t0, p.x * 100, p.y * 100, yaw, state["h"]))
            except Exception:
                pass
    node.destroy_node()
    rclpy.shutdown()
    print("done", a.out)


if __name__ == "__main__":
    main()

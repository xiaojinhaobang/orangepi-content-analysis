#!/usr/bin/env python3
"""起飞前体检（单进程版，约 3 秒）。只读，不改变任何状态。

检查：话题频率、TF 链、发布者数量、关键参数、位姿稳定性、残留进程。
退出码 0 = 全通过。
"""
import math
import subprocess
import sys
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import Vector3
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, Int16


class Preflight(rclpy.node.Node):
    def __init__(self):
        super().__init__("preflight_fast")
        self.n = {"scan": 0, "fcvel": 0, "height": 0, "tgt_pos": 0, "tgt_vel": 0}
        self.create_subscription(LaserScan, "/scan", lambda m: self.bump("scan"), 10)
        self.create_subscription(Vector3, "/fc_vel", lambda m: self.bump("fcvel"), 50)
        self.create_subscription(Int16, "/height", lambda m: self.bump("height"), 50)
        self.create_subscription(Float32MultiArray, "/target_position",
                                 lambda m: self.bump("tgt_pos"), 10)
        self.create_subscription(Float32MultiArray, "/target_velocity",
                                 lambda m: self.bump("tgt_vel"), 10)
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf, self)

    def bump(self, k):
        self.n[k] += 1

    def tf(self, a, b):
        try:
            t = self.buf.lookup_transform(a, b, rclpy.time.Time())
            p = t.transform.translation
            return (p.x * 100.0, p.y * 100.0)
        except Exception:
            return None


def main():
    rclpy.init()
    node = Preflight()
    t0 = time.monotonic()
    samples = []
    while time.monotonic() - t0 < 2.5:
        rclpy.spin_once(node, timeout_sec=0.02)
        p = node.tf("map", "dr_base")
        if p:
            samples.append(p)
    dur = time.monotonic() - t0

    fail = []

    def rep(cond, msg, detail=""):
        if cond:
            print("  PASS  %s %s" % (msg, detail))
        else:
            print("  FAIL  %s %s" % (msg, detail))
            fail.append(msg)

    print("=== 频率（%.1fs 窗口）===" % dur)
    rep(node.n["scan"] / dur >= 10, "/scan", "%.1f Hz" % (node.n["scan"] / dur))
    rep(node.n["fcvel"] / dur >= 100, "/fc_vel", "%.0f Hz" % (node.n["fcvel"] / dur))
    rep(node.n["height"] / dur >= 100, "/height", "%.0f Hz" % (node.n["height"] / dur))

    print("=== TF 链 ===")
    for a, b in (("map", "odom"), ("odom", "base_link"), ("map", "base_link"),
                 ("map", "dr_base"), ("map", "laser_link")):
        v = node.tf(a, b)
        rep(v is not None, "%s->%s" % (a, b),
            "(%.1f, %.1f) cm" % v if v else "取不到")
    lb = node.tf("map", "base_link")
    ld = node.tf("map", "dr_base")
    if lb and ld:
        d = math.hypot(lb[0] - ld[0], lb[1] - ld[1])
        rep(d < 5.0, "dr_base 跟随 base_link", "差 %.1f cm" % d)

    print("=== 发布者 ===")
    nv = node.count_publishers("/target_velocity")
    rep(nv == 1, "/target_velocity", "%d 个" % nv)
    npos = node.count_publishers("/target_position")
    print("  INFO  /target_position publisher 数 %d" % npos)
    rep(node.n["tgt_pos"] == 0, "起飞前无持续目标", "%d 帧" % node.n["tgt_pos"])

    print("=== 参数 ===")
    cli = node.create_client(GetParameters, "/dead_reckon/get_parameters")
    if cli.wait_for_service(timeout_sec=1.5):
        req = GetParameters.Request()
        req.names = ["passthrough", "gate_enable"]
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=1.5)
        vals = fut.result().values if fut.done() and fut.result() else []
        pt = vals[0].bool_value if len(vals) > 0 else None
        ge = vals[1].bool_value if len(vals) > 1 else None
        rep(pt is True, "passthrough=True", str(pt))
        rep(ge is False, "gate_enable=False", str(ge))
    else:
        rep(False, "读取 /dead_reckon 参数", "服务不可用")

    print("=== 位姿稳定性（%.1fs）===" % dur)
    if len(samples) > 20:
        xs = [p[0] for p in samples]
        ys = [p[1] for p in samples]
        pp = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        net = math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])
        rep(pp < 5.0, "位姿峰峰 <5cm", "%.1f cm（净 %.1f）" % (pp, net))
    else:
        rep(False, "位姿样本", "%d 个" % len(samples))

    print("=== 残留进程 ===")
    for pat, must_be_zero in (("scan_shift", True), ("ros2 bag play", True),
                              ("pos_hold", True)):
        out = subprocess.run(["pgrep", "-fc", pat], capture_output=True, text=True).stdout.strip() or "0"
        cnt = int(out.splitlines()[0]) if out.splitlines() else 0
        if must_be_zero:
            rep(cnt == 0, "无 %s" % pat, "%d 个" % cnt)
        else:
            print("  INFO  %s %d 个" % (pat, cnt))

    print("=== 结论 ===")
    print("全部通过" if not fail else "%d 项失败，不要起飞" % len(fail))
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if not fail else 1)


if __name__ == "__main__":
    main()

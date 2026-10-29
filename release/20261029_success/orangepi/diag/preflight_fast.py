#!/usr/bin/env python3
"""起飞前体检（单进程版，约 3 秒）。只读，不改变任何状态。

检查：话题频率、TF 链、发布者数量、关键参数、位姿稳定性、残留进程。
退出码 0 = 全通过。
"""
import math
import re
import subprocess
import sys
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import Twist, Vector3
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, Int16


class Preflight(rclpy.node.Node):
    def __init__(self):
        super().__init__("preflight_fast")
        self.n = {"scan": 0, "fcvel": 0, "height": 0, "tgt_pos": 0,
                  "tgt_vel": 0, "velf": 0}
        self.create_subscription(LaserScan, "/scan", lambda m: self.bump("scan"), 10)
        self.create_subscription(Vector3, "/fc_vel", lambda m: self.bump("fcvel"), 50)
        self.create_subscription(Int16, "/height", lambda m: self.bump("height"), 50)
        # ★ 2026-10-19：注入飞控的 0x32 速度观测。它停了/恒 0，飞控自己的定点
        #   就瞎了 —— 19:57 / 20:02 两次飘的直接原因之一。
        self.create_subscription(Twist, "/velocity_map_filtered",
                                 lambda m: self.bump("velf"), 50)
        self.create_subscription(Float32MultiArray, "/target_position",
                                 lambda m: self.bump("tgt_pos"), 10)
        self.create_subscription(Float32MultiArray, "/target_velocity",
                                 lambda m: self.bump("tgt_vel"), 10)
        # YAW_CONSISTENCY: 罗盘航向（飞控） vs SLAM 航向
        self.fc_yaw = None
        self.create_subscription(Float32MultiArray, "/fc_attitude_euler",
                                 self.on_euler, 20)
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf, self)

    def bump(self, k):
        self.n[k] += 1

    def on_euler(self, m):
        if len(m.data) >= 3:
            self.fc_yaw = float(m.data[2])

    def tf(self, a, b):
        try:
            t = self.buf.lookup_transform(a, b, rclpy.time.Time())
            p = t.transform.translation
            q = t.transform.rotation
            yaw = math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                          1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
            return (p.x * 100.0, p.y * 100.0, yaw)
        except Exception:
            return None


def main():
    rclpy.init()
    node = Preflight()
    t0 = time.monotonic()
    samples = []
    yaw_diffs = []
    while time.monotonic() - t0 < 3.5:
        rclpy.spin_once(node, timeout_sec=0.02)
        p = node.tf("map", "dr_base")
        if p:
            samples.append(p)
            if node.fc_yaw is not None:
                d = (node.fc_yaw - p[2] + 180.0) % 360.0 - 180.0
                yaw_diffs.append(d)
    dur = time.monotonic() - t0

    fail = []

    def rep(cond, msg, detail=""):
        if cond:
            print("  PASS  %s %s" % (msg, detail))
        else:
            print("  FAIL  %s %s" % (msg, detail))
            fail.append(msg)

    print("=== 频率（%.1fs 窗口）===" % dur)
    # Python 侧数 1200 点的扫描会漏帧，取 cartographer 日志里的权威速率
    auth = {}
    try:
        out = subprocess.run(["journalctl", "-u", "flight-link", "-n", "300", "-o", "cat",
                              "--no-pager"], capture_output=True, text=True, timeout=5).stdout
        for line in out.splitlines():
            m = re.search(r"(scan|imu|odom) rate: ([\d.]+) Hz", line)
            if m:
                auth[m.group(1)] = float(m.group(2))
    except Exception:
        pass
    scan_hz = auth.get("scan", node.n["scan"] / dur)
    rep(scan_hz >= 10, "/scan", "%.1f Hz%s" % (scan_hz, " (cartographer 日志)" if "scan" in auth else ""))
    if "imu" in auth:
        rep(auth["imu"] >= 10, "/imu (cartographer 侧)", "%.1f Hz" % auth["imu"])
    if "odom" in auth:
        rep(auth["odom"] >= 5, "/odom (cartographer 侧)", "%.1f Hz" % auth["odom"])
    rep(node.n["fcvel"] / dur >= 20, "/fc_vel", "%.0f Hz" % (node.n["fcvel"] / dur))
    rep(node.n["height"] / dur >= 20, "/height", "%.0f Hz" % (node.n["height"] / dur))
    rep(node.n["velf"] / dur >= 5, "/velocity_map_filtered（0x32 观测）",
        "%.1f Hz" % (node.n["velf"] / dur))

    print("=== TF 链 ===")
    for a, b in (("map", "odom"), ("odom", "base_link"), ("map", "base_link"),
                 ("map", "dr_base"), ("map", "laser_link")):
        v = node.tf(a, b)
        rep(v is not None, "%s->%s" % (a, b),
            "(%.1f, %.1f) cm" % (v[0], v[1]) if v else "取不到")
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

    # ★ 2026-10-19：open_loop 必须 False。
    #   True 时 velocity_filter 固定发 0，uart 把它当 0x32 观测灌给飞控 →
    #   飞控"看不见自己在动"，自身定点等于关掉（19:57 / 20:02 两次飘）。
    cli = node.create_client(GetParameters, "/velocity_filter/get_parameters")
    if cli.wait_for_service(timeout_sec=1.5):
        req = GetParameters.Request()
        req.names = ["open_loop"]
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=1.5)
        vals = fut.result().values if fut.done() and fut.result() else []
        ok_type = (len(vals) > 0
                   and vals[0].type == ParameterType.PARAMETER_BOOL)
        ol = vals[0].bool_value if ok_type else None
        rep(ok_type and ol is False, "velocity_filter open_loop=False", str(ol))
    else:
        rep(False, "读取 /velocity_filter open_loop", "服务不可用")

    print("=== 位姿稳定性（%.1fs）===" % dur)
    if len(samples) > 20:
        xs = [p[0] for p in samples]
        ys = [p[1] for p in samples]
        pp = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        net = math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])
        rep(pp < 5.0, "位姿峰峰 <5cm", "%.1f cm（净 %.1f）" % (pp, net))
    else:
        rep(False, "位姿样本", "%d 个" % len(samples))

    print("=== 罗盘 vs SLAM 航向 ===")
    if len(yaw_diffs) > 20:
        span = max(yaw_diffs) - min(yaw_diffs)
        rep(span <= 3.0, "航向差稳定（2.5s 内变化 <=3°）",
            "范围 %.1f°（%.1f ~ %.1f）" % (span, min(yaw_diffs), max(yaw_diffs)))
    else:
        print("  INFO  无 /fc_attitude_euler 数据，跳过")

    print("=== 残留进程 ===")
    # 模式里用 [x] 写法：既能匹配进程名，又不会匹配到"正在执行这段检查"的
    # shell 命令行本身（否则从含这些词的 ssh/bash 里跑会 100% 误报）。
    # 后缀限定 .py：避免匹配到"命令行里提到这些名字"的 shell（例如体检本身
    # 从一条包含 pos_hold 字样的 ssh 命令里启动时会误报）。
    for pat, must_be_zero in (("[s]can_shift.py", True), ("[r]os2 bag play", True),
                              ("[p]os_hold.py", True), ("[h]over_test.py", True),
                              ("[w]aypoint_nav.py", True)):
        out = subprocess.run(["pgrep", "-fc", pat], capture_output=True, text=True).stdout.strip() or "0"
        cnt = int(out.splitlines()[0]) if out.splitlines() else 0
        label = pat.replace("[", "").replace("]", "")
        if must_be_zero:
            rep(cnt == 0, "无 %s" % label, "%d 个" % cnt)
        else:
            print("  INFO  %s %d 个" % (label, cnt))

    print("=== 结论 ===")
    print("全部通过" if not fail else "%d 项失败，不要起飞" % len(fail))
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if not fail else 1)


if __name__ == "__main__":
    main()


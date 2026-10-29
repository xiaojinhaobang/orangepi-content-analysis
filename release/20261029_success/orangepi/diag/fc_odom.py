#!/usr/bin/env python3
"""Integrate the flight controller's own velocity (/fc_vel, body frame, cm/s)
into an odom->base_link transform, so cartographer gets a real motion prior.

Why: with use_imu_data=false and use_odometry=false, cartographer's
PoseExtrapolator has nothing but its own last two scan matches, so one bad match
becomes a constant-velocity extrapolation (measured: 1 m carried -> 97 m pose).
"""
import argparse
import math
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Vector3
from nav_msgs.msg import Odometry


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rate", type=float, default=50.0)
    a = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("fc_odom")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    br = tf2_ros.TransformBroadcaster(node)
    odom_pub = node.create_publisher(Odometry, "/odom", 20)
    st = {"vx": 0.0, "vy": 0.0, "t": 0.0, "x": 0.0, "y": 0.0, "yaw": 0.0,
          "yaw0": None, "has_vel": False, "last": time.monotonic()}

    def on_vel(m):
        st["vx"], st["vy"] = m.x, m.y      # cm/s, body frame
        st["t"] = time.monotonic()
        st["has_vel"] = True

    node.create_subscription(Vector3, "/fc_vel", on_vel, 20)

    def tick():
        now = time.monotonic()
        dt = min(max(now - st["last"], 0.0), 0.1)
        st["last"] = now
        # 航向取当前地图系航向（短时使用，够用）；拿不到就沿用上一次
        try:
            tr = buf.lookup_transform("map", "laser_link", rclpy.time.Time())
            q = tr.transform.rotation
            st["yaw"] = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            if st["yaw0"] is None:
                st["yaw0"] = st["yaw"]
        except Exception:
            pass
        fresh = st["has_vel"] and (now - st["t"]) < 0.3
        if fresh:
            c, s = math.cos(st["yaw"]), math.sin(st["yaw"])
            vx_map = (st["vx"] * c - st["vy"] * s) / 100.0
            vy_map = (st["vx"] * s + st["vy"] * c) / 100.0
            if abs(vx_map) < 2.0 and abs(vy_map) < 2.0:
                st["x"] += vx_map * dt
                st["y"] += vy_map * dt
        tr = TransformStamped()
        tr.header.stamp = node.get_clock().now().to_msg()
        tr.header.frame_id = "odom"
        tr.child_frame_id = "base_link"
        tr.transform.translation.x = st["x"]
        tr.transform.translation.y = st["y"]
        yaw_rel = (st["yaw"] - st["yaw0"]) if st["yaw0"] is not None else 0.0
        tr.transform.rotation.z = math.sin(yaw_rel / 2.0)
        tr.transform.rotation.w = math.cos(yaw_rel / 2.0)
        if False:  # TF 交给 cartographer 发，避免 odom->base_link 冲突
            br.sendTransform(tr)
        # cartographer 的 use_odometry 读的是 /odom 话题（nav_msgs/Odometry）
        od = Odometry()
        od.header.stamp = tr.header.stamp
        od.header.frame_id = "odom"
        od.child_frame_id = "base_link"
        od.pose.pose.position.x = tr.transform.translation.x
        od.pose.pose.position.y = tr.transform.translation.y
        od.pose.pose.orientation.z = tr.transform.rotation.z
        od.pose.pose.orientation.w = tr.transform.rotation.w
        od.twist.twist.linear.x = st["vx"] / 100.0
        od.twist.twist.linear.y = st["vy"] / 100.0
        odom_pub.publish(od)

    node.create_timer(1.0 / a.rate, tick)
    node.get_logger().info("fc_odom: odom->base_link from /fc_vel @%.0fHz" % a.rate)
    rclpy.spin(node)


if __name__ == "__main__":
    main()


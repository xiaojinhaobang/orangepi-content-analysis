#!/usr/bin/env python3
"""Record map/odom poses and radar velocity together for a fair comparison.

Run on the Orange Pi while the aircraft is unarmed. Move the airframe by hand
slowly; do not spin motors.
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
import tf2_ros


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Recorder(Node):
    def __init__(self, path):
        super().__init__('tf_velocity_compare')
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf, self)
        self.vel = (float('nan'), float('nan'))
        self.dr = (float('nan'), float('nan'))
        self.create_subscription(Twist, '/velocity_map', self.on_vel, 20)
        self.create_subscription(PoseStamped, '/dead_reckon/pose', self.on_dr, 20)
        self.f = open(path, 'w', buffering=1)
        self.f.write('t,map_x,map_y,map_yaw,odom_x,odom_y,odom_yaw,vel_x,vel_y,dr_x,dr_y\n')
        self.t0 = time.monotonic()
        self.last = 0.0
        self.create_timer(0.02, self.tick)
        print('READY: move the airframe by hand, motors unarmed', flush=True)

    def on_vel(self, m):
        self.vel = (m.linear.x * 100.0, m.linear.y * 100.0)

    def on_dr(self, m):
        self.dr = (m.pose.position.x * 100.0, m.pose.position.y * 100.0)

    def look(self, parent, child):
        try:
            tr = self.buf.lookup_transform(parent, child, rclpy.time.Time())
            p = tr.transform.translation
            q = tr.transform.rotation
            return (p.x * 100.0, p.y * 100.0, math.degrees(yaw_of(q)))
        except Exception:
            return (float('nan'), float('nan'), float('nan'))

    def tick(self):
        now = time.monotonic()
        if now - self.last < 0.05:
            return
        self.last = now
        mp = self.look('map', 'base_link')
        od = self.look('odom', 'base_link')
        self.f.write('%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n'
                     % (now - self.t0, mp[0], mp[1], mp[2],
                        od[0], od[1], od[2], self.vel[0], self.vel[1],
                        self.dr[0], self.dr[1]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=60.0)
    ap.add_argument('--out', default='/tmp/tf_vel_compare.csv')
    a = ap.parse_args()
    rclpy.init()
    n = Recorder(a.out)
    try:
        while rclpy.ok() and time.monotonic() - n.t0 < a.duration:
            rclpy.spin_once(n, timeout_sec=0.05)
    finally:
        n.f.close()
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print('DONE %s' % a.out, flush=True)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""在香橙派上测 /height 订阅能收到多少帧（排查网页版 alive=false）。"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

rclpy.init()
n = Node('hz_test')
box = {'n': 0, 'last': None}


def cb(m):
    box['n'] += 1
    box['last'] = int(m.data)


n.create_subscription(Int16, '/height', cb, 5)
t0 = time.time()
while time.time() - t0 < 6.0:
    rclpy.spin_once(n, timeout_sec=0.2)
print('6 秒收到 %d 帧 /height，最后值=%s' % (box['n'], box['last']))
rclpy.shutdown()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""等飞控解锁，一解锁就自动触发悬停自测（最多等 120 秒）。

用法（在香橙派上）： python3 /tmp/wait_unlock_fly.py [起飞高度] [悬停高度] [悬停秒数]
"""
import json
import sys
import time
import urllib.request

TAKEOFF = int(sys.argv[1]) if len(sys.argv) > 1 else 150
HOVER_Z = int(sys.argv[2]) if len(sys.argv) > 2 else 118
HOVER_S = float(sys.argv[3]) if len(sys.argv) > 3 else 12.0
BASE = 'http://localhost:8090'


def get(path):
    with urllib.request.urlopen(BASE + path, timeout=5) as r:
        return json.loads(r.read().decode())


def post(path, body):
    req = urllib.request.Request(BASE + path, data=json.dumps(body).encode(),
                                 headers={'Content-Type': 'application/json'}, method='POST')
    with urllib.request.urlopen(req, timeout=15) as r:
        return json.loads(r.read().decode())


for i in range(120):
    try:
        d = get('/api/status')
    except Exception as e:  # noqa: BLE001
        print('状态查询失败:', e)
        time.sleep(2)
        continue
    fc = d.get('fc') or []
    if len(fc) >= 2 and fc[1] == 1:
        print('检测到已解锁（模式 %d），第 %d 秒 → 触发悬停自测' % (fc[0], i))
        print(post('/api/hover_test', {'takeoff': TAKEOFF, 'hover_z': HOVER_Z,
                                       'hover_s': HOVER_S}))
        break
    if i % 10 == 0:
        print('  等解锁… %ds（当前 fc=%s ready=%s）' % (i, fc, d.get('ready')))
    time.sleep(1)
else:
    print('超时 120 秒仍未解锁，本次不飞')

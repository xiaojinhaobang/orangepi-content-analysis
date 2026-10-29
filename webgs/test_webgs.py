#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""网页版地面站自检（无界面、不起浏览器）。

    python test_webgs.py

覆盖：协议帧解析 → 事件落格与参考货号对照 → 阶段推算 → 模拟全流程 12 格 →
      UDP 双向（发 START / 收 EVENT）→ HTTP 页面与 API 可用。
"""
import json
import socket
import sys
import threading
import time
import urllib.request
from http.server import ThreadingHTTPServer

import server as gs
from protocol import build_event, build_status, pack_frame, TYPE_EVENT, TYPE_STATUS

FAILS = []


def check(name, cond, extra=''):
    print(('  [OK]  ' if cond else '  [FAIL]') + ' ' + name + ('  ' + str(extra) if extra else ''))
    if not cond:
        FAILS.append(name)


def post(port, path, body):
    req = urllib.request.Request(
        'http://127.0.0.1:%d%s' % (port, path),
        data=json.dumps(body or {}).encode(),
        headers={'Content-Type': 'application/json'}, method='POST')
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.loads(r.read().decode())


def get(port, path):
    with urllib.request.urlopen('http://127.0.0.1:%d%s' % (port, path), timeout=5) as r:
        return r.read()


def main():
    print('1) 起 HTTP 服务')
    srv = ThreadingHTTPServer(('127.0.0.1', 0), gs.Handler)
    port = srv.server_address[1]
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    html = get(port, '/').decode()
    check('页面 / 可访问且是完整 HTML', '<title>' in html and len(html) > 2000, '%d 字节' % len(html))
    s0 = json.loads(get(port, '/api/state').decode())
    check('空状态：12 格全空、阶段=准备', s0['done'] == 0 and s0['stage'] == 0 and len(s0['cells']) == 12)
    check('参考货号已带出', s0['cells'][0]['ref'] == 13 and s0['cells'][6]['ref'] == 8,
          'A1=%s B1=%s' % (s0['cells'][0]['ref'], s0['cells'][6]['ref']))

    print('2) UDP 通路（真的起一个监听，再发 2 帧进去）')
    gs.LINK = gs.UdpLink('127.0.0.1', 8899)      # 绑定 8899 收，回发到 127.0.0.1:8899
    gs.LINK.start()
    time.sleep(0.5)
    up = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    up.sendto(pack_frame(TYPE_STATUS, build_status(2, 118, 88)), ('127.0.0.1', 8899))
    up.sendto(pack_frame(TYPE_EVENT, build_event(23, 1, 6, 1234)), ('127.0.0.1', 8899))
    time.sleep(0.8)
    su = json.loads(get(port, '/api/state').decode())
    b6u = [c for c in su['cells'] if c['name'] == 'B6'][0]
    check('UDP 收到的 EVENT 落格（B6=23，与参考 6 不同 → 标 ≠）',
          b6u['num'] == 23 and b6u['ref'] == 6, b6u)
    check('UDP 收到的 STATUS 更新高度', su['drone']['height'] == 118 and su['drone']['state'] == 2,
          su['drone'])
    gs.LINK.stop()
    gs.LINK = None
    post(port, '/api/reset', {})

    print('3) 内置模拟全流程')
    post(port, '/api/sim', {'on': True, 'interval_ms': 80})
    s = None
    for _ in range(80):
        s = json.loads(get(port, '/api/state').decode())
        if s['done'] >= 12:
            break
        time.sleep(0.2)
    check('模拟跑完 12 格', s['done'] == 12, 'done=%s' % s['done'])
    check('阶段推进到"结果核对"', s['stage'] == 5, 'stage=%s' % s['stage'])
    b6 = [c for c in s['cells'] if c['name'] == 'B6'][0]
    check('B6 货号=参考值 6 且标记一致', b6['num'] == 6 and b6['num'] == b6['ref'], b6)
    check('日志有记录', len(s['log']) > 10, '%d 条' % len(s['log']))

    print('4) 查询 / 复位 / 开始任务（未连链路时应给提示）')
    r = post(port, '/api/query', {'num': 13})
    check('按货号查坐标', 'A1' in r['message'], r['message'])
    r = post(port, '/api/start', {'mode': 0})
    check('未连链路时开始任务给提示而不是静默', '未连接' in r['message'], r['message'])
    post(port, '/api/sim', {'on': False})
    r = post(port, '/api/reset', {})
    s2 = json.loads(get(port, '/api/state').decode())
    check('复位后归零', s2['done'] == 0 and s2['drone']['height'] == 0)

    srv.shutdown()
    print()
    if FAILS:
        print('自检失败 %d 项: %s' % (len(FAILS), FAILS))
        sys.exit(1)
    print('网页版地面站自检全部通过 [OK]')


if __name__ == '__main__':
    main()

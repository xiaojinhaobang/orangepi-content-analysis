#!/usr/bin/env python3
"""Listen on a UDP port and decode ground-station protocol frames."""

import socket
import struct
import sys
import time

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8888
SECONDS = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0
NAMES = {0x01: 'EVENT', 0x02: 'STATUS', 0x03: 'ROUTE', 0x04: 'REPORT',
         0x10: 'QUERY', 0x11: 'START'}

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('0.0.0.0', PORT))
sock.settimeout(1.0)

start = time.time()
count = 0
while time.time() - start < SECONDS:
    try:
        data, addr = sock.recvfrom(2048)
    except socket.timeout:
        continue
    count += 1
    if data[:2] != b'\xAA\x55':
        print('  [%d] %s 坏帧 %r' % (count, addr, data[:8]))
        continue
    ftype, plen = data[2], data[3]
    payload = data[4:4 + plen]
    ok = (ftype + plen + sum(payload)) & 0xFF == data[4 + plen]
    extra = ''
    if ftype == 0x02 and plen >= 4:
        st, h, batt = struct.unpack('<BHB', payload)
        extra = ' state=%d height=%dcm batt=%d%%' % (st, h, batt)
    print('  [%d] from %s  %s len=%d sum_ok=%s%s'
          % (count, addr, NAMES.get(ftype, hex(ftype)), plen, ok, extra))

print('共收到 %d 帧（%d 秒）' % (count, SECONDS))

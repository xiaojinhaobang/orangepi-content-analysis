"""最小复现：bind 0.0.0.0:8888，在**子线程**里 recvfrom，看能不能收到板子的帧。

用来判断"地面站程序收不到帧"到底是线程的问题、还是别的。
用法: python tools/test_udp_thread.py [秒数]
"""

import socket
import sys
import threading
import time

secs = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0
poke = "--poke" in sys.argv
BOARD = "192.168.137.186"
PORT = 8899 if "--port8899" in sys.argv else 8888
HEAD = b"\xAA\x55"


def pack_frame(ftype, payload=b""):
    body = bytes([ftype, len(payload)]) + payload
    return HEAD + body + bytes([(ftype + len(payload) + sum(payload)) & 0xFF])

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
BIND = "192.168.137.1" if "--bindhotspot" in sys.argv else "0.0.0.0"
sock.bind((BIND, PORT))
print("bound %s:%d" % (BIND, PORT))

n = [0]
stop = [False]


def loop():
    while not stop[0]:
        try:
            data, addr = sock.recvfrom(2048)
        except OSError:
            return
        n[0] += 1
        if n[0] == 1:
            print("first packet from %s, %d bytes" % (addr, len(data)))


th = threading.Thread(target=loop, daemon=True)
th.start()

if poke:
    def keepalive():
        while not stop[0]:
            try:
                sock.sendto(pack_frame(0x7F), (BOARD, PORT))
            except OSError as e:
                print("poke failed:", e)
            time.sleep(0.5)

    threading.Thread(target=keepalive, daemon=True).start()
    print("keepalive: 每 0.5s 给板子发一包 0x7F")

time.sleep(secs)
stop[0] = True
print("received %d packets in %.0fs" % (n[0], secs))

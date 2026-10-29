"""Loopback test for a UART: short TX to RX, then verify bytes come back.

Uses stty for line configuration (no pyserial on the board) and os.read/os.write
for raw I/O. Also verifies the flight-controller frame format
AA FF ID LEN DATA SUM ADD end to end.
"""

import argparse
import os
import random
import select
import subprocess
import time


def configure(port: str, baud: int) -> None:
    """Configure the line as raw 8N1 with no flow control."""
    subprocess.run(
        ["stty", "-F", port, str(baud), "raw", "-echo", "-crtscts",
         "cs8", "-parenb", "-cstopb", "min", "0", "time", "1"],
        check=True, capture_output=True,
    )


def drain(fd: int) -> None:
    """Discard anything already buffered."""
    while select.select([fd], [], [], 0.02)[0]:
        if not os.read(fd, 4096):
            break


def roundtrip(fd: int, payload: bytes, wait_s: float) -> bytes:
    """Write payload and collect whatever comes back."""
    drain(fd)
    start = time.perf_counter()
    os.write(fd, payload)
    received = bytearray()
    deadline = start + wait_s
    while time.perf_counter() < deadline and len(received) < len(payload):
        ready = select.select([fd], [], [], 0.05)[0]
        if ready:
            chunk = os.read(fd, 4096)
            if chunk:
                received.extend(chunk)
    return bytes(received)


def build_frame(frame_id: int, payload: bytes) -> bytes:
    """Build an AA FF ID LEN DATA SUM ADD frame (same algorithm as serial_comm)."""
    frame = bytearray([0xAA, 0xFF, frame_id, len(payload)])
    frame.extend(payload)
    checksum = 0
    add = 0
    for byte in frame:
        checksum = (checksum + byte) & 0xFF
        add = (add + checksum) & 0xFF
    frame.extend([checksum, add])
    return bytes(frame)


def report(name: str, sent: bytes, got: bytes) -> bool:
    """Print one test result."""
    if got == sent:
        print(f"  [PASS] {name}: {len(sent)} bytes 回环完全一致")
        return True
    if not got:
        print(f"  [FAIL] {name}: 发出 {len(sent)} 字节，收到 0 字节")
        return False
    mismatch = next((i for i, (a, b) in enumerate(zip(sent, got)) if a != b), None)
    print(f"  [FAIL] {name}: 发出 {len(sent)}，收到 {len(got)}，"
          f"首个不一致偏移={mismatch}")
    return False


def main() -> None:
    """Run the loopback suite."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyS3")
    parser.add_argument("--bauds", default="921600,500000,115200")
    parser.add_argument("--payload", type=int, default=256)
    args = parser.parse_args()

    bauds = [int(value) for value in args.bauds.split(",") if value.strip()]
    print(f"端口 {args.port}，回环测试（需短接 TX/RX）")
    print("  40pin: uart3 = 31(TX) / 33(RX)，uart4 = 23(TX) / 19(RX)")

    fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    wait_s = 1.0
    try:
        for baud in bauds:
            configure(args.port, baud)
            print(f"--- {baud} baud ---")

            report("单字节 0x55", b"\x55", roundtrip(fd, b"\x55", wait_s))

            payload = bytes(random.randrange(256) for _ in range(args.payload))
            start = time.perf_counter()
            got = roundtrip(fd, payload, wait_s + 0.5)
            elapsed = max(1e-6, time.perf_counter() - start)
            report(f"随机 {args.payload} 字节", payload, got)
            print(f"         耗时 {elapsed * 1000:.1f} ms "
                  f"(≈{len(got) * 10 / elapsed / 1000:.1f} kbps 有效)")

            frame = build_frame(0x31, bytes([0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]))
            print(f"         飞控帧 {frame.hex(' ').upper()}")
            report("飞控帧 0x31 回环", frame, roundtrip(fd, frame, wait_s))
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()

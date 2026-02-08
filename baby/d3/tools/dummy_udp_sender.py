#!/usr/bin/env python3
import socket
import struct
import time

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

WP_MAGIC = 0xA5
WP_TYPE_WAYPOINT = 1

PKT_FMT = "<BHBiiH"  # magic, type, exception, x_mm, y_mm, crc16

PHASES = [
    (0.5, True),   # 1) both on
    (0.5, False),  # 2) wifi off
    (0.5, False),  # 3) both off
    (0.5, False),  # 4) aruco on, wifi off
    (0.5, True),   # 5) wifi on
]

SEND_HZ = 20.0

def klog(msg: str) -> None:
    try:
        with open("/dev/kmsg", "w") as f:
            f.write(f"dummy_udp: {msg}\n")
    except Exception:
        pass
    print(f"[dummy_udp] {msg}", flush=True)

def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_pkt(exception: int, x_mm: int, y_mm: int) -> bytes:
    header = struct.pack("<BHBii", WP_MAGIC, WP_TYPE_WAYPOINT, exception, x_mm, y_mm)
    crc = crc16_ccitt_false(header)
    return struct.pack(PKT_FMT, WP_MAGIC, WP_TYPE_WAYPOINT, exception, x_mm, y_mm, crc)


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    pkt_idx = 0
    phase_idx = 0
    phase_start = time.time()

    klog(f"sending to {UDP_IP}:{UDP_PORT}")

    while True:
        now = time.time()
        if phase_idx < len(PHASES):
            dur, enabled = PHASES[phase_idx]
            if (now - phase_start) >= dur:
                phase_idx += 1
                phase_start = now
                continue
        else:
            enabled = True

        if enabled:
            x_mm = 1000 + (pkt_idx % 50) * 10
            y_mm = -500 + (pkt_idx % 40) * 5
            exception = pkt_idx % 4  # 0~3
            pkt = build_pkt(exception, x_mm, y_mm)
            sock.sendto(pkt, (UDP_IP, UDP_PORT))
            pkt_idx += 1

        time.sleep(1.0 / SEND_HZ)


if __name__ == "__main__":
    main()

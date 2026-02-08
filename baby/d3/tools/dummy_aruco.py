#!/usr/bin/env python3
import os
import socket
import struct
import time

SOCKET_PATH = "/tmp/aruco_socket"

# ArUcoRawPacket: int32 id, float dist_cm, float error_px, char steering (packed)
# C side is packed, so no padding. We'll send little-endian.
PKT_FMT = "<iffc"

PHASES = [
    (0.5, True),   # 1) both on
    (0.5, True),   # 2) wifi off, aruco on
    (0.5, False),  # 3) both off
    (0.5, True),   # 4) aruco on
    (0.5, True),   # 5) both on
]

SEND_HZ = 20.0

def klog(msg: str) -> None:
    try:
        with open("/dev/kmsg", "w") as f:
            f.write(f"dummy_aruco: {msg}\n")
    except Exception:
        pass
    print(f"[dummy_aruco] {msg}", flush=True)

def main():
    if os.path.exists(SOCKET_PATH):
        os.unlink(SOCKET_PATH)

    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(SOCKET_PATH)
    srv.listen(1)
    klog(f"listening {SOCKET_PATH}")

    conn, _ = srv.accept()
    klog("client connected")
    conn.setblocking(True)

    t0 = time.time()
    pkt_idx = 0
    phase_idx = 0
    phase_start = time.time()

    try:
        while True:
            now = time.time()
            # phase control
            if phase_idx < len(PHASES):
                dur, enabled = PHASES[phase_idx]
                if (now - phase_start) >= dur:
                    phase_idx += 1
                    phase_start = now
                    continue
            else:
                enabled = True

            if enabled:
                # simple oscillation
                dist_cm = 50.0 + 10.0 * (pkt_idx % 10)
                error_px = -100.0 + 20.0 * (pkt_idx % 11)
                pkt = struct.pack(PKT_FMT, 1, dist_cm, error_px, b'N')
                try:
                    conn.sendall(pkt)
                except BrokenPipeError:
                    klog("client disconnected")
                    break
                pkt_idx += 1

            time.sleep(1.0 / SEND_HZ)

    finally:
        conn.close()
        srv.close()
        if os.path.exists(SOCKET_PATH):
            os.unlink(SOCKET_PATH)


if __name__ == "__main__":
    main()

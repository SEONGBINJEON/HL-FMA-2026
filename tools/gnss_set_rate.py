#!/usr/bin/env python3
"""u-blox 수신기의 측위 주기를 바꾼다 (UBX-CFG-RATE).

RAM 에만 쓴다. 수신기 전원을 껐다 켜면 원래 설정으로 돌아오며, 영구 저장(CFG-CFG save)은
하지 않는다. 경로 추종은 1 Hz 로는 위치 갱신이 느려 자세가 만료되므로 10 Hz 로 올린다.

보내는 것은 CFG-RATE 한 프레임뿐이고 다른 설정(측위 모드, 보정 입력, 출력 문장)은 건드리지
않는다. ACK/NAK 를 확인해 보고한다.
"""
import argparse
import os
import re
import struct
import termios
import time


def ubx(cls, mid, payload=b""):
    """UBX 프레임 하나를 만든다 (Fletcher-8 체크섬)."""
    body = bytes([cls, mid]) + struct.pack("<H", len(payload)) + payload
    a = b = 0
    for x in body:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return b"\xb5\x62" + body + bytes([a, b])


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", required=True)
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--listen-s", type=float, default=3.0, help="변경 후 관찰 시간")
    a = ap.parse_args()

    meas_ms = int(round(1000.0 / a.hz))
    if not (50 <= meas_ms <= 10000):
        raise SystemExit(f"측위 주기 {meas_ms} ms 는 범위를 벗어난다")
    frame = ubx(0x06, 0x08, struct.pack("<HHH", meas_ms, 1, 1))  # measRate, navRate=1, timeRef=GPS

    fd = os.open(a.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        attrs = termios.tcgetattr(fd)
        attrs[0] = attrs[1] = attrs[3] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[4] = attrs[5] = getattr(termios, f"B{a.baud}")
        cc = list(attrs[6])
        cc[termios.VMIN] = 0
        cc[termios.VTIME] = 0
        attrs[6] = cc
        termios.tcsetattr(fd, termios.TCSANOW, attrs)

        os.write(fd, frame)
        buf = b""
        t0 = time.monotonic()
        while time.monotonic() - t0 < a.listen_s:
            try:
                chunk = os.read(fd, 4096)
                if chunk:
                    buf += chunk
            except BlockingIOError:
                pass
            time.sleep(0.02)

        ack = b"\xb5\x62\x05\x01" in buf
        nak = b"\xb5\x62\x05\x00" in buf
        gga = len(re.findall(rb"\$G[NP]GGA", buf))
        print(f"요청: 측위 주기 {meas_ms} ms = {1000 / meas_ms:.1f} Hz (RAM)")
        print(f"보낸 프레임: {frame.hex()}")
        print(f"ACK {ack} | NAK {nak} | 수신 {len(buf)}바이트 | 관찰 {a.listen_s:.0f}초 동안 GGA {gga}개"
              f" (= 약 {gga / a.listen_s:.1f} Hz)")
        return 1 if nak else 0
    finally:
        os.close(fd)


if __name__ == "__main__":
    raise SystemExit(main())

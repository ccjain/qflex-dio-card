"""One-shot CLI to turn a DIO Card relay on or off via Modbus FC05.

Usage:
    py tests/relay.py <relay 1-7> <on|off> [--port COM5] [--slave 1]

Exit codes:
    0  success (FC05 echo matched request)
    1  operational failure (cannot open port, no reply, wrong echo)
    2  usage error (bad args)
"""

from __future__ import annotations

import argparse
import sys
import time

import serial


BAUD = 9600
RESP_TIMEOUT = 0.25     # 250 ms -- generous for one FC05 round-trip @ 9600 8N1
DRAIN_TAIL = 0.05       # keep reading 50 ms after first byte to catch trailing CRC


def crc16_modbus(data: bytes) -> bytes:
    """Modbus RTU CRC-16 (poly 0xA001, seed 0xFFFF, little-endian out).

    Duplicated from tests/run_bench_smoke.py -- kept inline so this tool
    has no dependency on the test harness.
    """
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def build_fc05_frame(slave: int, relay: int, on: bool) -> bytes:
    """Build a Modbus FC05 (Write Single Coil) frame.

    Coil address = relay - 1 (relay 1 -> coil 0, relay 7 -> coil 6).
    Value = 0xFF00 to energise, 0x0000 to de-energise (per FC05 spec).
    """
    coil = relay - 1
    value_hi = 0xFF if on else 0x00
    body = bytes([slave, 0x05, 0x00, coil, value_hi, 0x00])
    return body + crc16_modbus(body)


def hex_bytes(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)


def parse_args(argv: list[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        prog="relay.py",
        description="Turn a DIO Card relay on or off (Modbus FC05).",
    )
    p.add_argument("relay", type=int, help="relay number 1..7")
    p.add_argument("action", type=str, help="on or off")
    p.add_argument("--port", default="COM5", help="serial port (default COM5)")
    p.add_argument("--slave", type=int, default=1,
                   help="Modbus slave id 1..247 (default 1)")
    args = p.parse_args(argv)

    args.action = args.action.lower()
    if args.relay < 1 or args.relay > 7:
        p.error(f"relay must be 1..7, got {args.relay}")
    if args.action not in ("on", "off"):
        p.error(f"action must be 'on' or 'off', got {args.action!r}")
    if args.slave < 1 or args.slave > 247:
        p.error(f"--slave must be 1..247, got {args.slave}")
    return args


def send_and_read(ser: serial.Serial, frame: bytes,
                  timeout: float = RESP_TIMEOUT) -> bytes:
    """Send `frame`, then read whatever the device responds with up to `timeout`.

    Mirrors the drain logic in tests/run_bench_smoke.py: after the first byte
    arrives, keep reading for DRAIN_TAIL seconds so a trailing CRC byte isn't
    cut off.
    """
    ser.reset_input_buffer()
    ser.write(frame)
    ser.flush()
    deadline = time.monotonic() + timeout
    buf = bytearray()
    while time.monotonic() < deadline:
        n = ser.in_waiting
        if n:
            buf.extend(ser.read(n))
            deadline = time.monotonic() + DRAIN_TAIL
        else:
            time.sleep(0.005)
        # FC05 echo is exactly 8 bytes -- stop early once we have it.
        if len(buf) >= 8:
            break
    return bytes(buf)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)

    on = (args.action == "on")
    tx = build_fc05_frame(args.slave, args.relay, on)

    try:
        ser = serial.Serial(args.port, BAUD, bytesize=8, parity="N",
                            stopbits=1, timeout=0)
    except (serial.SerialException, OSError, ValueError) as e:
        print(f"cannot open {args.port}: {e}", file=sys.stderr)
        return 1

    try:
        try:
            rx = send_and_read(ser, tx)
        except (serial.SerialException, OSError) as e:
            print(f"serial I/O error on {args.port}: {e}", file=sys.stderr)
            return 1
    finally:
        ser.close()

    if not rx:
        print(f"no reply from slave {args.slave} on {args.port}",
              file=sys.stderr)
        return 1

    if rx != tx:
        print("echo mismatch", file=sys.stderr)
        print(f"  TX: {hex_bytes(tx)}", file=sys.stderr)
        print(f"  RX: {hex_bytes(rx)}", file=sys.stderr)
        return 1

    print(f"relay {args.relay} -> {args.action.upper()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

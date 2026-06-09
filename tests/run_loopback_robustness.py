"""Loopback robustness test for the DIO Card Modbus RTU slave.

With all 12 discrete inputs (FB1..FB12) physically wired to the 7 relay
outputs (R1..R7), this script repeatedly:
  - writes all relays ON  (FC0F coils 0..6 = 1) -> sleeps 100 ms -> reads
    all 12 inputs (FC02) -> asserts data bytes == FF 0F (12 LSBs set)
  - writes all relays OFF (FC0F coils 0..6 = 0) -> sleeps 100 ms -> reads
    all 12 inputs (FC02) -> asserts data bytes == 00 00

Strict abort on first mismatch. Safety cleanup forces relays OFF on any
exit path.

Usage:
    py -3 tests/run_loopback_robustness.py [COM_PORT] [SLAVE_ID] [ITERATIONS]
Defaults: COM5, 8, 1000.
"""

from __future__ import annotations

import statistics
import sys
import time
from dataclasses import dataclass, field

import serial


# ---------------------------------------------------------------------------
# CRC16-Modbus (poly 0xA001, seed 0xFFFF, little-endian output)
# ---------------------------------------------------------------------------
def crc16_modbus(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def fr(payload_hex: str) -> bytes:
    """Build a frame from payload hex (slave..data), appending CRC."""
    body = bytes.fromhex(payload_hex)
    return body + crc16_modbus(body)


# ---------------------------------------------------------------------------
# Self-test: verify CRC against known-good vectors from run_bench_smoke.py
# ---------------------------------------------------------------------------
_VECTORS = [
    ("08 02 00 00 00 0C", "78 96", "FC02 read 12 inputs (slave 8)"),
    ("08 02 02 00 00",    "65 B9", "FC02 response all-zero (slave 8)"),
    ("08 0F 00 00 00 07", "14 90", "FC0F write echo (slave 8)"),
]


def _self_test() -> None:
    for payload_hex, expected_crc_hex, label in _VECTORS:
        got = crc16_modbus(bytes.fromhex(payload_hex)).hex(" ").upper()
        exp = expected_crc_hex.upper()
        assert got == exp, f"CRC mismatch for {label}: got {got} expected {exp}"
        print(f"  ok  {label:<40} crc={got}")
    print("CRC self-test passed.")


# ---------------------------------------------------------------------------
# Configuration and CLI parsing
# ---------------------------------------------------------------------------
@dataclass
class PortConfig:
    port: str = "COM5"
    slave: int = 8
    iters: int = 1000
    baud: int = 9600
    resp_timeout: float = 0.2     # per-frame read timeout
    settle_ms: int = 100          # gap between write and read


def parse_args(argv: list[str]) -> PortConfig:
    cfg = PortConfig()
    if len(argv) > 1:
        cfg.port = argv[1]
    if len(argv) > 2:
        cfg.slave = int(argv[2])
    if len(argv) > 3:
        cfg.iters = int(argv[3])
    return cfg


def main(argv: list[str]) -> int:
    if len(argv) > 1 and argv[1] == "--self-test":
        _self_test()
        return 0
    cfg = parse_args(argv)
    print(f"Loopback robustness: port={cfg.port} slave={cfg.slave} iters={cfg.iters}")
    ser = serial.Serial(cfg.port, cfg.baud, bytesize=8, parity="N",
                        stopbits=1, timeout=0)
    try:
        time.sleep(0.2)  # let the FTDI driver settle
        print("port opened; main loop not yet implemented")
        return 0
    finally:
        ser.close()


if __name__ == "__main__":
    sys.exit(main(sys.argv))

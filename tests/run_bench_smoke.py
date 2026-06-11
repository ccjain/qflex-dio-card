"""Bench smoke runner for the DIO Card Modbus RTU slave.

Replays every raw frame from tests/mbpoll_smoke.md against the device on
the given COM port and checks the response against the doc's expected
bytes. Read tests, write tests (relay clicks), exception paths, and the
wrong-slave timeout path are all exercised.

Usage:
    py -3 tests/run_bench_smoke.py [COM_PORT]
Default COM_PORT is COM5.
"""

from __future__ import annotations

import sys
import time
import serial


PORT = sys.argv[1] if len(sys.argv) > 1 else "COM5"
SLAVE = int(sys.argv[2]) if len(sys.argv) > 2 else 8
BAUD = 9600
RESP_TIMEOUT = 0.5         # generous for 9600 8N1
SILENT_TIMEOUT = 0.3       # used for "expect no reply" tests
INTERFRAME_GAP = 0.05      # >> 3.5 char times at 9600 (~4 ms)


def hex_bytes(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)


def crc16_modbus(data: bytes) -> bytes:
    """Modbus RTU CRC-16 (poly 0xA001, seed 0xFFFF, little-endian out)."""
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


def send(ser: serial.Serial, frame: bytes, *, read: int = 256,
         timeout: float = RESP_TIMEOUT) -> bytes:
    ser.reset_input_buffer()
    ser.write(frame)
    ser.flush()
    deadline = time.monotonic() + timeout
    chunks = bytearray()
    while time.monotonic() < deadline:
        n = ser.in_waiting
        if n:
            chunks.extend(ser.read(n))
            # keep draining for a bit in case the response trickles in
            deadline = time.monotonic() + 0.05
        else:
            time.sleep(0.01)
        if len(chunks) >= read:
            break
    return bytes(chunks)


def check(label: str, tx: bytes, rx: bytes, expect: bytes | None,
          *, expect_silent: bool = False) -> bool:
    print(f"\n[{label}]")
    print(f"  TX: {hex_bytes(tx)}")
    print(f"  RX: {hex_bytes(rx) if rx else '<no reply>'}")
    if expect_silent:
        ok = len(rx) == 0
        print(f"  expect: <no reply>     -> {'PASS' if ok else 'FAIL'}")
        return ok
    ok = rx == expect
    print(f"  expect: {hex_bytes(expect)} -> {'PASS' if ok else 'FAIL'}")
    return ok


def main() -> int:
    print(f"Opening {PORT} @ {BAUD} 8N1 ...")
    ser = serial.Serial(PORT, BAUD, bytesize=8, parity="N", stopbits=1,
                        timeout=0)
    time.sleep(0.2)
    results: list[tuple[str, bool]] = []

    def step(label: str, tx: bytes, expect: bytes | None = None, *,
             expect_silent: bool = False, timeout: float = RESP_TIMEOUT) -> None:
        rx = send(ser, tx, timeout=timeout)
        results.append((label, check(label, tx, rx, expect,
                                     expect_silent=expect_silent)))
        time.sleep(INTERFRAME_GAP)

    # Slave-aware helpers
    SID = f"{SLAVE:02X}"
    def f1(rest_hex: str) -> bytes:
        return fr(f"{SID} {rest_hex}")
    READ7 = f1("01 00 00 00 07")              # FC01 read coils 0..6
    READ12IN = f1("02 00 00 00 0C")           # FC02 read 12 inputs
    def coil_resp(data_byte: int) -> bytes:
        return f1(f"01 01 {data_byte:02X}")
    def input_resp_2(b0: int, b1: int) -> bytes:
        return f1(f"02 02 {b0:02X} {b1:02X}")
    wrong_slave = 1 if SLAVE != 1 else 2

    # ----- Coil reads / writes (FC 01 / 05 / 0F) -----------------------------
    # 1.1 Read all 7 coils - boot state should be all zero
    step("1.1 read 7 coils (boot)", READ7, coil_resp(0x00))

    # 1.2 Energise relay 1 -- echo back
    tx = f1("05 00 00 FF 00")
    step("1.2 FC05 relay1=ON", tx, tx)
    step("1.2v re-read coils -> coil1=1", READ7, coil_resp(0x01))

    # 1.3 De-energise relay 1 -- echo back
    tx = f1("05 00 00 00 00")
    step("1.3 FC05 relay1=OFF", tx, tx)

    # 1.4 FC0F multi-write 0x55 -> R1,R3,R5,R7 on
    step("1.4 FC0F pattern 0x55",
         f1("0F 00 00 00 07 01 55"),
         f1("0F 00 00 00 07"))
    step("1.4v re-read coils -> 0x55", READ7, coil_resp(0x55))

    # Cleanup: all relays off
    step("cleanup FC0F all-off",
         f1("0F 00 00 00 07 01 00"),
         f1("0F 00 00 00 07"))

    # 1.5 Illegal FC05 value -> exception 0x03
    step("1.5 FC05 illegal value 0x1234 -> ex03",
         f1("05 00 00 12 34"),
         f1("85 03"))

    # 1.6 Out-of-range coil 8 -> exception 0x02
    step("1.6 FC05 coil8 OOR -> ex02",
         f1("05 00 07 FF 00"),
         f1("85 02"))

    # ----- Discrete-input reads (FC 02) -------------------------------------
    # 2.1 Read all 12 feedbacks - inputs floating, all zero
    step("2.1 read 12 inputs (floating)", READ12IN, input_resp_2(0x00, 0x00))

    # 2.3 Out-of-range FC02 -> exception 0x02
    step("2.3 FC02 OOR addr 13 -> ex02",
         f1("02 00 0C 00 01"),
         f1("82 02"))

    # ----- Negative tests ---------------------------------------------------
    # 3.1 Wrong slave ID -> silent drop
    step(f"3.1 wrong slave id={wrong_slave} -> silent",
         fr(f"{wrong_slave:02X} 01 00 00 00 01"),
         expect_silent=True,
         timeout=SILENT_TIMEOUT)

    # 3.2 Unsupported FC 03 -> exception 0x01
    step("3.2 FC03 unsupported -> ex01",
         f1("03 00 00 00 01"),
         f1("83 01"))

    # ----- Summary ----------------------------------------------------------
    ser.close()
    print("\n" + "=" * 60)
    passed = sum(1 for _, ok in results if ok)
    for label, ok in results:
        mark = "PASS" if ok else "FAIL"
        print(f"  [{mark}] {label}")
    print(f"\n{passed}/{len(results)} tests passed")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())

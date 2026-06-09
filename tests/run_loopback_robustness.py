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

    frames = build_frames(8)
    assert frames["read_req"]   == bytes.fromhex("08 02 00 00 00 0C 78 96")
    assert frames["read_low"]   == bytes.fromhex("08 02 02 00 00 65 B9")
    assert frames["write_echo"] == bytes.fromhex("08 0F 00 00 00 07 14 90")
    print("  ok  build_frames(8) -> known-good wire bytes")

    assert classify(b"", frames["read_low"]) == "no_reply"
    assert classify(b"\x08\x02", frames["read_low"]) == "short"
    assert classify(frames["read_low"], frames["read_low"]) == "ok"
    assert classify(bytes.fromhex("08 02 02 00 01 A4 79"),
                    frames["read_low"]) == "wrong_data"
    assert classify(bytes.fromhex("08 02 02 00 00 00 00"),
                    frames["read_low"]) == "wrong_crc"
    assert classify(bytes.fromhex("09 02 02 00 00 6C 18"),
                    frames["read_low"]) == "wrong_slave_or_fc"
    print("  ok  classify() -> correct buckets for every failure mode")

    diff = per_bit_diff(bytes.fromhex("FE 0F"), bytes.fromhex("FF 0F"))
    assert diff == [0], f"per_bit_diff wrong: got {diff}"
    diff = per_bit_diff(bytes.fromhex("00 00"), bytes.fromhex("FF 0F"))
    assert diff == list(range(12)), f"per_bit_diff wrong: got {diff}"
    print("  ok  per_bit_diff() -> correct input indices")

    print("Self-test passed.")


# ---------------------------------------------------------------------------
# Frame templates — parameterised by slave ID
# ---------------------------------------------------------------------------
def build_frames(slave: int) -> dict[str, bytes]:
    sid = f"{slave:02X}"
    return {
        # Requests
        "write_on":   fr(f"{sid} 0F 00 00 00 07 01 7F"),  # all 7 coils -> 1
        "write_off":  fr(f"{sid} 0F 00 00 00 07 01 00"),  # all 7 coils -> 0
        "read_req":   fr(f"{sid} 02 00 00 00 0C"),        # FC02 12 inputs
        # Expected responses
        "write_echo": fr(f"{sid} 0F 00 00 00 07"),        # echo header only
        "read_high":  fr(f"{sid} 02 02 FF 0F"),           # 12 LSBs set
        "read_low":   fr(f"{sid} 02 02 00 00"),           # all bits clear
    }


# ---------------------------------------------------------------------------
# Response classification
# ---------------------------------------------------------------------------
def classify(rx: bytes, expected: bytes) -> str:
    """Return 'ok' on exact match, else one of the failure category strings."""
    if len(rx) == 0:
        return "no_reply"
    if len(rx) < len(expected):
        return "short"
    if rx == expected:
        return "ok"
    # Same length, mismatched — figure out which layer disagrees.
    if rx[:2] != expected[:2]:
        return "wrong_slave_or_fc"
    if crc16_modbus(rx[:-2]) != rx[-2:]:
        return "wrong_crc"
    return "wrong_data"


def per_bit_diff(actual_data: bytes, expected_data: bytes) -> list[int]:
    """Return list of input indices (0..11) whose bit differs from expected."""
    a = actual_data[0] | (actual_data[1] << 8)
    e = expected_data[0] | (expected_data[1] << 8)
    diff = a ^ e
    return [i for i in range(12) if (diff >> i) & 1]


# ---------------------------------------------------------------------------
# Serial I/O transaction helper
# ---------------------------------------------------------------------------
def transact(ser: serial.Serial, tx: bytes, want_bytes: int,
             timeout_s: float) -> tuple[bytes, float]:
    """Send tx, read up to want_bytes (drain on small bursts), return (rx, ms)."""
    ser.reset_input_buffer()
    t0 = time.perf_counter()
    ser.write(tx)
    ser.flush()
    deadline = t0 + timeout_s
    buf = bytearray()
    while time.perf_counter() < deadline:
        n = ser.in_waiting
        if n:
            buf.extend(ser.read(n))
            # extend deadline briefly to drain trailing bytes
            deadline = time.perf_counter() + 0.02
        else:
            time.sleep(0.001)
        if len(buf) >= want_bytes:
            break
    t1 = time.perf_counter()
    return bytes(buf), (t1 - t0) * 1000.0


def force_relays_off(ser: serial.Serial, frames: dict[str, bytes]) -> None:
    """Best-effort: turn all relays off. Used as cleanup on every exit path.

    Errors are logged but not raised — we are already exiting and a stuck
    relay should not become a Python traceback.
    """
    try:
        rx, _ = transact(ser, frames["write_off"],
                         want_bytes=len(frames["write_echo"]),
                         timeout_s=0.3)
        if classify(rx, frames["write_echo"]) == "ok":
            print("cleanup: relays forced OFF")
        else:
            print(f"cleanup: relays-off echo not OK ({rx.hex(' ').upper()})")
    except Exception as e:
        print(f"cleanup: relays-off raised {type(e).__name__}: {e}")


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


# ---------------------------------------------------------------------------
# Main loop with strict abort and latency capture
# ---------------------------------------------------------------------------
@dataclass
class RunResult:
    iters_completed: int = 0
    write_ms: list[float] = field(default_factory=list)
    read_ms: list[float] = field(default_factory=list)
    failure: dict | None = None    # populated on abort


def _do_half(ser: serial.Serial, cfg: PortConfig, frames: dict[str, bytes],
             *, on: bool, result: RunResult, iter_idx: int) -> bool:
    """Run one half (ON or OFF). Returns True on success, False on abort."""
    write_key  = "write_on"  if on else "write_off"
    read_key   = "read_high" if on else "read_low"
    half_label = "ON" if on else "OFF"

    # Write
    rx_w, wms = transact(ser, frames[write_key],
                         want_bytes=len(frames["write_echo"]),
                         timeout_s=cfg.resp_timeout)
    cat = classify(rx_w, frames["write_echo"])
    if cat != "ok":
        result.failure = {"iter": iter_idx, "half": half_label,
                          "phase": "write", "cat": cat, "rx": rx_w,
                          "expected": frames["write_echo"]}
        return False
    result.write_ms.append(wms)

    time.sleep(cfg.settle_ms / 1000.0)

    # Read
    rx_r, rms = transact(ser, frames["read_req"],
                         want_bytes=len(frames[read_key]),
                         timeout_s=cfg.resp_timeout)
    cat = classify(rx_r, frames[read_key])
    if cat != "ok":
        result.failure = {"iter": iter_idx, "half": half_label,
                          "phase": "read", "cat": cat, "rx": rx_r,
                          "expected": frames[read_key]}
        return False
    result.read_ms.append(rms)
    return True


def _percentile(xs: list[float], p: float) -> float:
    """Linear-interpolation percentile over a sorted copy of xs."""
    if not xs:
        return 0.0
    s = sorted(xs)
    if len(s) == 1:
        return s[0]
    k = (len(s) - 1) * p / 100.0
    lo = int(k)
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def summarise(label: str, samples: list[float]) -> str:
    if not samples:
        return f"{label}: (no samples)"
    return (f"{label} ms  min={min(samples):.2f}  "
            f"mean={statistics.fmean(samples):.2f}  "
            f"max={max(samples):.2f}  "
            f"p95={_percentile(samples, 95):.2f}  "
            f"p99={_percentile(samples, 99):.2f}")


def run_loop(ser: serial.Serial, cfg: PortConfig,
             frames: dict[str, bytes]) -> RunResult:
    result = RunResult()
    t_start = time.perf_counter()
    for i in range(cfg.iters):
        if not _do_half(ser, cfg, frames, on=True,  result=result, iter_idx=i):
            return result
        if not _do_half(ser, cfg, frames, on=False, result=result, iter_idx=i):
            return result
        result.iters_completed = i + 1
        if (i + 1) % 10 == 0 or (i + 1) == cfg.iters:
            last_w = result.write_ms[-1] if result.write_ms else 0
            last_r = result.read_ms[-1]  if result.read_ms  else 0
            print(f"  iter {i+1:>5}/{cfg.iters} | ok | "
                  f"last write={last_w:5.2f}ms read={last_r:5.2f}ms",
                  end="\r")
    elapsed = time.perf_counter() - t_start
    print()  # newline after the \r-overwritten progress line
    print(f"  total runtime: {elapsed:.1f}s")
    return result


def main(argv: list[str]) -> int:
    if len(argv) > 1 and argv[1] == "--self-test":
        _self_test()
        return 0
    cfg = parse_args(argv)
    print(f"Loopback robustness: port={cfg.port} slave={cfg.slave} iters={cfg.iters}")
    ser = serial.Serial(cfg.port, cfg.baud, bytesize=8, parity="N",
                        stopbits=1, timeout=0)
    frames = build_frames(cfg.slave)
    try:
        time.sleep(0.2)  # let the FTDI driver settle
        try:
            result = run_loop(ser, cfg, frames)
        except KeyboardInterrupt:
            print("\ninterrupted by user — running cleanup")
            force_relays_off(ser, frames)
            return 130   # conventional exit code for SIGINT
        if result.failure:
            f = result.failure
            print(f"=== loopback FAIL @ iter {f['iter']}, {f['half']}-half, "
                  f"{f['phase']}: cat={f['cat']} ===")
            print(f"  expected: {f['expected'].hex(' ').upper()}")
            print(f"  actual  : {f['rx'].hex(' ').upper()}")
            if (f['phase'] == "read"
                    and len(f['rx']) >= len(f['expected'])
                    and f['cat'] in ("wrong_data", "wrong_crc")):
                bad = per_bit_diff(f['rx'][3:5], f['expected'][3:5])
                if bad:
                    names = ", ".join(f"FB{i+1}" for i in bad)
                    print(f"  mismatched bits: {names}")
            force_relays_off(ser, frames)
            return 1
        force_relays_off(ser, frames)
        print(f"\n=== loopback PASS ({result.iters_completed} iters, "
              f"{2*result.iters_completed} actuations) ===")
        print("  " + summarise("write latency", result.write_ms))
        print("  " + summarise("read  latency", result.read_ms))
        return 0
    finally:
        ser.close()


if __name__ == "__main__":
    sys.exit(main(sys.argv))

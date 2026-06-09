# Modbus Loopback Robustness Test — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a Python test script that exercises the Modbus FC0F write path and FC02 read path in a closed loop (relays driving feedback inputs), running 1000 ON+OFF cycles with strict pass/fail and full latency stats.

**Architecture:** Single self-contained Python script in `tests/run_loopback_robustness.py`, ~180 LOC. Reuses the CRC16-Modbus pattern from `tests/run_bench_smoke.py` (kept duplicated rather than refactored — YAGNI; we have 2 robustness-style scripts, not enough to justify a shared module yet). No new dependencies; uses `pyserial` (already installed) and `time.perf_counter` for sub-ms latency capture. Safety cleanup via `try/finally` forces all relays OFF on every exit path.

**Tech Stack:** Python 3.x (the `py -3` launcher resolves to 3.14 on this bench), `pyserial 3.5`, standard library only otherwise. Target hardware: STM32C092 DIO Card on COM5 at 9600 8N1, slave ID 8.

---

## File Structure

| File                                   | Status | Responsibility                                                  |
|----------------------------------------|--------|-----------------------------------------------------------------|
| `tests/run_loopback_robustness.py`     | new    | The test script itself — CLI, frame builders, loop, reporting   |
| `README.md`                            | modify | One-line snippet in test/usage section pointing at new script   |

No firmware changes. No shared module refactor.

---

## Reference: known-good CRCs (from `run_bench_smoke.py` validation 2026-06-08)

These are used as test vectors in Task 1 to prove the CRC helper works before
we touch the serial port.

| Payload (hex, no CRC)           | Slave | Description                  | CRC (LE)  |
|---------------------------------|-------|------------------------------|-----------|
| `08 02 00 00 00 0C`             | 8     | FC02 read 12 inputs request  | `78 96`   |
| `08 02 02 00 00`                | 8     | FC02 response, all 0 data    | `65 B9`   |
| `08 0F 00 00 00 07`             | 8     | FC0F write echo (any value)  | `14 90`   |

These match what we observed on the wire during yesterday's 13/13 PASS run.

---

### Task 1: Skeleton + CRC helper + frame builders + self-test

**Files:**
- Create: `tests/run_loopback_robustness.py`

- [ ] **Step 1: Create the file with header docstring, imports, constants, helpers, and a `__main__`-gated self-test**

```python
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


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--self-test":
        _self_test()
        sys.exit(0)
    print("loopback robustness script — main loop not yet implemented")
```

- [ ] **Step 2: Run the self-test**

Run: `py -3 tests/run_loopback_robustness.py --self-test`
Expected:
```
  ok  FC02 read 12 inputs (slave 8)            crc=78 96
  ok  FC02 response all-zero (slave 8)         crc=65 B9
  ok  FC0F write echo (slave 8)                crc=14 90
CRC self-test passed.
```

- [ ] **Step 3: Commit**

```bash
git add tests/run_loopback_robustness.py
git commit -m "test(loopback): skeleton + CRC helper with self-test vectors"
```

---

### Task 2: CLI parsing + serial port open/close + dry-run main

**Files:**
- Modify: `tests/run_loopback_robustness.py`

- [ ] **Step 1: Add CLI parser, PortConfig dataclass, and replace the placeholder `__main__`**

Replace the `if __name__ == "__main__":` block at the bottom of the file with:

```python
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
```

- [ ] **Step 2: Verify CLI works with no args (uses defaults) — physical port must be present**

Run: `py -3 tests/run_loopback_robustness.py`
Expected (with FTDI adapter on COM5):
```
Loopback robustness: port=COM5 slave=8 iters=1000
port opened; main loop not yet implemented
```

If COM5 is unavailable expect `serial.serialutil.SerialException: could not open port 'COM5'` — that's correct error propagation.

- [ ] **Step 3: Verify --self-test still works**

Run: `py -3 tests/run_loopback_robustness.py --self-test`
Expected: same CRC self-test output as Task 1, exit 0.

- [ ] **Step 4: Commit**

```bash
git add tests/run_loopback_robustness.py
git commit -m "test(loopback): CLI parsing + serial port open/close"
```

---

### Task 3: Frame builders + classification helpers

**Files:**
- Modify: `tests/run_loopback_robustness.py`

- [ ] **Step 1: Add frame template builders and classifier between the self-test and `PortConfig`**

Insert the following block before the `@dataclass` for `PortConfig`:

```python
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
```

- [ ] **Step 2: Extend `_self_test` to also exercise `build_frames` and `classify`**

Replace the existing `_self_test` body with:

```python
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
    assert classify(bytes.fromhex("08 02 02 00 01 A4 78"),
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
```

- [ ] **Step 3: Run extended self-test**

Run: `py -3 tests/run_loopback_robustness.py --self-test`
Expected: 4 `ok` lines + `Self-test passed.`

- [ ] **Step 4: Commit**

```bash
git add tests/run_loopback_robustness.py
git commit -m "test(loopback): frame builders + response classifier with self-test"
```

---

### Task 4: Single-iteration transaction primitive (no loop yet)

**Files:**
- Modify: `tests/run_loopback_robustness.py`

- [ ] **Step 1: Add a serial I/O helper that sends a request, reads N bytes with timeout, and returns the bytes + latency**

Insert below the classification helpers:

```python
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
```

- [ ] **Step 2: Replace the placeholder `main` body with a single ON+OFF iteration to validate end-to-end**

Replace the inside of `try:` in `main()` (the `time.sleep(0.2)` + print) with:

```python
        time.sleep(0.2)  # let the FTDI driver settle
        frames = build_frames(cfg.slave)

        # ---- ON half ----
        rx, wms = transact(ser, frames["write_on"],
                           want_bytes=len(frames["write_echo"]),
                           timeout_s=cfg.resp_timeout)
        print(f"write_on   wms={wms:6.2f}  rx={rx.hex(' ').upper()}")
        assert classify(rx, frames["write_echo"]) == "ok", "write_on echo bad"

        time.sleep(cfg.settle_ms / 1000.0)

        rx, rms = transact(ser, frames["read_req"],
                           want_bytes=len(frames["read_high"]),
                           timeout_s=cfg.resp_timeout)
        print(f"read_high  rms={rms:6.2f}  rx={rx.hex(' ').upper()}")
        assert classify(rx, frames["read_high"]) == "ok", "read_high mismatch"

        # ---- OFF half ----
        rx, wms = transact(ser, frames["write_off"],
                           want_bytes=len(frames["write_echo"]),
                           timeout_s=cfg.resp_timeout)
        print(f"write_off  wms={wms:6.2f}  rx={rx.hex(' ').upper()}")
        assert classify(rx, frames["write_echo"]) == "ok", "write_off echo bad"

        time.sleep(cfg.settle_ms / 1000.0)

        rx, rms = transact(ser, frames["read_req"],
                           want_bytes=len(frames["read_low"]),
                           timeout_s=cfg.resp_timeout)
        print(f"read_low   rms={rms:6.2f}  rx={rx.hex(' ').upper()}")
        assert classify(rx, frames["read_low"]) == "ok", "read_low mismatch"

        print("\nsingle iteration OK")
        return 0
```

- [ ] **Step 3: Run a single iteration against the board**

Run: `py -3 tests/run_loopback_robustness.py COM5 8 1`
Expected (relays should click twice — ON then OFF):
```
Loopback robustness: port=COM5 slave=8 iters=1
write_on   wms= 14.xx  rx=08 0F 00 00 00 07 14 90
read_high  rms= 18.xx  rx=08 02 02 FF 0F 25 A8
write_off  wms= 14.xx  rx=08 0F 00 00 00 07 14 90
read_low   rms= 18.xx  rx=08 02 02 00 00 65 B9
single iteration OK
```

(Latency values are illustrative; the bytes are the important match.)

- [ ] **Step 4: Re-run self-test to confirm nothing broke**

Run: `py -3 tests/run_loopback_robustness.py --self-test`
Expected: self-test passes as before.

- [ ] **Step 5: Commit**

```bash
git add tests/run_loopback_robustness.py
git commit -m "test(loopback): single-iteration transaction primitive (manual proof)"
```

---

### Task 5: Loop, strict abort, latency capture

**Files:**
- Modify: `tests/run_loopback_robustness.py`

- [ ] **Step 1: Add an iteration data type and a `run_loop` function above `main`**

Insert before `main`:

```python
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


def run_loop(ser: serial.Serial, cfg: PortConfig,
             frames: dict[str, bytes]) -> RunResult:
    result = RunResult()
    for i in range(cfg.iters):
        if not _do_half(ser, cfg, frames, on=True,  result=result, iter_idx=i):
            return result
        if not _do_half(ser, cfg, frames, on=False, result=result, iter_idx=i):
            return result
        result.iters_completed = i + 1
    return result
```

- [ ] **Step 2: Replace the manual single-iteration body inside `main`'s `try:` with a call to `run_loop`**

Inside `main()`, replace the entire manual iteration block (`# ---- ON half ----` through `return 0` immediately before the `finally`) with:

```python
        time.sleep(0.2)
        frames = build_frames(cfg.slave)
        result = run_loop(ser, cfg, frames)
        if result.failure:
            print(f"FAIL at iter {result.failure['iter']} "
                  f"{result.failure['half']}-half {result.failure['phase']}: "
                  f"cat={result.failure['cat']}")
            print(f"  expected: {result.failure['expected'].hex(' ').upper()}")
            print(f"  actual  : {result.failure['rx'].hex(' ').upper()}")
            return 1
        print(f"PASS: {result.iters_completed} iters completed")
        return 0
```

- [ ] **Step 3: Run a 5-iter dry run against the board**

Run: `py -3 tests/run_loopback_robustness.py COM5 8 5`
Expected: relays click 10 times; final line `PASS: 5 iters completed`.

- [ ] **Step 4: Commit**

```bash
git add tests/run_loopback_robustness.py
git commit -m "test(loopback): main loop with strict abort + latency capture"
```

---

### Task 6: Safety cleanup (force OFF on every exit path)

**Files:**
- Modify: `tests/run_loopback_robustness.py`

- [ ] **Step 1: Add a `force_relays_off` function below `transact`**

Insert below `transact`:

```python
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
```

- [ ] **Step 2: Wrap `main`'s `try` to invoke cleanup on every exit path**

Replace the existing `try: ... finally: ser.close()` block in `main()` with:

```python
    frames = build_frames(cfg.slave)
    try:
        time.sleep(0.2)
        try:
            result = run_loop(ser, cfg, frames)
        except KeyboardInterrupt:
            print("\ninterrupted by user — running cleanup")
            force_relays_off(ser, frames)
            return 130   # conventional exit code for SIGINT
        if result.failure:
            print(f"FAIL at iter {result.failure['iter']} "
                  f"{result.failure['half']}-half {result.failure['phase']}: "
                  f"cat={result.failure['cat']}")
            print(f"  expected: {result.failure['expected'].hex(' ').upper()}")
            print(f"  actual  : {result.failure['rx'].hex(' ').upper()}")
            force_relays_off(ser, frames)
            return 1
        force_relays_off(ser, frames)
        print(f"PASS: {result.iters_completed} iters completed")
        return 0
    finally:
        ser.close()
```

- [ ] **Step 3: Verify cleanup runs on success (3-iter run)**

Run: `py -3 tests/run_loopback_robustness.py COM5 8 3`
Expected last two lines:
```
cleanup: relays forced OFF
PASS: 3 iters completed
```

- [ ] **Step 4: Verify cleanup runs on KeyboardInterrupt (5000-iter run, hit Ctrl-C mid-run)**

Run: `py -3 tests/run_loopback_robustness.py COM5 8 5000` and press Ctrl-C after a few seconds.
Expected:
```
interrupted by user — running cleanup
cleanup: relays forced OFF
```
And the relays should NOT be left clicking / energised.

- [ ] **Step 5: Commit**

```bash
git add tests/run_loopback_robustness.py
git commit -m "test(loopback): safety cleanup forces relays OFF on every exit path"
```

---

### Task 7: Live progress + final stats summary + per-bit failure diff

**Files:**
- Modify: `tests/run_loopback_robustness.py`

- [ ] **Step 1: Add a progress callback to `_do_half`/`run_loop` and a stats summary helper**

Replace the existing `run_loop` function with:

```python
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
```

- [ ] **Step 2: Update the FAIL branch in `main` to include per-bit diff for read failures**

Inside `main()`, replace the existing `if result.failure:` block with:

```python
        if result.failure:
            f = result.failure
            print(f"=== loopback FAIL @ iter {f['iter']}, {f['half']}-half, "
                  f"{f['phase']}: cat={f['cat']} ===")
            print(f"  expected: {f['expected'].hex(' ').upper()}")
            print(f"  actual  : {f['rx'].hex(' ').upper()}")
            # If we have enough bytes for a data slice on a read failure,
            # show which input bits disagreed.
            if (f['phase'] == "read"
                    and len(f['rx']) >= len(f['expected'])
                    and f['cat'] in ("wrong_data", "wrong_crc")):
                bad = per_bit_diff(f['rx'][3:5], f['expected'][3:5])
                if bad:
                    names = ", ".join(f"FB{i+1}" for i in bad)
                    print(f"  mismatched bits: {names}")
            force_relays_off(ser, frames)
            return 1
```

- [ ] **Step 3: Update the PASS branch to print stats**

Replace the PASS branch in `main()` with:

```python
        force_relays_off(ser, frames)
        print(f"\n=== loopback PASS ({result.iters_completed} iters, "
              f"{2*result.iters_completed} actuations) ===")
        print("  " + summarise("write latency", result.write_ms))
        print("  " + summarise("read  latency", result.read_ms))
        return 0
```

- [ ] **Step 4: Run 25 iterations to verify progress + summary look right**

Run: `py -3 tests/run_loopback_robustness.py COM5 8 25`
Expected:
- A progress line overwriting itself every 10 iters
- `total runtime: ~7.0s` (around 280ms × 25 = 7s)
- `cleanup: relays forced OFF`
- `=== loopback PASS (25 iters, 50 actuations) ===`
- Two latency stat lines

- [ ] **Step 5: Commit**

```bash
git add tests/run_loopback_robustness.py
git commit -m "test(loopback): live progress, latency stats, per-bit failure diff"
```

---

### Task 8: Full 1000-iteration bench run + README cross-link

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Run the full 1000-iteration test against the live board**

Run: `py -3 tests/run_loopback_robustness.py COM5 8 1000`
Expected: ~4-5 min runtime; relays click 2000 times; final output similar to:
```
  iter  1000/1000 | ok | last write=14.20ms read=18.40ms
  total runtime: 281.4s
cleanup: relays forced OFF

=== loopback PASS (1000 iters, 2000 actuations) ===
  write latency ms  min=13.90  mean=14.40  max=17.10  p95=15.20  p99=16.00
  read  latency ms  min=17.20  mean=18.50  max=22.40  p95=19.30  p99=20.70
```

Record the actual stats (you'll reference them in the README snippet below).

- [ ] **Step 2: Add a "Robustness" subsection to `README.md`**

Locate the "Bench validation" / "Smoke tests" section in `README.md` (use Grep to find it). Append a new subsection at the end of that section:

```markdown
### Loopback robustness (1000-iter)

With every discrete input wired to a relay output, exercise the full
write→GPIO→read→GPIO loop 1000 times (= 2000 relay actuations).

```bash
py -3 tests/run_loopback_robustness.py COM5 8 1000
```

The script aborts on the first mismatch and force-OFFs the relays on every
exit path (including Ctrl-C). See
`docs/superpowers/specs/2026-06-09-modbus-loopback-robustness-design.md`
for the test design.
```

- [ ] **Step 3: Commit**

```bash
git add README.md
git commit -m "docs(readme): cross-link loopback robustness test"
```

- [ ] **Step 4: Push (only if user explicitly asks)**

DO NOT push without user confirmation. If they ask:
```bash
git push origin main
```

---

## Self-Review Notes

**Spec coverage check (cross-referenced to `docs/superpowers/specs/2026-06-09-modbus-loopback-robustness-design.md`):**

| Spec section | Plan task         | Notes                                                              |
|--------------|-------------------|--------------------------------------------------------------------|
| §3.1 file    | Task 1            | `tests/run_loopback_robustness.py` created                         |
| §3.2 CLI     | Task 2            | port/slave/iters with defaults COM5/8/1000                         |
| §3.3 loop    | Task 5            | ON+OFF per iter, 100 ms settle, FF 0F / 00 00 expectations         |
| §3.4 strict  | Task 5            | abort-on-first-mismatch via `RunResult.failure`                     |
| §3.5 safety  | Task 6            | `force_relays_off` in normal/fail/KeyboardInterrupt paths           |
| §3.6 stats   | Task 7            | min/mean/max/p95/p99 for both write and read                       |
| §3.7 output  | Task 7            | live every 10 iters + final summary; per-bit diff on read FAIL     |
| §5 wear note | Task 8            | README snippet mentions 2000 actuations                            |
| §7 success   | Tasks 4, 5, 6, 8  | manual single-iter validation, 5-iter run, Ctrl-C run, 1000 run    |

All spec sections covered.

**Type/name consistency check:** `crc16_modbus`, `fr`, `build_frames`, `classify`, `per_bit_diff`, `transact`, `force_relays_off`, `run_loop`, `_do_half`, `_percentile`, `summarise`, `RunResult`, `PortConfig` — names used consistently across all tasks.

**Placeholder scan:** No TBD / TODO / "implement later" entries. All code blocks are complete.

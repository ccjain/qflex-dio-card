# DIO Card relay CLI — design

**Status:** approved 2026-06-09
**Author:** brainstorming with kmittal@quenchchargers.com
**File to create:** `tests/relay.py`

## Problem

Turning a single relay on or off from the bench currently requires hand-building a Modbus FC05 frame (slave + FC + address + value + CRC) and sending it with mbpoll or a one-off Python snippet. We have a smoke test that exercises every code path but nothing scriptable for "give me one relay, on, now."

## Goal

A one-shot CLI tool that flips exactly one relay on the DIO Card. Each invocation: parse args → open serial → send one FC05 frame → verify echo → exit.

## Non-goals

- Reading coil state (FC01) before writing.
- A `toggle` action.
- Batch writes via FC0F (multi-coil).
- Reading discrete-input feedbacks (FC02).
- An interactive REPL.
- Slave-ID auto-scan.

These are all easy follow-ups but out of scope for this script.

## Interface

```
py tests/relay.py <relay> <on|off> [--port COM5] [--slave 1]
```

| Arg / flag | Type | Default | Meaning |
|---|---|---|---|
| `<relay>` | int 1–7 | (required) | Relay number as printed on the board. Maps to Modbus coil address `relay - 1`. |
| `<on \| off>` | literal | (required) | Action. Case-insensitive. |
| `--port` | string | `COM5` | Serial port. Matches `run_bench_smoke.py` default. |
| `--slave` | int 1–247 | `1` | Modbus slave ID. Device currently reports 1; pass `--slave 8` (or whatever DIP is set to) if changed. |

### Exit codes

| Code | Meaning |
|---|---|
| 0 | Write acknowledged. Echo matches request. |
| 1 | Operational failure: cannot open port, no reply, wrong echo, CRC mismatch. Details printed to stderr. |
| 2 | Usage error: bad arg shape, relay out of range, action not `on`/`off`. Usage printed to stderr. |

### Output format

Success (stdout, exactly one line):
```
relay <N> -> ON
```
or `OFF`.

Failure on the wire (stderr, two lines):
```
no reply from slave <id> on <port>
```
or
```
echo mismatch
  TX: <hex bytes>
  RX: <hex bytes>
```

No banner, no progress text. The script is meant to be chainable in shell.

## Modbus frame

Function code **05 — Write Single Coil**.

| Byte | Value | Notes |
|---|---|---|
| 0 | slave id | from `--slave` |
| 1 | `0x05` | FC05 |
| 2 | `0x00` | coil address high |
| 3 | relay - 1 | coil address low |
| 4 | `0xFF` (on) or `0x00` (off) | value high |
| 5 | `0x00` | value low (FC05 spec: only 0xFF00 or 0x0000 are legal) |
| 6 | CRC lo | CRC-16/Modbus, poly 0xA001, seed 0xFFFF, little-endian |
| 7 | CRC hi | |

The device's FC05 reply is byte-identical to the request — the script confirms success by exact-match comparison of TX against RX.

## Implementation notes

- **Serial config:** `9600 8N1`, read `timeout=0` (non-blocking), drain loop matching `run_bench_smoke.py`. Total wait budget: 250 ms.
- **CRC helper:** copy the 7-line `crc16_modbus()` from `run_bench_smoke.py`. Standalone — the tool must not depend on a test file (wrong dependency direction). If we ever need shared helpers across multiple tools, extract `tests/_modbus.py` then.
- **Each call opens and closes the port.** Adds ~200 ms latency per call, which is fine for a one-shot tool and isolates failures.
- **No state preserved between invocations.** No config files, no env vars (other than what `serial.Serial` honors implicitly).
- **Target size:** ~60 lines, single file, single helper function plus `main`.

## Testing

The smoke test (`tests/run_bench_smoke.py`) already covers FC05 protocol behavior. For this tool, manual bench verification is sufficient:

1. `py tests/relay.py 1 on` → relay 1 clicks on. Exit 0.
2. `py tests/relay.py 1 off` → relay 1 clicks off. Exit 0.
3. `py tests/relay.py 0 on` → usage error to stderr. Exit 2.
4. `py tests/relay.py 1 maybe` → usage error to stderr. Exit 2.
5. `py tests/relay.py 1 on --port COM99` → cannot open. Exit 1.
6. `py tests/relay.py 1 on --slave 99` → no reply from slave 99. Exit 1.

No automated test file. The smoke test is the protocol-coverage authority; this script is a thin user-facing wrapper.

## Why these choices

- **One-shot CLI over REPL** — matches the user's literal ask, scriptable, smallest surface area.
- **On/off only, no toggle** — toggle needs an FC01 read first; the user explicitly chose minimal.
- **Default slave = 1** — matches the device's current reported ID, so `py tests/relay.py 1 on` works without any flag. Users can override per call.
- **Standalone (no import from smoke test)** — a tool depending on a test is inverted. Seven lines of CRC duplication is cheaper than the coupling.

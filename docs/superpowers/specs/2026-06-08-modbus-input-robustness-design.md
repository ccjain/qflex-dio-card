# Modbus RTU Input-Read Robustness Test — Design

**Date:** 2026-06-08
**Author:** brainstorm session (kmittal@quenchchargers.com + Claude)
**Status:** Approved, ready for implementation plan
**Related artifacts:**
- `tests/run_bench_smoke.py` — single-shot bench suite (already passing 13/13)
- `tests/mbpoll_smoke.md` — bench validation spec (Phase F)
- `Core/Src/app/feedback.c` — slave-side input polarity / debounce
- `Core/Src/app/modbus_app.c` — FC02 dispatch glue

---

## 1. Purpose

Stress-test the slave's FC02 (`Read Discrete Inputs`) round-trip with **2000
back-to-back requests** to:

1. Catch race conditions in the IDLE-frame detector under continuous traffic.
2. Detect occasional CRC failures, dropped responses, or framing glitches that a
   single-shot smoke test cannot surface.
3. Establish a **response-time baseline** (min / mean / max / p95 / p99) we can
   compare against in future regressions or after firmware changes.
4. Confirm the **default input state** (all 12 bits = 0 when nothing is wired
   into FB1..FB12) is observed consistently across the run.

This complements the existing single-shot smoke suite — that proves the protocol
works; this proves it keeps working under load.

## 2. Background & invariant we are testing

The DIO Card's 12 discrete inputs (FB1..FB12) are GPIO inputs with internal
pull-up enabled. The firmware (`Core/Src/app/feedback.c:57`) inverts the
electrical level so that the Modbus bit reads:

| Pin state                          | Modbus bit |
|------------------------------------|------------|
| Floating (pull-up wins, pin HIGH)  | `0`        |
| Shorted to GND (contactor closed)  | `1`        |

With nothing wired into FB1..FB12, every read must return:

```
RX (slave 8): 08 02 02 00 00 65 B9
                       ^^^^^ data bytes — both zero
```

If any of those data bytes ever flips to non-zero during a 2000-iter back-to-back
run, that is either (a) electrical noise the debouncer should have absorbed, or
(b) a firmware bug. Either way it should fail the test.

## 3. Test architecture

### 3.1 File layout

Single Python script:

```
tests/run_robustness_inputs.py        # ~150 LOC, sibling to run_bench_smoke.py
```

Reuses the existing CRC16-Modbus helper pattern from `run_bench_smoke.py`. No new
runtime dependencies — `pyserial` is already installed.

### 3.2 CLI

```
py -3 tests/run_robustness_inputs.py [COM_PORT] [SLAVE_ID] [ITERATIONS]
```

| Position | Default | Notes                                                |
|----------|---------|------------------------------------------------------|
| 1        | `COM5`  | FTDI USB-RS485 adapter on this bench                 |
| 2        | `8`     | DIP-configured slave ID currently on the board       |
| 3        | `2000`  | Spec-required iteration count                        |

Defaults match the just-validated bench setup. Overrides allow re-running with a
different DIP setting or on a different adapter without code changes.

### 3.3 Per-iteration flow

```
For i in 0..N-1:
    t0   = perf_counter()
    flush RX buffer
    write REQUEST   (8 bytes)
    rx   = read until 7 bytes or 200 ms timeout
    t1   = perf_counter()
    latency_ms[i] = (t1 - t0) * 1000
    classify(rx) -> ok | no_reply | short | wrong_crc | wrong_slave_or_fc | wrong_data
```

`REQUEST` is computed once before the loop:
```
SID 02 00 00 00 0C  <crc16>      # FC02, read 12 inputs starting at addr 0
```

`EXPECTED` is computed once before the loop:
```
SID 02 02 00 00  <crc16>         # 2 data bytes, both zero
```

The script does an **exact byte compare** against `EXPECTED`. The classification
buckets exist purely for diagnostic reporting on a failure — the pass criterion
is `failures == 0`.

### 3.4 Failure categories

| Category             | Condition                                                            |
|----------------------|----------------------------------------------------------------------|
| `no_reply`           | `len(rx) == 0` after timeout                                         |
| `short`              | `0 < len(rx) < 7`                                                    |
| `wrong_crc`          | 7 bytes, but `crc16(rx[:5]) != rx[5:7]`                              |
| `wrong_slave_or_fc`  | header bytes (slave / FC) don't match expected                       |
| `wrong_data`         | header + CRC OK, but `rx[3:5] != b"\x00\x00"`                        |

`wrong_data` is the interesting one — it would mean an input briefly read as `1`
on a floating line, which is the signal we'd care about most.

### 3.5 Statistics

Over **all successful samples**:

```
min, mean, max, p95, p99, stdev   # all in milliseconds
```

p95/p99 computed by sorting the latency list (2000 samples is small enough that
sort-based percentiles are fine — no need for streaming algorithms).

### 3.6 Output

Live progress every ~100 iterations (overwriting one line so the terminal
doesn't scroll):

```
Iter 1700/2000 | ok=1700 fail=0 | last=18.4ms
```

Final summary (always printed, success or failure):

```
=== robustness PASS ===
latency ms  min=17.21  mean=18.43  max=24.12  p95=19.10  p99=20.55  stdev=0.41
failures by category: {}
```

On failure:

```
=== robustness FAIL ===
latency ms  min=17.21  mean=18.43  max=24.12  p95=19.10  p99=20.55  stdev=0.41
failures by category: {'wrong_data': 3, 'no_reply': 1}
first 3 failures:
  iter   742  cat=wrong_data  rx=08 02 02 00 01 A4 78
  iter  1031  cat=wrong_data  rx=08 02 02 00 02 ...
  iter  1450  cat=no_reply    rx=
```

Exit code: `0` on PASS, `1` on FAIL — enables CI/scripted integration later.

## 4. Design rationale (decisions made during brainstorming)

| Decision                  | Choice               | Why                                                              |
|---------------------------|----------------------|------------------------------------------------------------------|
| Expected default state    | all `0` bits         | Matches firmware polarity (`feedback.c:57`) + just-passed smoke   |
| Cadence                   | back-to-back, no gap | Max stress on IDLE-framer; ~30 s wall-clock at 9600               |
| Pass criterion            | strict (0 failures)  | Robustness test — any flake is a regression                      |
| Latency stats             | min/mean/max/p95/p99 | Standard distribution-shape capture; enough to spot tail issues   |
| Output style              | live + summary       | Long-running test needs progress; summary is the machine record   |

## 5. Out of scope (deliberately)

- **Wired-feedback verification** (smoke §2.2): requires a physical jumper to
  GND. The robustness run only checks the floating baseline.
- **Coil-write stress (FC05 / FC0F):** running 2000 relay actuations would put
  unnecessary wear on the physical relays. A separate, lower-iteration
  write-stress test could be added later if needed.
- **Multi-slave / multi-port:** single-target by design. The script accepts
  port and slave args so multiple invocations cover this trivially.
- **Long-duration soak:** smoke §4 already covers a 60 s 1 Hz soak; this is
  back-to-back stress, a different stress dimension.

## 6. Risks & open considerations

1. **CRC false-negatives from the FTDI adapter buffer:** under continuous
   traffic, the adapter could in theory return concatenated frames if our read
   loop is sloppy. Mitigation: we read **exactly 7 bytes** with timeout — any
   surplus bytes get caught on the next `flush()` and surface as a
   `wrong_slave_or_fc` or `short` on the *following* iteration.
2. **Adapter-driver back-pressure on Windows:** an FTDI VCP driver hiccup could
   manifest as a `no_reply` even if the slave actually responded. We don't
   distinguish driver-side vs slave-side failure in the summary; the failure
   bucket alone won't tell you which. Acceptable for a first version — if the
   issue arises we can add a USB-level loopback comparison.
3. **System-clock granularity:** `time.perf_counter()` on Windows is
   sub-microsecond, well below the ~15 ms expected latency. No mitigation
   needed.

## 7. Success criteria

The test is considered well-designed if:

- Running it immediately after the bench smoke suite produces `PASS` with
  `failures by category: {}` and a max latency under ~30 ms.
- Running it with the board powered off produces `FAIL` with all 2000 iterations
  in `no_reply` (i.e. the script gracefully reports the failure mode rather
  than crashing).
- Running it with `SLAVE_ID` set to a value other than the DIP-configured one
  also produces `FAIL` with all `no_reply` (silent-drop behavior verified by
  smoke §3.1 still applies).

## 8. Implementation hand-off

After this spec is approved and committed, the next step is to invoke
`superpowers:writing-plans` to produce a per-step implementation plan with
test-first ordering. The plan should cover at minimum:

1. Create file skeleton + CRC helper + CLI args.
2. Add request / expected frame builders.
3. Implement the main loop with classification.
4. Add stats + reporting.
5. Local run against COM5 / slave 8 — confirm PASS.
6. Commit + cross-link from `README.md` (test invocation snippet).

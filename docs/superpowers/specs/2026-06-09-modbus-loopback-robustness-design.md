# Modbus Loopback Robustness Test — Design

**Date:** 2026-06-09
**Author:** brainstorm session (kmittal@quenchchargers.com + Claude)
**Status:** Approved, ready for implementation plan
**Related artifacts:**
- `tests/run_bench_smoke.py` — single-shot bench suite (13/13 passing as of 2026-06-08)
- `tests/mbpoll_smoke.md` — bench validation spec (Phase F)
- `docs/superpowers/specs/2026-06-08-modbus-input-robustness-design.md` — input-only robustness sibling
- `Core/Inc/pin_map.h` — relay / feedback pin tables (coils 0..6 → R1..R7, inputs 0..11 → FB1..FB12)
- `Core/Src/app/feedback.c` — input polarity / debounce
- `Core/Src/app/relay.c` — relay write path

---

## 1. Purpose

The user has wired each of the 12 discrete inputs (FB1..FB12) to one of the 7
relay outputs (R1..R7) so that the relays directly drive the inputs in a
loopback. With this physical configuration:

- Writing **all relays ON** must produce a read of **all 12 inputs HIGH** (12
  LSBs set).
- Writing **all relays OFF** must produce a read of **all 12 inputs LOW**.

The test repeats this ON/OFF cycle **1000 times** to catch:

1. Stuck-on or stuck-off relays (mechanical failure)
2. Broken or intermittent loopback wires
3. Race conditions in the FC0F write path under repeated load
4. Edge cases in the firmware feedback debouncer that only appear under churn
5. Drift in the 100 ms settle / scan timing
6. Any latency outliers across writes or reads

The complementary input-only robustness test (sibling spec) covers the read
path alone with no relay actuation. This test exercises the entire
write→GPIO→read→GPIO path end-to-end.

## 2. Wiring (user-supplied loopback)

| Relay | Modbus coil | Connects to (DIO) | Modbus input |
|-------|-------------|--------------------|--------------|
| K15   | _user label_ | J1, J4            | _2 inputs_   |
| K1    | _user label_ | J9, J12           | _2 inputs_   |
| K2    | _user label_ | J16, J19          | _2 inputs_   |
| K3    | _user label_ | J2, J6            | _2 inputs_   |
| K4    | _user label_ | J10, J13          | _2 inputs_   |
| K5    | _user label_ | J17               | _1 input_    |
| K6    | _user label_ | J20               | _1 input_    |

Total: 7 relays drive 12 inputs (5 relays each drive 2 inputs; 2 relays each
drive 1 input). The test doesn't depend on **which** relay drives which input —
because we always energise all relays or none, individual mapping is irrelevant.
The wiring is informational; what matters is that every one of the 12 inputs is
electrically driven by some relay.

## 3. Test architecture

### 3.1 File layout

Single Python script:

```
tests/run_loopback_robustness.py     # ~180 LOC, sibling to run_bench_smoke.py
```

Reuses the same CRC16-Modbus / frame-builder pattern. No new dependencies —
`pyserial` already installed.

### 3.2 CLI

```
py -3 tests/run_loopback_robustness.py [COM_PORT] [SLAVE_ID] [ITERATIONS]
```

| Position | Default | Notes                                                |
|----------|---------|------------------------------------------------------|
| 1        | `COM5`  | FTDI USB-RS485 adapter on this bench                 |
| 2        | `8`     | DIP-configured slave ID currently on the board       |
| 3        | `1000`  | Spec-required iteration count                        |

### 3.3 Per-iteration flow

One iteration = one full ON+OFF cycle (two writes, two reads).

```
loop i in 0..N-1:
    # --- ON half ---
    t_w0 = perf_counter()
    write FC0F (coils 0..6 = 1)        # 01 0F 0000 0007 01 7F <crc>
    t_w1 = perf_counter()              # write latency
    sleep 100 ms                       # relay settle + debouncer
    t_r0 = perf_counter()
    read  FC02 (inputs 0..11)          # 01 02 0000 000C <crc>
    t_r1 = perf_counter()              # read latency
    assert data bytes == FF 0F         # 12 LSBs set, upper 4 bits padded 0

    # --- OFF half ---
    write FC0F (coils 0..6 = 0)
    sleep 100 ms
    read  FC02 (inputs 0..11)
    assert data bytes == 00 00
```

Frame layouts:

| Direction | Frame                                | Notes                          |
|-----------|--------------------------------------|--------------------------------|
| write ON  | `SID 0F 0000 0007 01 7F <crc>`       | 7 coils, value byte `0x7F`     |
| write OFF | `SID 0F 0000 0007 01 00 <crc>`       | 7 coils, value byte `0x00`     |
| write rsp | `SID 0F 0000 0007 <crc>`             | echo of address + count        |
| read req  | `SID 02 0000 000C <crc>`             | 12 inputs starting at addr 0   |
| read rsp ON  | `SID 02 02 FF 0F <crc>`           | bits 0..11 high; pad bits = 0  |
| read rsp OFF | `SID 02 02 00 00 <crc>`           | all bits low                   |

`SID` is the slave ID (default 8). All CRCs are CRC16-Modbus (poly 0xA001,
seed 0xFFFF, little-endian).

### 3.4 Pass / fail criterion

**Strict abort.** Any of the following terminates the run with FAIL:

- Write response not equal to expected echo
- Read response data bytes not exactly equal to expected (`FF 0F` or `00 00`)
- No reply (length 0) within 200 ms timeout
- CRC mismatch in any received frame

Successful iterations record write and read latency for later statistics.

### 3.5 Safety cleanup

On normal exit AND on failure abort AND on KeyboardInterrupt, the script
issues a final `write FC0F (all 0)` before closing the serial port to leave the
relays de-energised. If even that final write fails, log a warning but still
close the port and exit.

### 3.6 Statistics

Over all successful iterations, computed separately for write and read:

```
min, mean, max, p95, p99   # all in milliseconds
```

p95/p99 from sorted samples (1000–4000 samples is small enough for sort-based
percentiles).

### 3.7 Output

Live every 10 iterations:

```
Iter  430/1000 | ok | last write=14.2ms read=18.6ms
```

Final summary on PASS:

```
=== loopback PASS (1000 iters, 2000 actuations) ===
write latency ms  min=13.9  mean=14.4  max=17.1  p95=15.2  p99=16.0
read  latency ms  min=17.2  mean=18.5  max=22.4  p95=19.3  p99=20.7
total runtime: 4 min 47 s
```

Final summary on FAIL (example: input 0 stayed low when relays were on):

```
=== loopback FAIL @ iter 437, ON-half ===
expected read data: FF 0F
actual   read data: FE 0F
mismatched bits   : input 0 (FB1)
RX raw            : 08 02 02 FE 0F 65 22
```

Exit code: `0` on PASS, `1` on FAIL.

## 4. Design rationale (decisions made during brainstorming)

| Decision               | Choice                                  | Why                                                                              |
|------------------------|-----------------------------------------|----------------------------------------------------------------------------------|
| Iteration shape        | full ON+OFF cycle per iter              | Symmetric, complete loopback verification each pass                              |
| Iteration count        | 1000 (= 2000 actuations)                | User-requested; well below relay mechanical life                                 |
| Pattern                | all-on / all-off (FC0F single shot)     | Simplest stress on write path; avoids per-relay address gymnastics               |
| Settle delay           | 100 ms between write and read           | User-requested; >> debouncer (3 × 5 ms = 15 ms) and relay mechanical (~5 ms)    |
| Pass criterion         | strict abort on first mismatch          | Wastes no relay life chasing failures after the first; clear bisect target       |
| Latency capture        | write + read separately, full stats     | Cheap to measure; useful comparison vs sibling input-only spec                   |
| Safety cleanup         | force OFF on every exit path            | Avoids leaving relays energised after an interrupted run                         |

## 5. Out of scope

- **Per-relay loopback** (one relay at a time, verify only its driven inputs).
  Possible future test if the all-on/all-off test passes but the user suspects
  a single relay misbehaving.
- **Walking-1 / walking-0 patterns.** Same rationale — useful diagnostic, but
  not the question this test is answering.
- **Latency thresholds.** We measure and report; we don't fail on slow
  latency. Hard thresholds belong in a later perf-regression test.
- **Modifying firmware** (debounce N, scan period, etc.). Test exercises the
  shipped firmware as-is.
- **Multi-port / multi-slave runs.** Single target per invocation; script
  accepts CLI args, so loops in a shell can cover multiple slaves.

## 6. Risks & open considerations

1. **FB12 on PC13.** PC13 on STM32 parts can have wake-up / RTC tamper
   alternate functions. The pin map (`Core/Inc/pin_map.h:54`) configures it as
   a plain pull-up GPIO; if there is any residual reset-related behavior on
   this pin we'd see input 11 occasionally misread. Mitigation: failure
   reporting names the specific input index → easy to spot.
2. **Relay back-EMF / contact bounce.** Real relays bounce on close; the
   firmware's 3-sample debouncer at 5 ms scan = 15 ms minimum stable window.
   100 ms settle is comfortably above that, but if the user's relays have an
   unusually long bounce, we could see a transient `FE 0F` style miss. If
   that happens, increasing settle to 200 ms is the first knob; not the test's
   problem to fix automatically.
3. **Bus contention on early start.** The first iteration's write could
   collide with a prior partial frame in the FTDI driver buffer if a previous
   run was interrupted. The script flushes RX before each transaction, which
   handles this.
4. **Driver-side `no_reply` ambiguity.** As with the sibling spec, an FTDI VCP
   hiccup is indistinguishable from a slave-side dropout. Acceptable; if it
   becomes a recurring noise source, add a USB-loopback control run.

## 7. Success criteria

The test is well-designed if:

- Running it immediately after the bench smoke suite produces `PASS` with
  zero failures and read latency comparable to smoke (~18–25 ms).
- Cutting one loopback wire (any of the 12) produces `FAIL` at iter 0,
  ON-half, with the correct mismatched bit named.
- Pulling power to the board mid-run produces `FAIL` with `no_reply` cleanly,
  not a Python traceback; and the cleanup attempt is logged.
- A Ctrl-C interrupt mid-run leaves relays in OFF state.

## 8. Implementation hand-off

Next step: invoke `superpowers:writing-plans` to produce a per-step
implementation plan. The plan should cover at minimum:

1. Create file skeleton + CRC helper + CLI args (or factor a shared
   `tests/_mb_common.py` if it's about to be used by 3+ scripts).
2. Build the four frame templates and per-iteration loop.
3. Implement strict abort + per-bit mismatch report.
4. Implement safety cleanup (force OFF) on all exit paths.
5. Add latency capture + stats summary.
6. Local run against COM5 / slave 8 with the user's wiring — confirm PASS.
7. Commit + cross-link from `README.md` (test invocation snippet).

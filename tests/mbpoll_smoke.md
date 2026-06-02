# DIO Card — Modbus RTU bench smoke tests

This document captures the on-bench smoke tests that the firmware was
validated against during the v1 bring-up. Tests below are written for
`mbpoll` (https://github.com/epsilonrt/mbpoll) but the raw frames are
also recorded so you can replay them from PowerShell, QModMaster, or any
other Modbus master.

**Setup (assumed throughout this file):**
- DIO Card slave ID = **1** (DIP1 closed, DIP2/3/4 open)
- USB-RS485 adapter on `COM4` at 9600 8N1
- A and B wires matched (no swap)

If your COM port differs, substitute throughout. `mbpoll` examples use the
Linux convention `/dev/ttyUSB0`; Windows users substitute `COM4`.

---

## 1. Coil reads / writes (FC 01, 05, 0F)

### 1.1 Read all 7 coils
```
mbpoll -m rtu -a 1 -b 9600 -P none -t 0 -r 1 -c 7 COM4
# raw: 01 01 00 00 00 07 7D C8
```
On boot all relays are off → expect every bit = 0:
```
[1]: 0   [2]: 0   [3]: 0   [4]: 0   [5]: 0   [6]: 0   [7]: 0
```

### 1.2 Energise relay 1 (FC 05)
```
mbpoll -m rtu -a 1 -b 9600 -P none -t 0 -r 1 COM4 1
# raw: 01 05 00 00 FF 00 8C 3A
```
Audible click; coil 0 echoes back. Verify with §1.1 — `[1]: 1` now.

### 1.3 De-energise relay 1
```
mbpoll -m rtu -a 1 -b 9600 -P none -t 0 -r 1 COM4 0
# raw: 01 05 00 00 00 00 CD CA
```

### 1.4 Write multi (FC 0F): pattern 0x55 → R1,R3,R5,R7 on
```
mbpoll -m rtu -a 1 -b 9600 -P none -t 0 -r 1 COM4 1 0 1 0 1 0 1
# raw: 01 0F 00 00 00 07 01 55 0E A9
```
Four relays click. Verify with §1.1.

### 1.5 Illegal coil value (FC 05 with anything other than 0x0000 / 0xFF00)
```
# raw: 01 05 00 00 12 34 7C 9C   (deliberately bad value 0x1234)
```
Expect exception 0x03 (illegal data value):
```
RX: 01 85 03 02 91
```

### 1.6 Out-of-range write (coil 7 doesn't exist; only 0..6)
```
mbpoll -m rtu -a 1 -b 9600 -P none -t 0 -r 8 COM4 1
# raw: 01 05 00 07 FF 00 7C F9
```
Expect exception 0x02 (illegal data address):
```
RX: 01 85 02 C3 51
```

---

## 2. Discrete-input reads (FC 02)

### 2.1 Read all 12 feedbacks
```
mbpoll -m rtu -a 1 -b 9600 -P none -t 1 -r 1 -c 12 COM4
# raw: 01 02 00 00 00 0C 78 0F
```
With nothing wired to FB1..FB12 (inputs floating, pulled high internally),
every bit = 0:
```
RX: 01 02 02 00 00 B9 B8
```

### 2.2 Activate one feedback
Short FB1 (PD0) to GND. Re-read:
Expect `[1]: 1` and the rest zero — RX byte should be `01 02 02 01 00 ...`.

### 2.3 Out-of-range read
```
mbpoll -m rtu -a 1 -b 9600 -P none -t 1 -r 13 -c 1 COM4
# raw: 01 02 00 0C 00 01 38 0F
```
Expect exception 0x02.

---

## 3. Negative tests

### 3.1 Wrong slave ID — silent drop
```
mbpoll -m rtu -a 7 -b 9600 -P none -t 0 -r 1 -c 1 COM4
```
Master times out; board sends nothing.

### 3.2 Unsupported FC (FC 03 Read Holding Registers)
```
mbpoll -m rtu -a 1 -b 9600 -P none -t 4 -r 1 -c 1 COM4
# raw: 01 03 00 00 00 01 84 0A
```
Expect exception 0x01 (illegal function):
```
RX: 01 83 01 80 F0
```

### 3.3 DIP = 0 (all open) — fast-blink LED, no traffic
Open all 4 DIP switches and power-cycle. Heartbeat LED on PF3 should
toggle every 100 ms (5 Hz blink). Any Modbus request times out.

---

## 4. Stability / long-running

Run `mbpoll` in repeat mode for 60 seconds and confirm no missed responses:
```
mbpoll -m rtu -a 1 -b 9600 -P none -t 1 -r 1 -c 12 -1 -l 1000 COM4
```
LED should keep its steady 1 Hz blink throughout; every request gets a reply.

---

## Notes

- All "raw" bytes in this file include the trailing 2-byte CRC (LSB first).
- CRCs were computed offline; if you tweak a frame, recompute via
  `mb_rtu_crc16()` in the firmware or the Python helper that ships with
  this repo's bring-up notes.
- Modbus master must wait at least 3.5 character times (~4 ms at 9600)
  between consecutive requests, otherwise the slave's IDLE-based frame
  detection may concatenate them. `mbpoll`'s default inter-poll delay
  (-l) is safely above this.

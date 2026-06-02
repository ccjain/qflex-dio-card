# DIO Card — Modbus RTU Slave Firmware (Design Spec)

- **Date:** 2026-06-02
- **Author:** Quench Chargers — firmware (drafted with AI Sherpa)
- **Target hardware:** Quench DIO Card (STM32C092CBTX, 48-pin LQFP)
- **Inputs to this spec:**
  - `DIO_Card_Modbus_Address_Mapping.xlsx` (Modbus register map + DIP→slave-ID table)
  - `LC uC selection sheet.xlsx` (MCU candidates + pin map)

---

## 1. Purpose & Scope

Build the firmware for the Quench DIO Card so it behaves as a **Modbus RTU slave** on an RS-485 bus. A master device on the bus must be able to:

- Read the state of **12 contactor-feedback digital inputs** (function code `0x02`).
- Read and control **7 onboard relays** (function codes `0x01`, `0x05`, `0x0F`).

The board's slave address is configured by a **4-position DIP switch** at boot.

Out of scope for v1 (explicitly): R8 (optional, not populated on this board build), CAN, bootloader/OTA, Modbus diagnostics FCs, holding/input register spaces, LSE/32.768 kHz crystal usage.

---

## 2. Hardware Target

| Item | Value |
|---|---|
| MCU | STM32C092CBTX (ARM Cortex-M0+, 48 MHz, 256 KB Flash / 30 KB RAM, LQFP-48) |
| SYSCLK | HSI 48 MHz internal RC (STM32C0 has no PLL). HSE 8 MHz crystal is populated on the board but **unused in v1** — kept for low-drift / EMI-compliance fallback. |
| LSE | 32.768 kHz crystal on PC14/PC15 — **present, unused in v1** |
| Debug | SWD on PA13 (SWDIO) / PA14 (SWCLK, also BOOT0) |
| Transceiver | RS-485 half-duplex with DE/RE on a single GPIO (PB2) |
| Indicators | One heartbeat LED on PF3 |
| Power | Per board schematic (not specified here) |

---

## 3. Final Pin Map

### 3.1 Relays — Outputs (active-low, push-pull, 7 channels)

GPIO HIGH = relay off; GPIO LOW = coil energized (board uses an inverting driver).

| Modbus Coil (zero-based) | Label  | MCU Pin |
|---|---|---|
| 0 | RELAY1 | PA15 |
| 1 | RELAY2 | PA12 |
| 2 | RELAY3 | PA11 |
| 3 | RELAY4 | PA10 |
| 4 | RELAY5 | PC7  |
| 5 | RELAY6 | PC6  |
| 6 | RELAY7 | PA9  |

All relay GPIOs initialize **HIGH (off)** before USART/Modbus are brought up, so the board cannot momentarily energize a relay during reset.

### 3.2 Feedbacks — Discrete Inputs (active-low w/ internal pull-up, 12 channels)

Pin reads 0 when the external contactor is closed; firmware inverts so the Modbus master sees `1 = contactor closed`.

| Modbus Discrete Input (zero-based) | Label | MCU Pin |
|---|---|---|
| 0  | FB1  | PD0  |
| 1  | FB2  | PD1  |
| 2  | FB3  | PD2  |
| 3  | FB4  | PD3  |
| 4  | FB5  | PB3  |
| 5  | FB6  | PB4  |
| 6  | FB7  | PB5  |
| 7  | FB8  | PB6  |
| 8  | FB9  | PB7  |
| 9  | FB10 | PB8  |
| 10 | FB11 | PB9  |
| 11 | FB12 | PC13 |

### 3.3 DIP Switch — Slave-ID selector (active-low w/ internal pull-up, 4 bits)

Switch closed → pin reads 0 → firmware interprets that bit as `1`.

| DIP position | Weight | MCU Pin |
|---|---|---|
| DIP1 (LSB) | 1 | PA8  |
| DIP2       | 2 | PB15 |
| DIP3       | 4 | PB14 |
| DIP4 (MSB) | 8 | PB13 |

`slave_id = DIP1 + 2·DIP2 + 4·DIP3 + 8·DIP4`, valid range **1-15**.

`slave_id == 0` → board enters **config-error state**: ignores all Modbus traffic, fast-blinks the heartbeat LED (100 ms on / 100 ms off) forever. This is the canonical "DIP switch not set" indicator.

### 3.4 Communication

| Function | MCU Pin | Mode |
|---|---|---|
| USART3_TX  | PB10 | AF, push-pull |
| USART3_RX  | PB11 | AF, input (pull-up off — transceiver drives idle level) |
| RS485_DE/RE| PB2  | GPIO_OUT push-pull, init LOW (RX) |

`PB2 = HIGH` → transceiver in TX mode, `LOW` → RX mode. Toggled by firmware around every outgoing Modbus frame.

### 3.5 Indicator

| Function | MCU Pin |
|---|---|
| HEARTBEAT LED | PF3 (GPIO_OUT, init LOW) |

- Normal: 500 ms toggle (1 Hz blink)
- Config error (slave_id == 0): 100 ms toggle (5 Hz blink)
- Modbus traffic activity flicker: not implemented in v1 (room for future)

### 3.6 Reserved / unused on this board build

| Pin(s) | Original label | Firmware treatment |
|---|---|---|
| PA0-PA4, PA6, PA7 | Vacant on schematic | Initialized as analog input (lowest leakage). |
| PF0, PF1 | 8 MHz HSE crystal pads | Left at reset state (analog input) — HSE not used in v1. |
| PA5 | CAN_STBY | GPIO_OUT push-pull, init HIGH (CAN transceiver in standby — CAN unused in v1). |
| PB0 / PB1 | CAN_RX / CAN_TX | Left at HAL reset state (analog input). |
| PB12 | Was labeled RELAY12 in the pin-map sheet, not populated | Analog input. |
| PC14 / PC15 | LSE 32.768 kHz crystal | Initialized as `RCC_OSC_LSE_DISABLE`; HAL leaves them as analog. |
| PA13 / PA14 | SWDIO / SWCLK (also BOOT0) | AF, default debug function — never reconfigured. |

---

## 4. Modbus RTU Protocol Layer

### 4.1 Serial line settings (fixed in v1)

| Setting | Value |
|---|---|
| Baud rate | 9600 |
| Data bits | 8 |
| Parity    | None |
| Stop bits | 1 |
| Inter-frame silence | 3.5 character times (~3.65 ms @ 9600 8N1) — handled by USART idle-line interrupt |
| Inter-character timeout | 1.5 character times — implicit in DMA byte stream |

### 4.2 Supported function codes

| FC | Name | Address range | Behaviour |
|---|---|---|---|
| `0x01` | Read Coils | 0…6, qty 1…7 | Return current state of relays R1-R7 |
| `0x05` | Write Single Coil | 0…6 | Set one relay; values `0x0000` = off, `0xFF00` = on per spec; any other value → exception `0x03` |
| `0x0F` | Write Multiple Coils | start 0…6, qty s.t. start+qty ≤ 7 | Set a contiguous block of relays |
| `0x02` | Read Discrete Inputs | 0…11, qty 1…12 | Return debounced state of FB1-FB12 |

### 4.3 Error responses

| Condition | Exception |
|---|---|
| Unsupported function code | `0x01` Illegal Function |
| Address out of range, or `start + qty` exceeds the table | `0x02` Illegal Data Address |
| Write-coil value not `0x0000` or `0xFF00` | `0x03` Illegal Data Value |
| Frame to a different slave address | Silent drop (no reply, per Modbus RTU spec) |
| Broadcast (address 0) read | Silent drop (broadcast is write-only per spec) |
| Broadcast write | Apply, send no reply |
| CRC error | Silent drop |

### 4.4 Slave-ID handling

- ID is read **once at boot** from the DIP switch. Changes during runtime are not honoured until reset (matches industry convention; avoids race conditions on a live bus).
- ID 0 → config error mode, see §3.5.

---

## 5. Firmware Architecture

### 5.1 Execution model — bare-metal super-loop

```
main()
├── HAL_Init()
├── SystemClock_Config()           // HSI 48 MHz (no PLL on STM32C0)
├── MX_GPIO_Init()                 // relays HIGH, DE/RE LOW, inputs as IN w/ PU
├── MX_USART3_UART_Init()          // 9600 8N1, DMA RX, IT TX-complete
├── slave_id = dip_switch_read()
├── if (slave_id == 0) config_error_loop()
├── modbus_app_init(slave_id)
└── while (1) {
        feedback_scan();           // 5 ms cadence — read+debounce 12 inputs
        relay_apply();              // mirror coil_state[] to GPIOs
        modbus_app_poll();          // drains rx buffer -> mb_rtu_process -> tx
        heartbeat_tick();           // 500 ms toggle, soft-timer based
    }
```

No FreeRTOS. A 1 ms SysTick drives soft timers used by `feedback_scan`, `heartbeat_tick`, and Modbus framing.

### 5.2 Modbus transport — USART3 + DMA + DE/RE

- **RX path:** `HAL_UARTEx_ReceiveToIdleDMA()` parks DMA on the RX line. The HAL's idle-line callback (`HAL_UARTEx_RxEventCallback`) fires after **1 character time of bus silence** (USART_IDLE flag) → that delimits a Modbus RTU frame. The frame buffer (256 bytes — comfortably larger than any legal Modbus PDU) and a `frame_ready` flag are exposed for `modbus_app_poll()`. We process one frame at a time, so a single buffer is sufficient; DMA is rearmed only after the frame has been consumed.
- **TX path:** `mb_uart_send()` raises DE/RE (PB2=1), calls `HAL_UART_Transmit_IT()`, and the TX-complete ISR drops DE/RE back to RX mode (PB2=0). The Modbus app layer calls this after building each response frame.

This gives us Modbus RTU framing essentially for free: USART idle (1 char) ≥ T1.5 inter-character timeout, and the application's own pacing between request/response cycles delivers the T3.5 inter-frame silence. The Modbus protocol layer (`modbus_rtu.c`, §5.3) validates CRC and frame length on every received PDU.

### 5.3 Module boundaries

| File | Responsibility | Public surface |
|---|---|---|
| `relay.c/.h` | Owns the 7 relay GPIOs. State held in `coil_state[7]`. | `relay_init()`, `relay_set(i, on)`, `relay_get(i)`, `relay_apply()` |
| `feedback.c/.h` | Owns the 12 input GPIOs + 3-sample debounce. State held in `fb_state[12]`. | `feedback_init()`, `feedback_scan()`, `feedback_get(i)` |
| `dip_switch.c/.h` | Reads the 4 DIP pins, returns slave ID. | `dip_switch_read() -> uint8_t` |
| `heartbeat.c/.h` | LED blink with two modes (normal / config-error). | `heartbeat_init(mode)`, `heartbeat_tick()` |
| `mb_uart.c/.h` | USART3 + DMA + DE/RE glue. Hardware-only, no protocol knowledge. | `mb_uart_init()`, `mb_uart_send()`, `mb_uart_rx_ready()`, `mb_uart_rx_buffer()`, `mb_uart_rx_release()` |
| `modbus_rtu.c/.h` | **Pure Modbus RTU protocol layer.** CRC16-Modbus, slave-ID match, frame validation, exception responses. No hardware dependencies — testable on the host. | `mb_rtu_crc16()`, `mb_rtu_process(req, req_len, fc_handlers*, resp, resp_len*)` |
| `modbus_app.c/.h` | Wires `modbus_rtu` to `mb_uart` and provides the 4 FC handlers that bridge to `relay`/`feedback`. | `modbus_app_init(slave_id)`, `modbus_app_poll()` |
| `main.c` | Init order, super-loop, error trap. | — |

Each `.c` is expected to stay under ~200 lines. Every module exposes only what its consumers need; no globals are shared across module boundaries directly. The deliberate split between `modbus_rtu` (pure logic) and `modbus_app` (app glue) means the protocol layer is 100% host-testable.

### 5.4 Data model

```c
// relay.c — internal
static bool coil_state[7];        // mirror of last-commanded relay state

// feedback.c — internal
static bool fb_state[12];         // debounced logical state (1 = contactor closed)
static uint8_t fb_history[12];    // 3-sample rolling buffer for debounce

// dip_switch.c — internal
// no state; pure read at boot
```

### 5.5 Protocol-layer API and FC handler contracts

The `modbus_rtu` layer is a single function pipe: bytes in, bytes out.

```c
typedef enum {
    MB_OK                        = 0x00,
    MB_EXC_ILLEGAL_FUNCTION      = 0x01,
    MB_EXC_ILLEGAL_DATA_ADDRESS  = 0x02,
    MB_EXC_ILLEGAL_DATA_VALUE    = 0x03,
} mb_error_t;

typedef struct {
    mb_error_t (*read_coils)(uint16_t addr, uint16_t qty, uint8_t *out_bits);
    mb_error_t (*write_single_coil)(uint16_t addr, bool value);
    mb_error_t (*write_multiple_coils)(uint16_t addr, uint16_t qty, const uint8_t *in_bits);
    mb_error_t (*read_discrete_inputs)(uint16_t addr, uint16_t qty, uint8_t *out_bits);
} mb_handlers_t;

/* CRC16 with Modbus polynomial 0xA001 (reflected) and seed 0xFFFF. */
uint16_t mb_rtu_crc16(const uint8_t *data, size_t len);

/* Validate, dispatch, and build a response.
 *   `req` / `req_len`   — raw bytes from the UART idle-line buffer
 *   `slave_id`          — our configured slave ID (0 not allowed)
 *   `h`                 — application FC handlers
 *   `resp` / `resp_len` — output buffer + size in/out
 * Returns true if a response frame was produced and should be sent.
 * Returns false if the frame should be silently dropped (wrong slave-ID,
 * bad CRC, broadcast write — caller does NOT transmit). */
bool mb_rtu_process(const uint8_t *req, size_t req_len,
                    uint8_t slave_id,
                    const mb_handlers_t *h,
                    uint8_t *resp, size_t *resp_len);
```

FC handler obligations:

- **Range check:** return `MB_EXC_ILLEGAL_DATA_ADDRESS` if `addr + qty` exceeds the table.
- **Write coil value check:** the protocol layer already rejects FC `0x05` payloads ≠ `0x0000` / `0xFF00` with `MB_EXC_ILLEGAL_DATA_VALUE`; the handler always receives `value` as a clean `bool`.
- **Read handlers:** write `qty` bits into `out_bits` as packed little-endian bytes (LSB-first within each byte, per Modbus spec).
- **Write multiple handler:** read `qty` bits from `in_bits` in the same packing.

The protocol layer (`modbus_rtu.c`) is fully testable on a host: feed it a request frame, assert on the response frame.

---

## 6. Project Layout

We do **not** use the STM32CubeMX GUI. The project is hand-authored against the official **STM32CubeC0** firmware package (vendored as a git submodule), which gives us deterministic, AI-friendly file generation and lets the entire project build and flash from the command line.

```
dio_card/
├── Core/
│   ├── Inc/
│   │   ├── main.h
│   │   ├── pin_map.h                           # all GPIO macros, single source of truth
│   │   ├── app_config.h                        # build-time constants (baud, counts)
│   │   ├── stm32c0xx_it.h
│   │   └── stm32c0xx_hal_conf.h                # hand-tuned: enable HAL_GPIO/UART/DMA/RCC/CORTEX, disable the rest
│   └── Src/
│       ├── main.c
│       ├── stm32c0xx_it.c                      # NMI/HardFault/SysTick + USART3 + DMA1_Channel1 IRQ handlers
│       ├── stm32c0xx_hal_msp.c                 # HAL_MspInit + peripheral MSP init (UART/DMA clocks, pinmux)
│       ├── system_stm32c0xx.c                  # from CMSIS template
│       └── app/
│           ├── relay.c        + relay.h
│           ├── feedback.c     + feedback.h
│           ├── dip_switch.c   + dip_switch.h
│           ├── heartbeat.c    + heartbeat.h
│           ├── mb_uart.c      + mb_uart.h
│           ├── modbus_rtu.c   + modbus_rtu.h        # pure protocol layer (CRC + dispatch + exceptions)
│           └── modbus_app.c   + modbus_app.h        # FC handlers + transport glue
├── Startup/
│   └── startup_stm32c092xx.s                   # copied from CMSIS device startup
├── Linker/
│   └── STM32C092CBTX_FLASH.ld                  # 256 KB Flash @ 0x08000000, 30 KB RAM @ 0x20000000
├── external/
│   └── STM32CubeC0/                            # git submodule: github.com/STMicroelectronics/STM32CubeC0
│       └── Drivers/
│           ├── STM32C0xx_HAL_Driver/           # HAL source — referenced by Makefile, NOT copied
│           ├── CMSIS/                          # core + device headers
│           └── ...
├── Makefile                                    # hand-written, references external/STM32CubeC0 paths
├── flash.bat                                   # one-liner: STM32_Programmer_CLI flash + reset
├── .gitignore                                  # build/, *.o, *.elf, *.bin
├── .gitmodules
├── README.md                                   # build/flash + Modbus map quick-ref
├── docs/
│   └── superpowers/
│       └── specs/
│           └── 2026-06-02-dio-card-modbus-slave-design.md   # this file
├── tests/
│   └── mbpoll_smoke.md                         # PC-side bench-test commands (mbpoll)
├── DIO_Card_Modbus_Address_Mapping.xlsx        # input artifact, preserved
└── LC uC selection sheet.xlsx                  # input artifact, preserved
```

---

## 7. Build, Flash, Debug

Everything is driven from the command line by the AI workflow. ST-LINK V2/V3 is already wired to the board on the developer machine.

**Toolchain (already installed on the dev machine):**

| Tool | Absolute path | Purpose |
|---|---|---|
| GCC | `C:\ST\STM32CubeIDE_1.19.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344\tools\bin\arm-none-eabi-gcc.exe` | Compile / link |
| Programmer CLI | `C:\ST\STM32CubeIDE_1.19.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.200.202503041107\tools\bin\STM32_Programmer_CLI.exe` | Flash via SWD |
| STLink server | `C:\ST\stlink_server\` | Optional shared-session ST-LINK daemon |

The `Makefile` will pin the GCC path via a configurable variable (`GCC_PATH ?= <above>`), so this is portable to any other dev box by overriding once.

**Build:**

```powershell
make -j               # → build/dio_card.elf, dio_card.hex, dio_card.bin, dio_card.map
```

**Flash (ST-LINK over SWD):**

```powershell
STM32_Programmer_CLI -c port=SWD freq=4000 -d build/dio_card.bin 0x08000000 -rst
```

A `flash.bat` will wrap this. The agent runs it as part of v1 verification.

**Debug:** STM32CubeIDE can import the project as a "Makefile project with existing code" and launch SWD debug sessions. v1 does not require interactive debug; printf-via-SWO is also out of scope.

**Vendored dependencies — first-time setup:**

```bash
git submodule add https://github.com/STMicroelectronics/STM32CubeC0.git external/STM32CubeC0
git submodule update --init --recursive
# No third-party Modbus library — the RTU protocol layer is in-tree (Core/Src/app/modbus_rtu.c).
```

---

## 8. Test Strategy

### 8.1 v1 — agent-executed bring-up

The board is wired to ST-LINK on the dev machine. The agent (this AI workflow) is responsible for the full loop:

1. **Build** — `make -j` must produce a clean `.bin` with no warnings under `-Wall -Wextra -Werror`.
2. **Flash** — `STM32_Programmer_CLI ... -d build/dio_card.bin 0x08000000 -rst` returns success.
3. **Sanity** — heartbeat LED on PF3 blinks at 1 Hz immediately after reset (verified visually by the developer; the agent reports "flashed, awaiting LED confirmation").

If the agent's machine also has a USB-RS485 adapter wired to the board's RS-485 port, it also runs the **bench validation** below. If not, the agent generates the test script and asks the developer to run it.

### 8.2 v1 — bench validation (PC-side Modbus master)

Run from a Windows shell, using either `mbpoll` or `Diagslave` against the COM port the USB-RS485 adapter enumerates as. Full command list lives in `tests/mbpoll_smoke.md`; the checklist:

- [ ] Slave answers at its DIP-configured ID, ignores other IDs.
- [ ] DIP = 0 → no reply on any address; LED fast-blinks.
- [ ] `FC 01` reads coils 0-6, returns 0 by default after boot.
- [ ] `FC 05` writing `0xFF00` to coil N energizes relay N (audible click + visible relay LED if fitted); writing `0x0000` releases it.
- [ ] `FC 05` writing any value other than `0x0000`/`0xFF00` → exception `0x03`.
- [ ] `FC 0F` writing 7 coils at offset 0 applies all relays atomically.
- [ ] `FC 02` reads discrete inputs 0-11, reflects the physical contactor state (jumper each input to GND in turn to verify).
- [ ] Any address outside the supported ranges → exception `0x02`.
- [ ] Any unsupported FC (e.g. `0x03` Read Holding Registers) → exception `0x01`.
- [ ] Bus left idle for 60 s → board still responsive (no DMA stall).

### 8.3 v1 — host-side unit tests (optional)

`dip_switch_read()` and the feedback debounce function are pure-logic — they can be cross-compiled against a fake GPIO read function and tested under `ceedling` or a one-file `assert()` harness on the dev machine. **Not blocking for v1**, but the code is structured to allow it.

### 8.4 Future

- HIL via a second STM32 acting as a Modbus master in CI.
- Long-soak burst test (10k frames, jitter measurement).
- printf-via-SWO for in-flight diagnostics.

---

## 9. Risks & Open Items

| Item | Risk | Mitigation |
|---|---|---|
| Schematic mismatch with this pin map | Medium | User confirmed DIP pins (PA8/PB15/PB14/PB13); other pins come straight from the supplied pin-map sheet. First bench bring-up will validate. |
| RS-485 transceiver turn-around timing | Low | DE/RE driven by TX-complete IRQ before any other byte processing — sub-µs lag, well inside Modbus T3.5. |
| EMI on long bus runs | Low-Medium | Not a firmware concern in v1; assume bus terminator + biasing resistors are on the board. |
| Future need for FC 0x03/0x06 (registers) | Low | Add a holding-register table + 2 handlers in `modbus_app.c` and the dispatch in `modbus_rtu.c`. Both are <50 lines of additions. |
| Hand-rolled protocol layer has subtle bugs | Medium | Mitigated by exhaustive host-side unit tests in `tests/host/test_modbus_rtu.c` — every FC, every exception, broadcast, bad CRC, wrong slave-ID. |

---

## 10. Acceptance Criteria

Firmware is "done" when:

1. `make -j` builds cleanly under `-Wall -Wextra -Werror`, no warnings.
2. `STM32_Programmer_CLI` flashes `build/dio_card.bin` and resets the board successfully.
3. After flash, the heartbeat LED on PF3 blinks at 1 Hz (developer-confirmed by eye, or fast 5 Hz blink if DIP = 0).
4. With DIP = N (1-15), the board answers Modbus frames at slave ID N (all checkboxes in §8.2 pass).
5. No file under `Core/Src/app/` exceeds 200 lines.
6. README documents: pin map, Modbus map, build, flash, DIP-to-slave-ID table.
7. Spec, code, and inputs are committed to git on `main` with a meaningful history.

---

## 11. End-to-end command sequence (for the agent)

Once the implementation is complete, the agent runs:

```powershell
# from E:\cjain\stm_workspace\dio_card
git init
git submodule add https://github.com/STMicroelectronics/STM32CubeC0.git external/STM32CubeC0
git submodule update --init --recursive
make -j
.\flash.bat       # wraps STM32_Programmer_CLI -c port=SWD -d build/dio_card.bin 0x08000000 -rst
```

Then waits for the developer to confirm the heartbeat LED and run the bench Modbus test from `tests/mbpoll_smoke.md`.

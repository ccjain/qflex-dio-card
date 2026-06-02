# Quench DIO Card — Modbus RTU Slave Firmware

STM32C092CBTX firmware that exposes **7 relays** and **12 contactor-feedback
inputs** to a Modbus RTU master over RS-485 at 9600 8N1. Slave ID is set
by a 4-position DIP switch read once at boot.

## Quick start

```powershell
# Clone with submodules (the build pulls only the two STM32CubeC0 submodules
# we need: HAL + CMSIS device files).
git clone <repo-url>
cd dio_card
git submodule update --init external/STM32CubeC0
cd external/STM32CubeC0
git submodule update --init Drivers/STM32C0xx_HAL_Driver Drivers/CMSIS/Device/ST/STM32C0xx
cd ../..

# Build (env.bat must point at your STM32CubeIDE toolchain paths)
.\build.bat

# Flash via ST-LINK on SWD
.\flash.bat
```

After flash the heartbeat LED on PF3 blinks at 1 Hz. If it blinks at 5 Hz
the DIP switch reads 0 (all open) and the board is in config-error mode —
set at least one DIP to close and reset.

## Pin map (single source of truth: `Core/Inc/pin_map.h`)

| Resource | MCU pin(s) | Drive |
|---|---|---|
| Relay 1-7 | PA15, PA12, PA11, PA10, PC7, PC6, PA9 | Active-low, push-pull |
| Feedback 1-12 | PD0-3, PB3-9, PC13 | Active-low w/ internal pull-up |
| DIP1-4 (slave ID) | PA8, PB15, PB14, PB13 | Active-low w/ pull-up (DIP1 = LSB, weight 1) |
| USART3 TX / RX / DE | PB10 / PB11 / PB2 | AF4 on PB10-11; DE GPIO push-pull (HIGH = TX) |
| Heartbeat LED | PF3 | Push-pull |
| Reserved (CAN, LSE, vacant) | PB0/PB1, PA5, PC14/PC15, PA0-4/6/7, PF0/PF1, PB12 | Left at HAL reset state |

See the design spec at `docs/superpowers/specs/2026-06-02-dio-card-modbus-slave-design.md`
for the rationale behind every pin.

## Modbus map

| FC | Address range (zero-based) | Maps to |
|---|---|---|
| 0x01 Read Coils | 0..6 | Relays 1-7 |
| 0x05 Write Single Coil | 0..6 | Relay 1-7 (value must be 0x0000 or 0xFF00) |
| 0x0F Write Multiple Coils | 0..6 (start + qty ≤ 7) | Atomic set of relays 1-7 |
| 0x02 Read Discrete Inputs | 0..11 | Feedbacks 1-12 (1 = contactor closed) |

**Exceptions:**
- Out of range or `start + qty` past the table → exception **0x02** (illegal data address)
- Unsupported FC → exception **0x01** (illegal function)
- FC 0x05 with value other than 0x0000 / 0xFF00 → exception **0x03** (illegal data value)
- Wrong slave ID, bad CRC, frame too short → silent drop (no reply per Modbus RTU spec)

Serial line: **9600 baud, 8 data bits, no parity, 1 stop bit**.

## DIP → slave ID

`slave_id = DIP1*1 + DIP2*2 + DIP3*4 + DIP4*8`. Valid range 1-15.
`slave_id = 0` (all DIPs open) puts the board in **config error**: it
ignores all Modbus traffic and fast-blinks the heartbeat LED at 5 Hz.

Switch position → "closed" means the slide is in the ON position, pulling
its pin to GND.

## Toolchain

Built and flashed using **STM32CubeIDE 1.19.0**'s bundled GCC, make, and
`STM32_Programmer_CLI`. The exact absolute paths live in `env.bat`; edit
that single file to retarget another dev box.

The repo deliberately does **not** use STM32CubeMX. The project is
hand-authored against the official STM32CubeC0 firmware package (vendored
as a git submodule).

## Project layout

```
Core/
  Inc/          # main.h, pin_map.h, app_config.h, stm32c0xx_hal_conf.h
  Src/
    main.c
    stm32c0xx_it.c           # ISRs (NMI, HardFault, SysTick, USART3_4)
    stm32c0xx_hal_msp.c      # HAL_MspInit
    system_stm32c0xx.c       # CMSIS template
    app/                     # all application modules
      heartbeat.{c,h}        # PF3 LED, 1 Hz normal / 5 Hz fault
      dip_switch.{c,h}       # one-shot read of the 4-DIP slave ID
      relay.{c,h}            # 7-channel active-low output
      feedback.{c,h}         # 12-channel input w/ 3-sample debounce
      mb_uart.{c,h}          # USART3 transport (bare-register init, IDLE framing)
      modbus_rtu.{c,h}       # pure protocol layer (CRC, validate, dispatch)
      modbus_app.{c,h}       # FC handlers bridging mb_uart <-> mb_rtu_process
Startup/                     # CMSIS startup_stm32c092xx.s
Linker/                      # STM32C092CBTX_FLASH.ld (128 KB Flash / 30 KB RAM)
external/STM32CubeC0/        # vendored HAL + CMSIS device submodules
docs/superpowers/
  specs/                     # design spec (this firmware's contract)
  plans/                     # task-by-task implementation plan
tests/
  mbpoll_smoke.md            # PC-side smoke tests (raw frames + mbpoll cmds)
  host/                      # host-compiled C tests (CRC, RTU, dip_decode, debounce)
```

## Testing

### Live bench tests
See `tests/mbpoll_smoke.md`. Quick PowerShell one-liner against COM4 and slave 1:

```powershell
$p = New-Object System.IO.Ports.SerialPort 'COM4',9600,'None',8,'One'
$p.Open(); $p.DiscardInBuffer()
# Read 7 coils:
$p.Write([byte[]]@(0x01,0x01,0x00,0x00,0x00,0x07,0x7D,0xC8),0,8)
Start-Sleep -Milliseconds 200
while ($p.BytesToRead -gt 0) { '{0:X2}' -f $p.ReadByte() }
$p.Close()
```

### Host-side unit tests
`tests/host/` contains C tests for the pure-logic functions
(`mb_rtu_crc16`, `mb_rtu_process`, `dip_decode`, `feedback_debounce_*`).
A host C compiler is required (MinGW-w64 or MSVC) — not currently on
PATH in this workspace. The test files are structured to compile with
`-DHOST_UNIT_TEST` once a host compiler is installed; see the plan doc
for the build commands.

CRC test vectors were verified offline in Python during bring-up.

## Known deviations from the design spec

1. **UART init bypasses HAL.** `HAL_UART_Init` does not bring USART3
   fully online on STM32C092 with CubeC0 v1.4.0 (full debug trail in the
   Phase D git history). The transport uses register-level init for
   USART3 itself; HAL is used for GPIO/RCC/NVIC/SysTick.

2. **No DMA on RX.** The original plan called for DMA + idle-line; the
   final implementation uses `RXNE` + `IDLE` interrupts. At 9600 baud
   the byte-rate is trivial (~960/s max) and this avoids any further
   HAL/DMA interaction.

3. **No PLL.** STM32C0 has no PLL. SYSCLK runs from HSI directly at
   48 MHz. The board's 8 MHz HSE crystal is populated but unused in v1.

## Authoring

This firmware was authored task-by-task per the implementation plan at
`docs/superpowers/plans/2026-06-02-dio-card-modbus-slave.md`. Each commit
on the `main` branch corresponds to one plan task.

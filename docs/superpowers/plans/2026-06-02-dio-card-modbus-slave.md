# DIO Card — Modbus RTU Slave Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:subagent-driven-development` (recommended) or `superpowers:executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build, flash, and bench-verify Modbus RTU slave firmware for the Quench DIO Card (STM32C092CBTX) so a Modbus master on RS-485 can read 12 contactor-feedback inputs and read/write 7 relays at a DIP-configured slave ID.

**Architecture:** Bare-metal super-loop on STM32 HAL. USART3 transports Modbus RTU via idle-line DMA RX + interrupt TX with a GPIO DE/RE around each TX. Slave ID read once at boot from a 4-DIP switch. An in-tree Modbus RTU protocol layer (`modbus_rtu.c` — CRC16, frame validation, FC dispatch, exception responses) is wired to application FC handlers in `modbus_app.c` that talk to the `relay` and `feedback` modules. The protocol layer has no hardware dependencies and is fully covered by host-side unit tests. STM32CubeC0 is vendored as a git submodule; no CubeMX GUI dance.

**Tech Stack:**
- MCU: STM32C092CBTX (Cortex-M0+ @ 48 MHz, 256 KB Flash / 30 KB RAM)
- Toolchain: `arm-none-eabi-gcc 13.3.1` bundled with STM32CubeIDE 1.19.0 (`C:\ST\STM32CubeIDE_1.19.0\...`)
- Build: hand-written `Makefile` driven by `make.exe` bundled with CubeIDE
- Flash: `STM32_Programmer_CLI.exe` over SWD (ST-LINK)
- HAL: STM32CubeC0 v1.x (git submodule of `github.com/STMicroelectronics/STM32CubeC0`)
- Modbus stack: **in-tree**, ~400 lines C, host-tested (`Core/Src/app/modbus_rtu.{c,h}`)

**Reference:** Design spec at `docs/superpowers/specs/2026-06-02-dio-card-modbus-slave-design.md`. All pin assignments, Modbus mapping, exception behavior, and acceptance criteria live there — this plan implements that spec.

---

## File Structure

Files this plan creates or modifies (every path is relative to `E:\cjain\stm_workspace\dio_card\`):

| Path | Created by | Purpose |
|---|---|---|
| `.gitignore` | Task 1 | Exclude `build/`, `*.elf`, `*.bin`, `*.o`, IDE cruft |
| `.gitmodules` | Task 2 (via git) | STM32CubeC0 submodule registration |
| `external/STM32CubeC0/` | Task 2 | Vendored HAL + CMSIS (submodule) |
| `Linker/STM32C092CBTX_FLASH.ld` | Task 3 | Memory layout (256K Flash @ 0x08000000, 30K RAM @ 0x20000000) |
| `Startup/startup_stm32c092xx.s` | Task 4 | CMSIS device startup (vector table, Reset_Handler) |
| `Core/Src/system_stm32c0xx.c` | Task 4 | CMSIS system init |
| `Core/Inc/stm32c0xx_hal_conf.h` | Task 5 | HAL module enable/disable |
| `Core/Inc/main.h` | Task 5 | App-wide includes |
| `Core/Inc/app_config.h` | Task 5 | Compile-time constants (counts, baud) |
| `Core/Inc/stm32c0xx_it.h` | Task 5 | ISR prototypes |
| `Core/Src/stm32c0xx_hal_msp.c` | Task 5 | HAL MSP base hooks |
| `Core/Src/stm32c0xx_it.c` | Task 5 | NMI/HardFault/SysTick handlers |
| `Core/Src/main.c` | Task 5 (skeleton) → Task 6 (clock) → Task 17 (super-loop) | App entry |
| `Makefile` | Task 5 | Build recipe |
| `env.bat` | Task 5 | PATH setup for toolchain |
| `build.bat`, `flash.bat` | Task 5 / Task 19 | One-shot CLI wrappers |
| `Core/Inc/pin_map.h` | Task 8 | All GPIO macros (single source of truth) |
| `Core/Src/app/heartbeat.{c,h}` | Task 7 | LED blink |
| `Core/Src/app/dip_switch.{c,h}` | Task 9 | Slave-ID read |
| `Core/Src/app/relay.{c,h}` | Task 10 | 7-relay output driver |
| `Core/Src/app/feedback.{c,h}` | Task 11 | 12-input debounced reader |
| `Core/Src/app/mb_uart.{c,h}` | Tasks 12-14 | USART3 + DMA + DE/RE transport |
| `Core/Src/app/modbus_rtu.{c,h}` | Tasks 15-16 | In-tree Modbus RTU protocol layer (CRC, validate, dispatch, exceptions) |
| `Core/Src/app/modbus_app.{c,h}` | Task 17 | FC handlers + transport glue |
| `tests/host/` (CMakeLists + tests) | Tasks 9, 11, 15, 16 | Pure-logic unit tests (host-compiled) — includes exhaustive Modbus RTU coverage |
| `tests/mbpoll_smoke.md` | Task 22 | Bench-test command list |
| `README.md` | Task 23 | Build/flash/test quick reference |

No file under `Core/Src/app/` should exceed 200 lines — that's an acceptance criterion (spec §10).

---

## Task 1: Bootstrap git repo and ignores

**Files:**
- Create: `.gitignore`

- [ ] **Step 1: Initialize git**

Run from `E:\cjain\stm_workspace\dio_card`:

```powershell
git init -b main
git config user.name  "Quench Firmware"
git config user.email "kmittal@quenchchargers.com"
```

Expected: `Initialized empty Git repository in .../dio_card/.git/`

- [ ] **Step 2: Write `.gitignore`**

Create `.gitignore` with:

```gitignore
# Build artifacts
build/
*.o
*.d
*.elf
*.bin
*.hex
*.map
*.list

# IDE
.vscode/
.idea/
*.launch
.settings/
.cproject
.project
.metadata/

# OS
Thumbs.db
.DS_Store

# Local-only env overrides
env.local.bat
```

- [ ] **Step 3: First commit (inputs + spec + plan + ignores)**

```powershell
git add .gitignore docs/ "DIO_Card_Modbus_Address_Mapping.xlsx" "LC uC selection sheet.xlsx"
git commit -m "chore: bootstrap repo with spec, plan, and input artifacts"
```

Expected: a commit on `main` containing those files. Verify with `git log --oneline -1`.

---

## Task 2: Vendor dependencies (STM32CubeC0 only)

**Files:**
- Create: `external/STM32CubeC0/` (submodule)
- Create: `.gitmodules`

The Modbus stack is in-tree (Tasks 15-17), so the only third-party dependency is ST's HAL/CMSIS package.

- [ ] **Step 1: Add STM32CubeC0 submodule (selective init)**

STM32CubeC0 itself contains many nested submodules (Nucleo BSPs, ThreadX/USBX/FileX middlewares, mbed-crypto) we don't need. Pull only the HAL driver and CMSIS device:

```powershell
git submodule add https://github.com/STMicroelectronics/STM32CubeC0.git external/STM32CubeC0
cd external/STM32CubeC0
git submodule update --init Drivers/STM32C0xx_HAL_Driver Drivers/CMSIS/Device/ST/STM32C0xx
cd ../..
```

(A full `--recursive` would pull ~600 MB. The selective form keeps us under 50 MB.) Verify these paths exist:

```
external/STM32CubeC0/Drivers/STM32C0xx_HAL_Driver/Src/stm32c0xx_hal.c
external/STM32CubeC0/Drivers/CMSIS/Device/ST/STM32C0xx/Include/stm32c092xx.h
external/STM32CubeC0/Drivers/CMSIS/Device/ST/STM32C0xx/Source/Templates/gcc/startup_stm32c092xx.s
```

If `stm32c092xx.h` doesn't exist, the submodule version is too old — check out the latest tag:
```powershell
cd external/STM32CubeC0; git fetch --tags; git checkout v1.2.0 ; cd ../..
```
(Adjust tag version as needed — STM32C092 was added in CubeC0 v1.2.x.)

- [ ] **Step 2: Commit**

```powershell
git add .gitmodules
git commit -m "chore: vendor STM32CubeC0 as git submodule"
```

Verify: `git log --oneline -2` shows the commit; `git submodule status` shows the cubec0 SHA.

---

## Task 3: Linker script

**Files:**
- Create: `Linker/STM32C092CBTX_FLASH.ld`

- [ ] **Step 1: Write the linker script**

The STM32C092CBTX has 256 KB Flash starting at 0x08000000 and 30 KB SRAM starting at 0x20000000 (from datasheet RM0490). Write `Linker/STM32C092CBTX_FLASH.ld`:

```ld
/* Linker script for STM32C092CBTX
 * Flash: 256 KB @ 0x08000000
 * RAM  : 30  KB @ 0x20000000
 * Stack: 4   KB at top of RAM (grows down)
 * Heap : disabled (bare-metal, no malloc)
 */

ENTRY(Reset_Handler)

_estack    = ORIGIN(RAM) + LENGTH(RAM);  /* end of RAM */
_Min_Heap_Size  = 0x0;                   /* no heap */
_Min_Stack_Size = 0x1000;                /* 4 KB stack */

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 256K
  RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 30K
}

SECTIONS
{
  /* Vector table at start of flash */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  /* Code + read-only constants */
  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    KEEP (*(.init))
    KEEP (*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM : {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH
  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH
  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  /* Initialized data: load from flash, copy to RAM by Reset_Handler */
  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT> FLASH

  /* Zero-initialized data */
  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM

  /* User heap + stack region — only used to enforce min sizes */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /DISCARD/ : { libc.a ( * ) libm.a ( * ) libgcc.a ( * ) }
  .ARM.attributes 0 : { *(.ARM.attributes) }
}
```

- [ ] **Step 2: Commit**

```powershell
git add Linker/STM32C092CBTX_FLASH.ld
git commit -m "build: add linker script for STM32C092CBTX"
```

---

## Task 4: Startup and CMSIS system files

**Files:**
- Create: `Startup/startup_stm32c092xx.s` (copied from CubeC0)
- Create: `Core/Src/system_stm32c0xx.c` (copied from CubeC0)

- [ ] **Step 1: Copy the device startup file**

```powershell
mkdir Startup
copy external\STM32CubeC0\Drivers\CMSIS\Device\ST\STM32C0xx\Source\Templates\gcc\startup_stm32c092xx.s Startup\
```

Verify the file exists and contains `Reset_Handler:`, `g_pfnVectors`, and an interrupt vector for `USART3_LPUART2_IRQHandler` and `DMA1_Channel1_IRQHandler` (or similar — STM32C092 IRQ names vary; we'll wire whichever names appear here in Task 13).

- [ ] **Step 2: Copy the CMSIS system file**

```powershell
mkdir Core\Src
copy external\STM32CubeC0\Drivers\CMSIS\Device\ST\STM32C0xx\Source\Templates\system_stm32c0xx.c Core\Src\
```

- [ ] **Step 3: Commit**

```powershell
git add Startup/ Core/Src/system_stm32c0xx.c
git commit -m "build: import CMSIS startup and system files for STM32C092"
```

---

## Task 5: Minimal skeleton — first clean build

**Files:**
- Create: `Core/Inc/main.h`
- Create: `Core/Inc/app_config.h`
- Create: `Core/Inc/stm32c0xx_it.h`
- Create: `Core/Inc/stm32c0xx_hal_conf.h`
- Create: `Core/Src/main.c`
- Create: `Core/Src/stm32c0xx_it.c`
- Create: `Core/Src/stm32c0xx_hal_msp.c`
- Create: `Makefile`
- Create: `env.bat`
- Create: `build.bat`

- [ ] **Step 1: `env.bat` — PATH setup**

```bat
@echo off
set "CUBEIDE_DIR=C:\ST\STM32CubeIDE_1.19.0\STM32CubeIDE\plugins"
set "GCC_DIR=%CUBEIDE_DIR%\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344\tools\bin"
set "MAKE_DIR=%CUBEIDE_DIR%\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845\tools\bin"
set "PROG_DIR=%CUBEIDE_DIR%\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.200.202503041107\tools\bin"
set "PATH=%GCC_DIR%;%MAKE_DIR%;%PROG_DIR%;%PATH%"
```

- [ ] **Step 2: `build.bat`**

```bat
@echo off
call "%~dp0env.bat"
make -j %*
```

- [ ] **Step 3: `Core/Inc/app_config.h`**

```c
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define APP_RELAY_COUNT       7
#define APP_FEEDBACK_COUNT   12
#define APP_DIP_BIT_COUNT     4

#define APP_MODBUS_BAUD       9600u
#define APP_MODBUS_FRAME_BUF  256u

#define APP_HEARTBEAT_NORMAL_MS    500u
#define APP_HEARTBEAT_FAULT_MS     100u

#define APP_FEEDBACK_SCAN_PERIOD_MS  5u
#define APP_FEEDBACK_DEBOUNCE_N      3u

#endif
```

- [ ] **Step 4: `Core/Inc/stm32c0xx_hal_conf.h` — minimal HAL config**

Copy `external/STM32CubeC0/Drivers/STM32C0xx_HAL_Driver/Inc/stm32c0xx_hal_conf_template.h` to `Core/Inc/stm32c0xx_hal_conf.h`, then edit to keep ONLY these `HAL_MODULE_ENABLED` defines (comment out all others):

```c
#define HAL_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED
#define HAL_EXTI_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED   /* required by HAL itself */
#define HAL_PWR_MODULE_ENABLED     /* required by clock config */
```

Also set:
```c
#define HSE_VALUE    8000000U
#define LSE_VALUE    32768U
#define TICK_INT_PRIORITY  0U
```

- [ ] **Step 5: `Core/Inc/main.h`**

```c
#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32c0xx_hal.h"
#include "app_config.h"

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif
```

- [ ] **Step 6: `Core/Inc/stm32c0xx_it.h`**

```c
#ifndef STM32C0xx_IT_H
#define STM32C0xx_IT_H

#ifdef __cplusplus
extern "C" {
#endif

void NMI_Handler(void);
void HardFault_Handler(void);
void SVC_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

#ifdef __cplusplus
}
#endif

#endif
```

- [ ] **Step 7: `Core/Src/stm32c0xx_it.c` — base ISRs**

```c
#include "main.h"
#include "stm32c0xx_it.h"

void NMI_Handler(void)        { while (1) {} }
void HardFault_Handler(void)  { while (1) {} }
void SVC_Handler(void)        {}
void PendSV_Handler(void)     {}
void SysTick_Handler(void)    { HAL_IncTick(); }
```

- [ ] **Step 8: `Core/Src/stm32c0xx_hal_msp.c`**

```c
#include "main.h"

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}
```

- [ ] **Step 9: `Core/Src/main.c` — skeleton**

```c
#include "main.h"

void Error_Handler(void) { __disable_irq(); while (1) {} }

int main(void)
{
    HAL_Init();
    /* Clock + peripherals come in later tasks. */
    while (1) {
        /* nothing yet */
    }
}
```

- [ ] **Step 10: `Makefile` — hand-written**

```makefile
# DIO Card — STM32C092CBTX Modbus RTU Slave firmware
# Run via build.bat (which sources env.bat for the toolchain on PATH).

TARGET     = dio_card
BUILD_DIR  = build
DEBUG      = 1
OPT        = -Og

MCU_FAMILY = STM32C092xx
MCU_FLAGS  = -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft

# ----- Paths -----
CUBE        = external/STM32CubeC0
HAL_DIR     = $(CUBE)/Drivers/STM32C0xx_HAL_Driver
CMSIS_DEV   = $(CUBE)/Drivers/CMSIS/Device/ST/STM32C0xx
CMSIS_CORE  = $(CUBE)/Drivers/CMSIS

# ----- Sources -----
C_SOURCES = \
  Core/Src/main.c \
  Core/Src/stm32c0xx_it.c \
  Core/Src/stm32c0xx_hal_msp.c \
  Core/Src/system_stm32c0xx.c \
  $(HAL_DIR)/Src/stm32c0xx_hal.c \
  $(HAL_DIR)/Src/stm32c0xx_hal_cortex.c \
  $(HAL_DIR)/Src/stm32c0xx_hal_dma.c \
  $(HAL_DIR)/Src/stm32c0xx_hal_dma_ex.c \
  $(HAL_DIR)/Src/stm32c0xx_hal_exti.c \
  $(HAL_DIR)/Src/stm32c0xx_hal_flash.c \
  $(HAL_DIR)/Src/stm32c0xx_hal_flash_ex.c \
  $(HAL_DIR)/Src/stm32c0xx_hal_gpio.c \
  $(HAL_DIR)/Src/stm32c0xx_hal_pwr.c \
  $(HAL_DIR)/Src/stm32c0xx_hal_pwr_ex.c \
  $(HAL_DIR)/Src/stm32c0xx_hal_rcc.c \
  $(HAL_DIR)/Src/stm32c0xx_hal_rcc_ex.c \
  $(HAL_DIR)/Src/stm32c0xx_hal_uart.c \
  $(HAL_DIR)/Src/stm32c0xx_hal_uart_ex.c

ASM_SOURCES = Startup/startup_stm32c092xx.s

# ----- Tools (must be on PATH via env.bat) -----
PREFIX  = arm-none-eabi-
CC      = $(PREFIX)gcc
AS      = $(PREFIX)gcc -x assembler-with-cpp
OBJCOPY = $(PREFIX)objcopy
SIZE    = $(PREFIX)size

# ----- Flags -----
DEFS    = -D$(MCU_FAMILY) -DUSE_HAL_DRIVER
INCS    = \
  -ICore/Inc \
  -ICore/Src/app \
  -I$(HAL_DIR)/Inc \
  -I$(CMSIS_DEV)/Include \
  -I$(CMSIS_CORE)/Include

CFLAGS  = $(MCU_FLAGS) $(DEFS) $(INCS) $(OPT) \
          -Wall -Wextra -Werror -Wno-unused-parameter \
          -fdata-sections -ffunction-sections \
          -ffreestanding -fno-builtin \
          -MMD -MP

ifeq ($(DEBUG),1)
  CFLAGS += -g3 -gdwarf-2
endif

ASFLAGS = $(MCU_FLAGS) $(OPT) -Wall -fdata-sections -ffunction-sections
ifeq ($(DEBUG),1)
  ASFLAGS += -g3 -gdwarf-2
endif

LDSCRIPT = Linker/STM32C092CBTX_FLASH.ld
LDFLAGS  = $(MCU_FLAGS) --specs=nano.specs --specs=nosys.specs \
           -T$(LDSCRIPT) \
           -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref \
           -Wl,--gc-sections \
           -Wl,--no-warn-rwx-segments

# ----- Objects -----
OBJS  = $(addprefix $(BUILD_DIR)/, $(notdir $(C_SOURCES:.c=.o)))
OBJS += $(addprefix $(BUILD_DIR)/, $(notdir $(ASM_SOURCES:.s=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

# ----- Rules -----
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -o $@ $<

$(BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	$(AS) -c $(ASFLAGS) -o $@ $<

$(BUILD_DIR)/$(TARGET).elf: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $@
	$(SIZE) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf
	$(OBJCOPY) -O binary -S $< $@

$(BUILD_DIR):
	@if not exist "$(BUILD_DIR)" mkdir "$(BUILD_DIR)"

clean:
	@if exist "$(BUILD_DIR)" rmdir /s /q "$(BUILD_DIR)"

-include $(OBJS:.o=.d)

.PHONY: all clean
```

- [ ] **Step 11: First build — verify the empty skeleton compiles**

```powershell
.\build.bat
```

Expected: a clean build ending with a `size` line showing flash & ram usage (tens of KB and zero RAM, since nothing's running yet). `build/dio_card.elf`, `dio_card.hex`, `dio_card.bin` must exist.

If the build fails with `stm32c092xx.h: No such file or directory` — STM32CubeC0 version is too old, see Task 2 step 1 notes.

If it fails with `multiple definition of SystemInit` — verify only ONE copy of `system_stm32c0xx.c` is in `C_SOURCES`.

- [ ] **Step 12: Commit**

```powershell
git add Makefile env.bat build.bat Core/
git commit -m "build: minimal compilable skeleton (empty main, HAL stubs)"
```

---

## Task 6: System clock — HSE 8 MHz → 48 MHz

**Files:**
- Modify: `Core/Src/main.c`

- [ ] **Step 1: Add `SystemClock_Config()` to `main.c`**

Replace the contents of `Core/Src/main.c` with:

```c
#include "main.h"

static void SystemClock_Config(void);

void Error_Handler(void) { __disable_irq(); while (1) {} }

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    while (1) {
        /* peripherals next task */
    }
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    /* HSE on, PLL: HSE/1 * 12 / 2 = 48 MHz */
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM       = RCC_PLLM_DIV1;
    osc.PLL.PLLN       = 12;
    osc.PLL.PLLP       = RCC_PLLP_DIV2;
    osc.PLL.PLLR       = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) { Error_Handler(); }

    clk.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
                       | RCC_CLOCKTYPE_PCLK1;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_1) != HAL_OK) { Error_Handler(); }
}
```

**Note on PLL math:** the STM32C0 PLL uses `(HSE / PLLM) * PLLN / PLLR` for SYSCLK. With HSE=8, M=1, N=12, R=2 → 48 MHz. PLLP/PLLQ unused here. If the C0 HAL on your CubeC0 version omits `PLLR`, drop it; some C0 PLLs only expose `PLLP`. Verify by opening `external/STM32CubeC0/Drivers/STM32C0xx_HAL_Driver/Inc/stm32c0xx_hal_rcc.h` and grepping for `PLLR`.

- [ ] **Step 2: Build**

```powershell
.\build.bat
```

Expected: clean build, no warnings.

- [ ] **Step 3: Commit**

```powershell
git add Core/Src/main.c
git commit -m "init: configure system clock to 48 MHz via HSE+PLL"
```

---

## Task 7: Heartbeat LED — first on-board verification

**Files:**
- Create: `Core/Src/app/heartbeat.c`
- Create: `Core/Src/app/heartbeat.h`
- Modify: `Core/Src/main.c`
- Modify: `Makefile` (add `Core/Src/app/heartbeat.c` to `C_SOURCES`)

- [ ] **Step 1: `Core/Src/app/heartbeat.h`**

```c
#ifndef APP_HEARTBEAT_H
#define APP_HEARTBEAT_H

#include <stdint.h>

typedef enum {
    HEARTBEAT_NORMAL = 0,
    HEARTBEAT_FAULT  = 1,
} heartbeat_mode_t;

void heartbeat_init(heartbeat_mode_t mode);
void heartbeat_set_mode(heartbeat_mode_t mode);
void heartbeat_tick(void);   /* call from super-loop; uses HAL_GetTick internally */

#endif
```

- [ ] **Step 2: `Core/Src/app/heartbeat.c`**

```c
#include "heartbeat.h"
#include "main.h"

#define HB_PORT  GPIOF
#define HB_PIN   GPIO_PIN_3

static heartbeat_mode_t s_mode;
static uint32_t s_last_ms;

static uint32_t period_ms(void) {
    return (s_mode == HEARTBEAT_FAULT) ? APP_HEARTBEAT_FAULT_MS
                                       : APP_HEARTBEAT_NORMAL_MS;
}

void heartbeat_init(heartbeat_mode_t mode) {
    __HAL_RCC_GPIOF_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Pin   = HB_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(HB_PORT, &g);
    HAL_GPIO_WritePin(HB_PORT, HB_PIN, GPIO_PIN_RESET);

    s_mode    = mode;
    s_last_ms = HAL_GetTick();
}

void heartbeat_set_mode(heartbeat_mode_t mode) { s_mode = mode; }

void heartbeat_tick(void) {
    uint32_t now = HAL_GetTick();
    if ((now - s_last_ms) >= period_ms()) {
        HAL_GPIO_TogglePin(HB_PORT, HB_PIN);
        s_last_ms = now;
    }
}
```

- [ ] **Step 3: Wire `heartbeat` into `main.c`**

Update `Core/Src/main.c` so the super-loop calls the tick:

```c
#include "main.h"
#include "heartbeat.h"

static void SystemClock_Config(void);

void Error_Handler(void) { __disable_irq(); while (1) {} }

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    heartbeat_init(HEARTBEAT_NORMAL);

    while (1) {
        heartbeat_tick();
    }
}

/* SystemClock_Config: unchanged from Task 6 */
static void SystemClock_Config(void) { /* ... keep body from Task 6 ... */ }
```

(Keep the full body of `SystemClock_Config` from Task 6 — repeated here only because the file is short.)

- [ ] **Step 4: Update Makefile**

In the `C_SOURCES` list, add:

```makefile
  Core/Src/app/heartbeat.c \
```

(`-ICore/Src/app` is already in `INCS` from Task 5, and `vpath` picks the directory up automatically via `sort dir`.)

- [ ] **Step 5: Build**

```powershell
.\build.bat
```

Expected: clean build. Size line should show a tiny bump (≈100 bytes) from the heartbeat code.

- [ ] **Step 6: Flash the board (FIRST on-board verification)**

```powershell
call env.bat
STM32_Programmer_CLI -c port=SWD freq=4000 -d build\dio_card.bin 0x08000000 -rst
```

Expected output ends with `File download complete` and `RUNNING Program`. The board's heartbeat LED (PF3) should now blink at 1 Hz (500 ms on, 500 ms off).

**STOP HERE and wait for the developer to confirm by eye:** "Is the LED blinking at 1 Hz?" — if no, debug GPIO clock / pin map / clock config before continuing.

- [ ] **Step 7: Commit**

```powershell
git add Core/Src/app/heartbeat.c Core/Src/app/heartbeat.h Core/Src/main.c Makefile
git commit -m "feat: heartbeat LED on PF3 blinks at 1 Hz"
```

---

## Task 8: Centralized pin map

**Files:**
- Create: `Core/Inc/pin_map.h`

- [ ] **Step 1: Write `Core/Inc/pin_map.h`**

This becomes the single source of truth for every GPIO macro. Every subsequent module references these symbols, never raw `PA15`/`GPIOA`/`GPIO_PIN_15`.

```c
#ifndef PIN_MAP_H
#define PIN_MAP_H

#include "stm32c0xx_hal.h"

/* ------------------------------------------------------------------
 * Heartbeat LED
 * ----------------------------------------------------------------*/
#define PIN_HEARTBEAT_PORT     GPIOF
#define PIN_HEARTBEAT_PIN      GPIO_PIN_3
#define PIN_HEARTBEAT_CLK_EN() __HAL_RCC_GPIOF_CLK_ENABLE()

/* ------------------------------------------------------------------
 * Relays (active-low, 7 channels)
 * Indexed 0..6 to match Modbus coil addresses.
 * ----------------------------------------------------------------*/
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} pin_t;

#define RELAY_PINS_INIT { \
    { GPIOA, GPIO_PIN_15 },  /* R1 */ \
    { GPIOA, GPIO_PIN_12 },  /* R2 */ \
    { GPIOA, GPIO_PIN_11 },  /* R3 */ \
    { GPIOA, GPIO_PIN_10 },  /* R4 */ \
    { GPIOC, GPIO_PIN_7  },  /* R5 */ \
    { GPIOC, GPIO_PIN_6  },  /* R6 */ \
    { GPIOA, GPIO_PIN_9  },  /* R7 */ \
}
#define RELAY_CLK_EN() do { \
    __HAL_RCC_GPIOA_CLK_ENABLE(); \
    __HAL_RCC_GPIOC_CLK_ENABLE(); \
} while (0)

/* Active-low: LOW energises the coil. */
#define RELAY_ON_STATE   GPIO_PIN_RESET
#define RELAY_OFF_STATE  GPIO_PIN_SET

/* ------------------------------------------------------------------
 * Feedback inputs (active-low w/ pull-up, 12 channels)
 * ----------------------------------------------------------------*/
#define FEEDBACK_PINS_INIT { \
    { GPIOD, GPIO_PIN_0  },  /* FB1  */ \
    { GPIOD, GPIO_PIN_1  },  /* FB2  */ \
    { GPIOD, GPIO_PIN_2  },  /* FB3  */ \
    { GPIOD, GPIO_PIN_3  },  /* FB4  */ \
    { GPIOB, GPIO_PIN_3  },  /* FB5  */ \
    { GPIOB, GPIO_PIN_4  },  /* FB6  */ \
    { GPIOB, GPIO_PIN_5  },  /* FB7  */ \
    { GPIOB, GPIO_PIN_6  },  /* FB8  */ \
    { GPIOB, GPIO_PIN_7  },  /* FB9  */ \
    { GPIOB, GPIO_PIN_8  },  /* FB10 */ \
    { GPIOB, GPIO_PIN_9  },  /* FB11 */ \
    { GPIOC, GPIO_PIN_13 },  /* FB12 */ \
}
#define FEEDBACK_CLK_EN() do { \
    __HAL_RCC_GPIOB_CLK_ENABLE(); \
    __HAL_RCC_GPIOC_CLK_ENABLE(); \
    __HAL_RCC_GPIOD_CLK_ENABLE(); \
} while (0)

/* ------------------------------------------------------------------
 * DIP switch (active-low w/ pull-up, 4 bits: PA8 = LSB)
 * ----------------------------------------------------------------*/
#define DIP_PINS_INIT { \
    { GPIOA, GPIO_PIN_8  },  /* DIP1 (LSB, weight 1) */ \
    { GPIOB, GPIO_PIN_15 },  /* DIP2 (weight 2)      */ \
    { GPIOB, GPIO_PIN_14 },  /* DIP3 (weight 4)      */ \
    { GPIOB, GPIO_PIN_13 },  /* DIP4 (MSB, weight 8) */ \
}
#define DIP_CLK_EN() do { \
    __HAL_RCC_GPIOA_CLK_ENABLE(); \
    __HAL_RCC_GPIOB_CLK_ENABLE(); \
} while (0)

/* ------------------------------------------------------------------
 * USART3 + RS485 DE/RE
 * ----------------------------------------------------------------*/
#define PIN_UART_TX_PORT       GPIOB
#define PIN_UART_TX_PIN        GPIO_PIN_10
#define PIN_UART_RX_PORT       GPIOB
#define PIN_UART_RX_PIN        GPIO_PIN_11
#define PIN_UART_AF            GPIO_AF4_USART3     /* verify in datasheet */
#define PIN_UART_CLK_EN()      __HAL_RCC_USART3_CLK_ENABLE()

#define PIN_RS485_DE_PORT      GPIOB
#define PIN_RS485_DE_PIN       GPIO_PIN_2
#define PIN_RS485_DE_CLK_EN()  __HAL_RCC_GPIOB_CLK_ENABLE()

#define RS485_TX_MODE() HAL_GPIO_WritePin(PIN_RS485_DE_PORT, PIN_RS485_DE_PIN, GPIO_PIN_SET)
#define RS485_RX_MODE() HAL_GPIO_WritePin(PIN_RS485_DE_PORT, PIN_RS485_DE_PIN, GPIO_PIN_RESET)

#endif /* PIN_MAP_H */
```

**Note on `PIN_UART_AF`:** the alternate function for USART3 TX on PB10 may be `GPIO_AF4_USART3` or a different AF on STM32C0 — verify by grepping `external/STM32CubeC0/Drivers/CMSIS/Device/ST/STM32C0xx/Include/stm32c092xx.h` for the AF mapping, OR by checking `Drivers/STM32C0xx_HAL_Driver/Inc/stm32c0xx_hal_gpio_ex.h`. Fix the macro if needed before Task 12.

- [ ] **Step 2: Build (sanity — should still compile, no other file uses pin_map.h yet)**

```powershell
.\build.bat
```

Expected: clean build.

- [ ] **Step 3: Commit**

```powershell
git add Core/Inc/pin_map.h
git commit -m "feat: centralize pin map in pin_map.h"
```

---

## Task 9: DIP-switch module + host-compiled unit test

**Files:**
- Create: `Core/Src/app/dip_switch.h`
- Create: `Core/Src/app/dip_switch.c`
- Create: `tests/host/CMakeLists.txt`
- Create: `tests/host/test_dip_decode.c`
- Modify: `Makefile` (add `dip_switch.c` to `C_SOURCES`)

The decode function (raw 4-bit reading → slave ID) is pure logic — testable on the host. The GPIO read is hardware. Split them.

- [ ] **Step 1: `Core/Src/app/dip_switch.h`**

```c
#ifndef APP_DIP_SWITCH_H
#define APP_DIP_SWITCH_H

#include <stdint.h>

/* Pure-logic helper: take 4 pin-level booleans (closed=true, open=false)
 * and return the encoded slave ID 0..15. Exposed for unit testing. */
uint8_t dip_decode(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3);

/* Initialise the 4 DIP pins as input pull-up and return the configured
 * slave ID. Call exactly once at boot. */
uint8_t dip_switch_read(void);

#endif
```

- [ ] **Step 2: `Core/Src/app/dip_switch.c`**

```c
#include "dip_switch.h"
#include "main.h"
#include "pin_map.h"

uint8_t dip_decode(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3)
{
    return (uint8_t)((b0 & 1u)
                   | ((b1 & 1u) << 1)
                   | ((b2 & 1u) << 2)
                   | ((b3 & 1u) << 3));
}

uint8_t dip_switch_read(void)
{
    static const pin_t dip[APP_DIP_BIT_COUNT] = DIP_PINS_INIT;
    DIP_CLK_EN();

    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    for (int i = 0; i < APP_DIP_BIT_COUNT; ++i) {
        g.Pin = dip[i].pin;
        HAL_GPIO_Init(dip[i].port, &g);
    }

    /* Settling time for the pull-ups against external wiring. */
    HAL_Delay(2);

    /* Switch closed -> pin reads 0 -> bit value 1. Invert. */
    uint8_t b[APP_DIP_BIT_COUNT];
    for (int i = 0; i < APP_DIP_BIT_COUNT; ++i) {
        b[i] = (HAL_GPIO_ReadPin(dip[i].port, dip[i].pin) == GPIO_PIN_RESET) ? 1u : 0u;
    }
    return dip_decode(b[0], b[1], b[2], b[3]);
}
```

- [ ] **Step 3: Write the host-side test BEFORE wiring it up — TDD**

`tests/host/test_dip_decode.c`:

```c
#include "../../Core/Src/app/dip_switch.h"
#include <assert.h>
#include <stdio.h>

/* Re-declare to avoid pulling in HAL on the host: */
uint8_t dip_decode(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3);

int main(void) {
    assert(dip_decode(0,0,0,0) == 0);   /* all open  */
    assert(dip_decode(1,0,0,0) == 1);   /* LSB only  */
    assert(dip_decode(0,1,0,0) == 2);
    assert(dip_decode(1,1,0,0) == 3);
    assert(dip_decode(0,0,1,0) == 4);
    assert(dip_decode(1,1,1,1) == 15);  /* all closed */
    /* Sanity: only the low bit matters per arg */
    assert(dip_decode(2,0,0,0) == 0);   /* even -> bit 0 = 0 */
    assert(dip_decode(3,0,0,0) == 1);   /* odd  -> bit 0 = 1 */
    printf("dip_decode: OK\n");
    return 0;
}
```

`tests/host/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.16)
project(dio_card_host_tests C)
set(CMAKE_C_STANDARD 11)

# Compile dip_decode standalone — it has no HAL dependency.
add_executable(test_dip_decode
    test_dip_decode.c
    ../../Core/Src/app/dip_switch.c)
target_compile_definitions(test_dip_decode PRIVATE HOST_UNIT_TEST)
target_include_directories(test_dip_decode PRIVATE ../../Core/Inc ../../Core/Src/app)
```

We need `dip_switch.c` to skip the HAL-using `dip_switch_read` when built for the host. Wrap that function in `#ifndef HOST_UNIT_TEST`:

Edit `Core/Src/app/dip_switch.c` and put the entire `dip_switch_read()` body inside `#ifndef HOST_UNIT_TEST ... #endif`:

```c
#include "dip_switch.h"

#ifndef HOST_UNIT_TEST
#include "main.h"
#include "pin_map.h"
#endif

uint8_t dip_decode(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3)
{
    return (uint8_t)((b0 & 1u)
                   | ((b1 & 1u) << 1)
                   | ((b2 & 1u) << 2)
                   | ((b3 & 1u) << 3));
}

#ifndef HOST_UNIT_TEST
uint8_t dip_switch_read(void)
{
    /* ...same body as before... */
}
#endif
```

- [ ] **Step 4: Run the test — should pass**

```powershell
cd tests\host
cmake -B build -S .
cmake --build build
.\build\Debug\test_dip_decode.exe
```

Expected: `dip_decode: OK`. If `cmake` isn't on PATH, install it or write a one-line gcc compile invocation:

```powershell
gcc -I..\..\Core\Inc -I..\..\Core\Src\app -DHOST_UNIT_TEST test_dip_decode.c ..\..\Core\Src\app\dip_switch.c -o test_dip_decode.exe
.\test_dip_decode.exe
```

(Either path works; whichever the dev box can run.) If it FAILS, fix `dip_decode` and rerun.

- [ ] **Step 5: Add `dip_switch.c` to the firmware Makefile**

Insert into `C_SOURCES`:

```makefile
  Core/Src/app/dip_switch.c \
```

- [ ] **Step 6: Build the firmware**

```powershell
cd ..\..
.\build.bat
```

Expected: clean build.

- [ ] **Step 7: Commit**

```powershell
git add Core/Src/app/dip_switch.* tests/host/ Makefile
git commit -m "feat: dip_switch module with pure-logic host-side test"
```

---

## Task 10: Relay output module

**Files:**
- Create: `Core/Src/app/relay.h`
- Create: `Core/Src/app/relay.c`
- Modify: `Makefile` (add `relay.c`)

- [ ] **Step 1: `Core/Src/app/relay.h`**

```c
#ifndef APP_RELAY_H
#define APP_RELAY_H

#include <stdbool.h>
#include <stdint.h>

void relay_init(void);
bool relay_get(uint8_t index);          /* index 0..APP_RELAY_COUNT-1 */
void relay_set(uint8_t index, bool on); /* updates in-memory state    */
void relay_apply(void);                 /* writes state to GPIOs      */

#endif
```

- [ ] **Step 2: `Core/Src/app/relay.c`**

```c
#include "relay.h"
#include "main.h"
#include "pin_map.h"

static const pin_t s_relay[APP_RELAY_COUNT] = RELAY_PINS_INIT;
static bool        s_state[APP_RELAY_COUNT];

void relay_init(void)
{
    RELAY_CLK_EN();

    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;

    for (uint8_t i = 0; i < APP_RELAY_COUNT; ++i) {
        /* Drive OFF before enabling output to avoid glitch. */
        HAL_GPIO_WritePin(s_relay[i].port, s_relay[i].pin, RELAY_OFF_STATE);
        g.Pin = s_relay[i].pin;
        HAL_GPIO_Init(s_relay[i].port, &g);
        s_state[i] = false;
    }
}

bool relay_get(uint8_t index)
{
    return (index < APP_RELAY_COUNT) ? s_state[index] : false;
}

void relay_set(uint8_t index, bool on)
{
    if (index < APP_RELAY_COUNT) {
        s_state[index] = on;
    }
}

void relay_apply(void)
{
    for (uint8_t i = 0; i < APP_RELAY_COUNT; ++i) {
        HAL_GPIO_WritePin(s_relay[i].port, s_relay[i].pin,
                          s_state[i] ? RELAY_ON_STATE : RELAY_OFF_STATE);
    }
}
```

- [ ] **Step 3: Add to Makefile**

```makefile
  Core/Src/app/relay.c \
```

- [ ] **Step 4: Build**

```powershell
.\build.bat
```

Expected: clean build.

- [ ] **Step 5: Commit**

```powershell
git add Core/Src/app/relay.* Makefile
git commit -m "feat: relay output module (7 channels, active-low)"
```

---

## Task 11: Feedback input module with debounce + host test

**Files:**
- Create: `Core/Src/app/feedback.h`
- Create: `Core/Src/app/feedback.c`
- Create: `tests/host/test_feedback_debounce.c`
- Modify: `tests/host/CMakeLists.txt`
- Modify: `Makefile`

- [ ] **Step 1: `Core/Src/app/feedback.h`**

```c
#ifndef APP_FEEDBACK_H
#define APP_FEEDBACK_H

#include <stdbool.h>
#include <stdint.h>

/* Pure-logic debounce step: takes the previous shift-register, the new
 * raw sample (0/1), and returns the new shift-register. The debounced
 * output is "all 1s" or "all 0s" in the low N bits — see
 * `feedback_debounce_value`. Exposed for unit testing. */
uint8_t feedback_debounce_shift(uint8_t history, uint8_t raw_bit);

/* Returns true once `history` is full of 1s (debounced HIGH), false once
 * full of 0s (debounced LOW), otherwise returns `previous`. */
bool feedback_debounce_value(uint8_t history, bool previous);

void feedback_init(void);
void feedback_scan(void);          /* call every APP_FEEDBACK_SCAN_PERIOD_MS */
bool feedback_get(uint8_t index);  /* debounced logical state, 1 = contactor closed */

#endif
```

- [ ] **Step 2: Write the host test FIRST**

`tests/host/test_feedback_debounce.c`:

```c
#include "../../Core/Src/app/feedback.h"
#include <assert.h>
#include <stdio.h>

int main(void) {
    /* shift_left semantics: new bit shifted into LSB */
    uint8_t h = 0;
    h = feedback_debounce_shift(h, 1); assert(h == 0x01);
    h = feedback_debounce_shift(h, 1); assert(h == 0x03);
    h = feedback_debounce_shift(h, 1); assert(h == 0x07);   /* 3 in a row */
    /* low-bit mask of width APP_FEEDBACK_DEBOUNCE_N == 3 -> 0x07 means all-high */

    /* debounce_value: needs N matching bits to switch */
    bool prev = false;
    assert(feedback_debounce_value(0x07, prev) == true);
    assert(feedback_debounce_value(0x06, prev) == false);   /* not all 1s, stays prev */
    assert(feedback_debounce_value(0x00, true)  == false);  /* all 0s -> switch off */
    assert(feedback_debounce_value(0x05, true)  == true);   /* mixed -> stay prev */

    printf("feedback_debounce: OK\n");
    return 0;
}
```

- [ ] **Step 3: `Core/Src/app/feedback.c`**

```c
#include "feedback.h"

#ifndef HOST_UNIT_TEST
#include "main.h"
#include "pin_map.h"
#endif

#include "app_config.h"

#define DEBOUNCE_N    APP_FEEDBACK_DEBOUNCE_N
#define DEBOUNCE_MASK ((1u << DEBOUNCE_N) - 1u)

uint8_t feedback_debounce_shift(uint8_t history, uint8_t raw_bit)
{
    history = (uint8_t)((history << 1) | (raw_bit & 1u));
    return (uint8_t)(history & DEBOUNCE_MASK);
}

bool feedback_debounce_value(uint8_t history, bool previous)
{
    if ((history & DEBOUNCE_MASK) == DEBOUNCE_MASK) return true;
    if ((history & DEBOUNCE_MASK) == 0u)            return false;
    return previous;
}

#ifndef HOST_UNIT_TEST

static const pin_t s_fb[APP_FEEDBACK_COUNT] = FEEDBACK_PINS_INIT;
static uint8_t     s_hist[APP_FEEDBACK_COUNT];
static bool        s_value[APP_FEEDBACK_COUNT];
static uint32_t    s_last_ms;

void feedback_init(void)
{
    FEEDBACK_CLK_EN();
    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    for (uint8_t i = 0; i < APP_FEEDBACK_COUNT; ++i) {
        g.Pin = s_fb[i].pin;
        HAL_GPIO_Init(s_fb[i].port, &g);
        s_hist[i]  = 0;
        s_value[i] = false;
    }
    s_last_ms = HAL_GetTick();
}

void feedback_scan(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - s_last_ms) < APP_FEEDBACK_SCAN_PERIOD_MS) return;
    s_last_ms = now;

    for (uint8_t i = 0; i < APP_FEEDBACK_COUNT; ++i) {
        /* Pin reads 0 when contactor is closed -> logical 1. Invert here. */
        uint8_t raw = (HAL_GPIO_ReadPin(s_fb[i].port, s_fb[i].pin) == GPIO_PIN_RESET) ? 1u : 0u;
        s_hist[i]  = feedback_debounce_shift(s_hist[i], raw);
        s_value[i] = feedback_debounce_value(s_hist[i], s_value[i]);
    }
}

bool feedback_get(uint8_t index)
{
    return (index < APP_FEEDBACK_COUNT) ? s_value[index] : false;
}

#endif /* HOST_UNIT_TEST */
```

- [ ] **Step 4: Update `tests/host/CMakeLists.txt`**

Append:

```cmake
add_executable(test_feedback_debounce
    test_feedback_debounce.c
    ../../Core/Src/app/feedback.c)
target_compile_definitions(test_feedback_debounce PRIVATE HOST_UNIT_TEST)
target_include_directories(test_feedback_debounce PRIVATE ../../Core/Inc ../../Core/Src/app)
```

- [ ] **Step 5: Run host tests**

```powershell
cd tests\host
cmake --build build
.\build\Debug\test_dip_decode.exe
.\build\Debug\test_feedback_debounce.exe
```

Expected: both print `OK`. (If CMake unavailable, use direct gcc as in Task 9.)

- [ ] **Step 6: Add to firmware Makefile**

```makefile
  Core/Src/app/feedback.c \
```

- [ ] **Step 7: Build firmware**

```powershell
cd ..\..
.\build.bat
```

Expected: clean build.

- [ ] **Step 8: Commit**

```powershell
git add Core/Src/app/feedback.* tests/host/CMakeLists.txt tests/host/test_feedback_debounce.c Makefile
git commit -m "feat: feedback input module with 3-sample debounce + host test"
```

---

## Task 12: USART3 init (transport — no Modbus yet)

**Files:**
- Create: `Core/Src/app/mb_uart.h`
- Create: `Core/Src/app/mb_uart.c`
- Modify: `Makefile`

This task only sets up USART3 + the DE/RE GPIO. DMA + idle-line come in Task 13. We verify by transmitting a fixed byte and watching the line with a scope or a serial sniffer.

- [ ] **Step 1: `Core/Src/app/mb_uart.h` (skeleton)**

```c
#ifndef APP_MB_UART_H
#define APP_MB_UART_H

#include <stddef.h>
#include <stdint.h>

void mb_uart_init(void);

/* Blocking-style helpers used in early bring-up. The
 * frame-buffer API (mb_uart_rx_ready, mb_uart_send) is added in later tasks. */
void mb_uart_tx_blocking(const uint8_t *data, size_t len);

#endif
```

- [ ] **Step 2: `Core/Src/app/mb_uart.c`**

```c
#include "mb_uart.h"
#include "main.h"
#include "pin_map.h"

static UART_HandleTypeDef s_uart;

void mb_uart_init(void)
{
    /* GPIO clocks. */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    PIN_UART_CLK_EN();
    PIN_RS485_DE_CLK_EN();

    /* DE/RE pin: output, default RX mode. */
    GPIO_InitTypeDef g = {0};
    g.Pin   = PIN_RS485_DE_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PIN_RS485_DE_PORT, &g);
    RS485_RX_MODE();

    /* TX/RX AF pins. */
    g.Pin       = PIN_UART_TX_PIN;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = PIN_UART_AF;
    HAL_GPIO_Init(PIN_UART_TX_PORT, &g);

    g.Pin       = PIN_UART_RX_PIN;
    HAL_GPIO_Init(PIN_UART_RX_PORT, &g);

    /* USART3 9600 8N1. */
    s_uart.Instance        = USART3;
    s_uart.Init.BaudRate   = APP_MODBUS_BAUD;
    s_uart.Init.WordLength = UART_WORDLENGTH_8B;
    s_uart.Init.StopBits   = UART_STOPBITS_1;
    s_uart.Init.Parity     = UART_PARITY_NONE;
    s_uart.Init.Mode       = UART_MODE_TX_RX;
    s_uart.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    s_uart.Init.OverSampling     = UART_OVERSAMPLING_16;
    s_uart.Init.OneBitSampling   = UART_ONE_BIT_SAMPLE_DISABLE;
    s_uart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&s_uart) != HAL_OK) { Error_Handler(); }
}

void mb_uart_tx_blocking(const uint8_t *data, size_t len)
{
    RS485_TX_MODE();
    (void)HAL_UART_Transmit(&s_uart, (uint8_t *)data, (uint16_t)len, 1000);
    RS485_RX_MODE();
}
```

- [ ] **Step 3: Wire it into main, transmit a periodic ping**

In `Core/Src/main.c`, add includes + call init, and emit a heartbeat byte every second:

```c
#include "main.h"
#include "heartbeat.h"
#include "mb_uart.h"

static void SystemClock_Config(void);
void Error_Handler(void) { __disable_irq(); while (1) {} }

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    heartbeat_init(HEARTBEAT_NORMAL);
    mb_uart_init();

    uint32_t last = HAL_GetTick();
    while (1) {
        heartbeat_tick();
        if (HAL_GetTick() - last >= 1000) {
            last = HAL_GetTick();
            const uint8_t ping = 'P';
            mb_uart_tx_blocking(&ping, 1);
        }
    }
}

static void SystemClock_Config(void) { /* keep body from Task 6 */ }
```

- [ ] **Step 4: Add to Makefile**

```makefile
  Core/Src/app/mb_uart.c \
```

- [ ] **Step 5: Build + flash**

```powershell
.\build.bat
call env.bat
STM32_Programmer_CLI -c port=SWD freq=4000 -d build\dio_card.bin 0x08000000 -rst
```

- [ ] **Step 6: On-board verification**

On the RS-485 bus (or hooked to a USB-RS485 adapter), open a serial monitor at 9600 8N1. Expected: a single `P` (0x50) every second. DE/RE pin (PB2) should pulse high during each byte.

**STOP HERE and confirm the developer sees the `P` byte coming out once per second.** If silent: check AF macro, USART3 clock, PB10/11 pin config; scope the TX pin.

- [ ] **Step 7: Commit**

```powershell
git add Core/Src/app/mb_uart.* Core/Src/main.c Makefile
git commit -m "feat: USART3 9600 8N1 transport, DE/RE on PB2 (1 Hz 'P' ping)"
```

---

## Task 13: DMA RX + idle-line callback (frame reception)

**Files:**
- Modify: `Core/Src/app/mb_uart.c` (add DMA RX)
- Modify: `Core/Src/app/mb_uart.h`
- Modify: `Core/Src/stm32c0xx_it.c` (add USART3 + DMA IRQ handlers)
- Modify: `Core/Src/stm32c0xx_hal_msp.c` (UART MSP init)
- Modify: `Core/Src/main.c` (echo any received frame back, for verification)

The receive path uses `HAL_UARTEx_ReceiveToIdleDMA` so any pause ≥ 1 char on the bus terminates the current frame.

- [ ] **Step 1: Update `mb_uart.h`**

```c
#ifndef APP_MB_UART_H
#define APP_MB_UART_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "app_config.h"

void mb_uart_init(void);
void mb_uart_tx_blocking(const uint8_t *data, size_t len);

/* Frame reception: returns true and fills *len iff a complete frame is
 * waiting in the internal buffer. Caller then consumes from `mb_uart_rx_buffer()`
 * and calls `mb_uart_rx_release()` to re-arm DMA. */
bool        mb_uart_rx_ready(size_t *len);
const uint8_t *mb_uart_rx_buffer(void);
void        mb_uart_rx_release(void);

/* HAL callbacks (called from ISR context). Declared here so the IRQ
 * handlers in stm32c0xx_it.c can find them. */
void mb_uart_on_rx_event(uint16_t size);

#endif
```

- [ ] **Step 2: Update `mb_uart.c`**

Replace the contents with:

```c
#include "mb_uart.h"
#include "main.h"
#include "pin_map.h"
#include <string.h>

static UART_HandleTypeDef s_uart;
static DMA_HandleTypeDef  s_dma_rx;
static uint8_t            s_rx_buf[APP_MODBUS_FRAME_BUF];
static volatile uint16_t  s_rx_len;
static volatile bool      s_rx_ready;

void mb_uart_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    PIN_UART_CLK_EN();
    PIN_RS485_DE_CLK_EN();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DE/RE. */
    GPIO_InitTypeDef g = {0};
    g.Pin   = PIN_RS485_DE_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PIN_RS485_DE_PORT, &g);
    RS485_RX_MODE();

    /* TX/RX. */
    g.Mode      = GPIO_MODE_AF_PP;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = PIN_UART_AF;
    g.Pin       = PIN_UART_TX_PIN; HAL_GPIO_Init(PIN_UART_TX_PORT, &g);
    g.Pin       = PIN_UART_RX_PIN; HAL_GPIO_Init(PIN_UART_RX_PORT, &g);

    /* DMA RX channel 1 -> USART3_RX. */
    s_dma_rx.Instance                 = DMA1_Channel1;
    s_dma_rx.Init.Request             = DMA_REQUEST_USART3_RX;
    s_dma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    s_dma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    s_dma_rx.Init.MemInc              = DMA_MINC_ENABLE;
    s_dma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    s_dma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    s_dma_rx.Init.Mode                = DMA_NORMAL;
    s_dma_rx.Init.Priority            = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&s_dma_rx) != HAL_OK) { Error_Handler(); }

    /* UART. */
    s_uart.Instance        = USART3;
    s_uart.Init.BaudRate   = APP_MODBUS_BAUD;
    s_uart.Init.WordLength = UART_WORDLENGTH_8B;
    s_uart.Init.StopBits   = UART_STOPBITS_1;
    s_uart.Init.Parity     = UART_PARITY_NONE;
    s_uart.Init.Mode       = UART_MODE_TX_RX;
    s_uart.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    s_uart.Init.OverSampling   = UART_OVERSAMPLING_16;
    s_uart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    s_uart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    s_uart.hdmarx = &s_dma_rx;
    __HAL_LINKDMA(&s_uart, hdmarx, s_dma_rx);
    if (HAL_UART_Init(&s_uart) != HAL_OK) { Error_Handler(); }

    /* IRQ priorities (everything at 0; only one priority level on M0+). */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    /* USART3 IRQ may be USART3_USART4_USART5_USART6_LPUART1_IRQn or similar
     * on STM32C0 — verify by opening Startup/startup_stm32c092xx.s and
     * locating the actual symbol. */
    HAL_NVIC_SetPriority(USART3_LPUART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_LPUART1_IRQn);

    /* Arm RX. */
    s_rx_ready = false;
    s_rx_len   = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(&s_uart, s_rx_buf, sizeof(s_rx_buf));
    __HAL_DMA_DISABLE_IT(&s_dma_rx, DMA_IT_HT); /* we only care about TC + idle */
}

void mb_uart_tx_blocking(const uint8_t *data, size_t len)
{
    RS485_TX_MODE();
    (void)HAL_UART_Transmit(&s_uart, (uint8_t *)data, (uint16_t)len, 1000);
    RS485_RX_MODE();
}

bool mb_uart_rx_ready(size_t *len)
{
    if (!s_rx_ready) return false;
    if (len) *len = s_rx_len;
    return true;
}

const uint8_t *mb_uart_rx_buffer(void) { return s_rx_buf; }

void mb_uart_rx_release(void)
{
    s_rx_ready = false;
    s_rx_len   = 0;
    HAL_UARTEx_ReceiveToIdle_DMA(&s_uart, s_rx_buf, sizeof(s_rx_buf));
    __HAL_DMA_DISABLE_IT(&s_dma_rx, DMA_IT_HT);
}

/* HAL callbacks fire from ISR context. */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance == USART3) {
        s_rx_len   = size;
        s_rx_ready = true;
    }
}

/* Provide ISR-entry wrappers that the IRQ handlers in stm32c0xx_it.c call. */
void mb_uart_on_rx_event(uint16_t size) { (void)size; /* handled by HAL callback */ }

/* HAL needs these via weak overrides — route them to our static handles. */
UART_HandleTypeDef *mb_uart_handle(void)  { return &s_uart; }
DMA_HandleTypeDef  *mb_uart_dma_rx(void)  { return &s_dma_rx; }
```

Add the two accessor prototypes to `mb_uart.h`:

```c
UART_HandleTypeDef *mb_uart_handle(void);
DMA_HandleTypeDef  *mb_uart_dma_rx(void);
```

- [ ] **Step 3: Wire the IRQ handlers in `stm32c0xx_it.c`**

Replace `Core/Src/stm32c0xx_it.c`:

```c
#include "main.h"
#include "stm32c0xx_it.h"
#include "mb_uart.h"

void NMI_Handler(void)       { while (1) {} }
void HardFault_Handler(void) { while (1) {} }
void SVC_Handler(void)       {}
void PendSV_Handler(void)    {}
void SysTick_Handler(void)   { HAL_IncTick(); }

void DMA1_Channel1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(mb_uart_dma_rx());
}

void USART3_LPUART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(mb_uart_handle());
}
```

**Note:** the IRQ handler name `USART3_LPUART1_IRQHandler` matches the symbol in the STM32C092 startup file. Verify by opening `Startup/startup_stm32c092xx.s` and grepping for `USART3` — use whatever name appears (the IRQ may be shared with LPUART1/USART4 depending on silicon revision). The DMA channel name (`DMA1_Channel1_IRQHandler`) is fixed.

- [ ] **Step 4: Update `stm32c0xx_hal_msp.c`**

Add UART MSP init so HAL_UART_Init can set up the clock. Replace contents:

```c
#include "main.h"

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}

/* GPIO + clock for the UART are set up in mb_uart_init directly — keep this
 * weak override as a no-op so HAL_UART_Init does not call CubeMX-style code. */
void HAL_UART_MspInit(UART_HandleTypeDef *huart) { (void)huart; }
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart) { (void)huart; }
```

- [ ] **Step 5: Update `main.c` — echo received frames**

```c
#include "main.h"
#include "heartbeat.h"
#include "mb_uart.h"
#include <string.h>

static void SystemClock_Config(void);
void Error_Handler(void) { __disable_irq(); while (1) {} }

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    heartbeat_init(HEARTBEAT_NORMAL);
    mb_uart_init();

    while (1) {
        heartbeat_tick();
        size_t len;
        if (mb_uart_rx_ready(&len)) {
            /* Copy out then release before transmitting — TX may take time. */
            uint8_t tmp[APP_MODBUS_FRAME_BUF];
            memcpy(tmp, mb_uart_rx_buffer(), len);
            mb_uart_rx_release();
            mb_uart_tx_blocking(tmp, len);
        }
    }
}

static void SystemClock_Config(void) { /* keep body from Task 6 */ }
```

- [ ] **Step 6: Build + flash**

```powershell
.\build.bat
call env.bat
STM32_Programmer_CLI -c port=SWD freq=4000 -d build\dio_card.bin 0x08000000 -rst
```

- [ ] **Step 7: Loopback verification**

From a PC serial terminal at 9600 8N1 (over USB-RS485 adapter):
- Send `AB CD`. Board should echo `AB CD` back after the bus goes idle.
- Send `01 02 03 04 05`. Board echoes the full 5-byte frame.

**STOP** and confirm echo works before continuing.

- [ ] **Step 8: Commit**

```powershell
git add Core/Src/app/mb_uart.* Core/Src/stm32c0xx_it.c Core/Src/stm32c0xx_hal_msp.c Core/Src/main.c
git commit -m "feat: DMA RX + idle-line detection + loopback echo on USART3"
```

---

## Task 14: TX path with DE/RE timing under interrupt

**Files:**
- Modify: `Core/Src/app/mb_uart.c` (replace blocking TX with IT-based TX + TC callback)
- Modify: `Core/Src/app/mb_uart.h`

The blocking TX from Task 13 holds DE high until `HAL_UART_Transmit` returns, which is fine but not ISR-safe. Replace with `HAL_UART_Transmit_IT` and drop DE in `HAL_UART_TxCpltCallback`.

- [ ] **Step 1: Replace TX block in `mb_uart.c`**

Remove `mb_uart_tx_blocking` and add:

```c
static volatile bool s_tx_busy;

void mb_uart_send(const uint8_t *data, size_t len)
{
    /* Spin until any in-flight TX finishes (Modbus is half-duplex; the
     * application never queues two responses anyway). */
    while (s_tx_busy) { /* yield */ }
    s_tx_busy = true;
    RS485_TX_MODE();
    if (HAL_UART_Transmit_IT(&s_uart, (uint8_t *)data, (uint16_t)len) != HAL_OK) {
        s_tx_busy = false;
        RS485_RX_MODE();
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        /* Tiny pause so the last stop bit clears the shift register before
         * we let the transceiver drop the line. ~ 1 bit time @ 9600 ≈ 104 µs;
         * the HAL TC callback already fires after TC flag, so 0 is safe on
         * most STM32 — but a 1-cycle nop block costs nothing. */
        for (volatile int i = 0; i < 50; ++i) { __NOP(); }
        RS485_RX_MODE();
        s_tx_busy = false;
    }
}
```

Update `mb_uart.h`: replace `mb_uart_tx_blocking` with:

```c
void mb_uart_send(const uint8_t *data, size_t len);
bool mb_uart_tx_busy(void);
```

Add to `mb_uart.c`:

```c
bool mb_uart_tx_busy(void) { return s_tx_busy; }
```

- [ ] **Step 2: Update `main.c` echo to use the new API**

```c
/* in the rx-ready branch, replace mb_uart_tx_blocking with: */
mb_uart_send(tmp, len);
```

- [ ] **Step 3: Build + flash**

```powershell
.\build.bat
call env.bat
STM32_Programmer_CLI -c port=SWD freq=4000 -d build\dio_card.bin 0x08000000 -rst
```

- [ ] **Step 4: Verify echo still works (regression)**

Send a few frames from the PC; confirm the same echo behaviour as Task 13. Scope DE (PB2) if possible: it should go LOW within ~1 char time after the final byte's stop bit.

- [ ] **Step 5: Commit**

```powershell
git add Core/Src/app/mb_uart.* Core/Src/main.c
git commit -m "feat: interrupt-driven TX with DE/RE release on TXC"
```

---

## Task 15: CRC16-Modbus + host test (TDD)

**Files:**
- Create: `Core/Src/app/modbus_rtu.h` (CRC declaration only at this stage)
- Create: `Core/Src/app/modbus_rtu.c` (CRC implementation only)
- Create: `tests/host/test_modbus_crc.c`
- Modify: `tests/host/CMakeLists.txt`
- Modify: `Makefile`

The CRC16-Modbus polynomial is `0xA001` (reflected, seed `0xFFFF`). It's the foundation everything else depends on, and it's trivial to test on the host with known-good vectors.

- [ ] **Step 1: Write the host test FIRST**

`tests/host/test_modbus_crc.c`:

```c
#include "../../Core/Src/app/modbus_rtu.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

/* Known-good Modbus CRC16 vectors:
 *   "0102"         -> CRC value 0xE181, on the wire LSB-first as "81 E1"
 *   "01030000000A" -> CRC value 0xCDC5, on the wire LSB-first as "C5 CD"
 * The function returns the CRC as a host uint16_t; the caller appends
 * low-byte-first when serialising onto the bus.
 */
int main(void) {
    {
        uint8_t  v[] = {0x01, 0x02};
        uint16_t c = mb_rtu_crc16(v, sizeof(v));
        assert(c == 0xE181);
    }
    {
        uint8_t  v[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x0A};
        uint16_t c = mb_rtu_crc16(v, sizeof(v));
        assert(c == 0xCDC5);
    }
    {
        /* Empty payload: seed echoes back. */
        uint16_t c = mb_rtu_crc16(NULL, 0);
        assert(c == 0xFFFF);
    }
    printf("modbus_crc16: OK\n");
    return 0;
}
```

(The two non-trivial vectors above are the classic textbook examples; the implementation in Step 3 must reproduce them.)

- [ ] **Step 2: `Core/Src/app/modbus_rtu.h` — header at this stage**

```c
#ifndef APP_MODBUS_RTU_H
#define APP_MODBUS_RTU_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

uint16_t mb_rtu_crc16(const uint8_t *data, size_t len);

#endif
```

- [ ] **Step 3: `Core/Src/app/modbus_rtu.c` — CRC implementation**

```c
#include "modbus_rtu.h"

uint16_t mb_rtu_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i];
        for (int b = 0; b < 8; ++b) {
            if (crc & 1u) {
                crc = (crc >> 1) ^ 0xA001u;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}
```

- [ ] **Step 4: Wire up host test**

Append to `tests/host/CMakeLists.txt`:

```cmake
add_executable(test_modbus_crc
    test_modbus_crc.c
    ../../Core/Src/app/modbus_rtu.c)
target_include_directories(test_modbus_crc PRIVATE ../../Core/Inc ../../Core/Src/app)
```

- [ ] **Step 5: Run host test — must pass**

```powershell
cd tests\host
cmake --build build
.\build\Debug\test_modbus_crc.exe
```

Expected: `modbus_crc16: OK`. If FAIL, debug the bit-shift loop in `mb_rtu_crc16`.

- [ ] **Step 6: Add `modbus_rtu.c` to firmware Makefile and build**

In `Makefile` `C_SOURCES`:

```makefile
  Core/Src/app/modbus_rtu.c \
```

Run:

```powershell
cd ..\..
.\build.bat
```

Expected: clean build (`modbus_rtu.c` adds maybe 100 bytes of code).

- [ ] **Step 7: Commit**

```powershell
git add Core/Src/app/modbus_rtu.* tests/host/test_modbus_crc.c tests/host/CMakeLists.txt Makefile
git commit -m "feat(modbus): CRC16-Modbus with known-vector host tests"
```

---

## Task 16: Modbus RTU protocol layer + exhaustive host tests

**Files:**
- Modify: `Core/Src/app/modbus_rtu.h` (add `mb_handlers_t`, `mb_error_t`, `mb_rtu_process`)
- Modify: `Core/Src/app/modbus_rtu.c` (frame validate, FC dispatch, exception builder)
- Create: `tests/host/test_modbus_rtu.c`
- Modify: `tests/host/CMakeLists.txt`

This task implements the full pure-logic RTU protocol layer. Every behaviour from spec §4 is exercised by a host test before we touch hardware.

- [ ] **Step 1: Extend `Core/Src/app/modbus_rtu.h` with the full API**

Replace the contents of `modbus_rtu.h` with:

```c
#ifndef APP_MODBUS_RTU_H
#define APP_MODBUS_RTU_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

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

uint16_t mb_rtu_crc16(const uint8_t *data, size_t len);

/* Validate, dispatch, and build the RTU response.
 *
 * Returns true  : *resp_len bytes were written to resp[]; caller MUST transmit.
 * Returns false : silent drop — wrong slave-ID, bad CRC, broadcast write
 *                 success, or malformed length. Caller MUST NOT transmit. */
bool mb_rtu_process(const uint8_t *req, size_t req_len,
                    uint8_t slave_id,
                    const mb_handlers_t *h,
                    uint8_t *resp, size_t *resp_len);

#endif
```

- [ ] **Step 2: Write the host test FIRST — exhaustive coverage**

`tests/host/test_modbus_rtu.c`:

```c
#include "../../Core/Src/app/modbus_rtu.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ---------- test fixtures: tiny in-memory model ---------- */
static uint8_t fake_coils[8];      /* 8 coils to test out-of-range on coil 7 */
static uint8_t fake_inputs[12];    /* 12 discrete inputs */

static mb_error_t h_read_coils(uint16_t addr, uint16_t qty, uint8_t *out) {
    if ((uint32_t)addr + qty > 7) return MB_EXC_ILLEGAL_DATA_ADDRESS;  /* match prod: 7 coils */
    memset(out, 0, (qty + 7) / 8);
    for (uint16_t i = 0; i < qty; ++i) {
        if (fake_coils[addr + i]) out[i / 8] |= (uint8_t)(1u << (i % 8));
    }
    return MB_OK;
}
static mb_error_t h_write_single_coil(uint16_t addr, bool v) {
    if (addr >= 7) return MB_EXC_ILLEGAL_DATA_ADDRESS;
    fake_coils[addr] = v ? 1 : 0;
    return MB_OK;
}
static mb_error_t h_write_multiple_coils(uint16_t addr, uint16_t qty, const uint8_t *in) {
    if ((uint32_t)addr + qty > 7) return MB_EXC_ILLEGAL_DATA_ADDRESS;
    for (uint16_t i = 0; i < qty; ++i) {
        fake_coils[addr + i] = (in[i / 8] >> (i % 8)) & 1u;
    }
    return MB_OK;
}
static mb_error_t h_read_di(uint16_t addr, uint16_t qty, uint8_t *out) {
    if ((uint32_t)addr + qty > 12) return MB_EXC_ILLEGAL_DATA_ADDRESS;
    memset(out, 0, (qty + 7) / 8);
    for (uint16_t i = 0; i < qty; ++i) {
        if (fake_inputs[addr + i]) out[i / 8] |= (uint8_t)(1u << (i % 8));
    }
    return MB_OK;
}

static const mb_handlers_t H = {
    .read_coils           = h_read_coils,
    .write_single_coil    = h_write_single_coil,
    .write_multiple_coils = h_write_multiple_coils,
    .read_discrete_inputs = h_read_di,
};

/* ---------- helpers ---------- */
static size_t build(uint8_t *buf, const uint8_t *body, size_t body_len) {
    memcpy(buf, body, body_len);
    uint16_t crc = mb_rtu_crc16(buf, body_len);
    buf[body_len + 0] = (uint8_t)(crc & 0xFF);     /* LSB first */
    buf[body_len + 1] = (uint8_t)(crc >> 8);
    return body_len + 2;
}

/* ---------- tests ---------- */
int main(void) {
    uint8_t req[64], resp[64];
    size_t  req_len, resp_len;

    /* 1. Read coils 0..6, all zero. */
    memset(fake_coils, 0, sizeof(fake_coils));
    req_len = build(req, (uint8_t[]){0x01, 0x01, 0x00, 0x00, 0x00, 0x07}, 6);
    resp_len = sizeof(resp);
    assert(mb_rtu_process(req, req_len, 1, &H, resp, &resp_len));
    /* response = [id=01][fc=01][bytecount=01][data=00][crc lo][crc hi] */
    assert(resp_len == 6);
    assert(resp[0] == 0x01 && resp[1] == 0x01 && resp[2] == 0x01 && resp[3] == 0x00);

    /* 2. Write single coil 0 to ON. */
    req_len = build(req, (uint8_t[]){0x01, 0x05, 0x00, 0x00, 0xFF, 0x00}, 6);
    resp_len = sizeof(resp);
    assert(mb_rtu_process(req, req_len, 1, &H, resp, &resp_len));
    /* echo: 8 bytes */
    assert(resp_len == 8);
    assert(fake_coils[0] == 1);

    /* 3. Read it back. */
    req_len = build(req, (uint8_t[]){0x01, 0x01, 0x00, 0x00, 0x00, 0x01}, 6);
    resp_len = sizeof(resp);
    assert(mb_rtu_process(req, req_len, 1, &H, resp, &resp_len));
    assert(resp[3] == 0x01);  /* bit 0 set */

    /* 4. Write single coil — illegal value (must reject before handler). */
    req_len = build(req, (uint8_t[]){0x01, 0x05, 0x00, 0x00, 0x12, 0x34}, 6);
    resp_len = sizeof(resp);
    assert(mb_rtu_process(req, req_len, 1, &H, resp, &resp_len));
    assert(resp[1] == 0x85);             /* FC | 0x80 */
    assert(resp[2] == MB_EXC_ILLEGAL_DATA_VALUE);

    /* 5. Read coils out of range. */
    req_len = build(req, (uint8_t[]){0x01, 0x01, 0x00, 0x00, 0x00, 0x08}, 6);  /* qty 8 past 7 */
    resp_len = sizeof(resp);
    assert(mb_rtu_process(req, req_len, 1, &H, resp, &resp_len));
    assert(resp[1] == 0x81);
    assert(resp[2] == MB_EXC_ILLEGAL_DATA_ADDRESS);

    /* 6. Read discrete inputs — set FB5, then read all 12. */
    memset(fake_inputs, 0, sizeof(fake_inputs));
    fake_inputs[4] = 1;
    req_len = build(req, (uint8_t[]){0x01, 0x02, 0x00, 0x00, 0x00, 0x0C}, 6);
    resp_len = sizeof(resp);
    assert(mb_rtu_process(req, req_len, 1, &H, resp, &resp_len));
    /* bytecount = 2 (12 bits = 2 bytes). bit 4 set => 0x10 in first byte. */
    assert(resp[2] == 0x02);
    assert(resp[3] == 0x10);
    assert(resp[4] == 0x00);

    /* 7. Write multiple coils — turn on 1, 3, 5. */
    req_len = build(req, (uint8_t[]){0x01, 0x0F, 0x00, 0x00, 0x00, 0x07, 0x01, 0x55}, 8);
    /* bits: 1010101 -> coils 0,2,4,6 = 1 */
    resp_len = sizeof(resp);
    assert(mb_rtu_process(req, req_len, 1, &H, resp, &resp_len));
    /* echo: 8 bytes */
    assert(resp_len == 8);
    assert(fake_coils[0] == 1 && fake_coils[1] == 0 && fake_coils[2] == 1 && fake_coils[4] == 1 && fake_coils[6] == 1);

    /* 8. Unsupported FC -> exception 01. */
    req_len = build(req, (uint8_t[]){0x01, 0x03, 0x00, 0x00, 0x00, 0x01}, 6);
    resp_len = sizeof(resp);
    assert(mb_rtu_process(req, req_len, 1, &H, resp, &resp_len));
    assert(resp[1] == 0x83);
    assert(resp[2] == MB_EXC_ILLEGAL_FUNCTION);

    /* 9. Wrong slave-ID -> silent drop. */
    req_len = build(req, (uint8_t[]){0x07, 0x01, 0x00, 0x00, 0x00, 0x01}, 6);
    resp_len = sizeof(resp);
    assert(!mb_rtu_process(req, req_len, 1, &H, resp, &resp_len));

    /* 10. Bad CRC -> silent drop. */
    req_len = build(req, (uint8_t[]){0x01, 0x01, 0x00, 0x00, 0x00, 0x01}, 6);
    req[req_len - 1] ^= 0xFF;
    resp_len = sizeof(resp);
    assert(!mb_rtu_process(req, req_len, 1, &H, resp, &resp_len));

    /* 11. Broadcast write -> applied, no reply. */
    memset(fake_coils, 0, sizeof(fake_coils));
    req_len = build(req, (uint8_t[]){0x00, 0x05, 0x00, 0x02, 0xFF, 0x00}, 6);
    resp_len = sizeof(resp);
    assert(!mb_rtu_process(req, req_len, 1, &H, resp, &resp_len));
    assert(fake_coils[2] == 1);  /* applied */

    /* 12. Broadcast read -> silent drop, no apply. */
    req_len = build(req, (uint8_t[]){0x00, 0x01, 0x00, 0x00, 0x00, 0x01}, 6);
    resp_len = sizeof(resp);
    assert(!mb_rtu_process(req, req_len, 1, &H, resp, &resp_len));

    /* 13. Frame too short. */
    resp_len = sizeof(resp);
    assert(!mb_rtu_process((uint8_t[]){0x01, 0x01}, 2, 1, &H, resp, &resp_len));

    printf("modbus_rtu: 13 cases OK\n");
    return 0;
}
```

- [ ] **Step 3: Implement `mb_rtu_process` in `modbus_rtu.c`**

Append to (or replace the end of) `Core/Src/app/modbus_rtu.c`:

```c
#include <string.h>

#define FC_READ_COILS              0x01
#define FC_READ_DISCRETE_INPUTS    0x02
#define FC_WRITE_SINGLE_COIL       0x05
#define FC_WRITE_MULTIPLE_COILS    0x0F

#define MIN_FRAME_LEN              4   /* slave + fc + at least 0 bytes + crc(2) */

static size_t build_exception(uint8_t *resp, uint8_t slave_id, uint8_t fc, mb_error_t exc)
{
    resp[0] = slave_id;
    resp[1] = (uint8_t)(fc | 0x80u);
    resp[2] = (uint8_t)exc;
    uint16_t crc = mb_rtu_crc16(resp, 3);
    resp[3] = (uint8_t)(crc & 0xFF);
    resp[4] = (uint8_t)(crc >> 8);
    return 5;
}

static size_t finalize(uint8_t *resp, size_t body_len)
{
    uint16_t crc = mb_rtu_crc16(resp, body_len);
    resp[body_len + 0] = (uint8_t)(crc & 0xFF);
    resp[body_len + 1] = (uint8_t)(crc >> 8);
    return body_len + 2;
}

bool mb_rtu_process(const uint8_t *req, size_t req_len,
                    uint8_t slave_id,
                    const mb_handlers_t *h,
                    uint8_t *resp, size_t *resp_len)
{
    if (req_len < MIN_FRAME_LEN) return false;

    /* CRC check on the whole frame including the trailing 2-byte CRC.
     * A correct frame yields a zero residue. */
    uint16_t calc_crc  = mb_rtu_crc16(req, req_len - 2);
    uint16_t frame_crc = (uint16_t)req[req_len - 2] | ((uint16_t)req[req_len - 1] << 8);
    if (calc_crc != frame_crc) return false;

    uint8_t  addr = req[0];
    uint8_t  fc   = req[1];
    bool     is_broadcast = (addr == 0);

    if (!is_broadcast && addr != slave_id) return false;

    /* ---- dispatch ---- */
    mb_error_t exc = MB_OK;
    size_t     body_len = 0;

    switch (fc) {
        case FC_READ_COILS:
        case FC_READ_DISCRETE_INPUTS: {
            if (is_broadcast) return false;  /* reads are not broadcast */
            if (req_len < 8) { exc = MB_EXC_ILLEGAL_DATA_VALUE; break; }
            uint16_t a = ((uint16_t)req[2] << 8) | req[3];
            uint16_t q = ((uint16_t)req[4] << 8) | req[5];
            if (q == 0 || q > 2000) { exc = MB_EXC_ILLEGAL_DATA_VALUE; break; }

            uint8_t bytes = (uint8_t)((q + 7) / 8);
            resp[0] = slave_id; resp[1] = fc; resp[2] = bytes;
            memset(&resp[3], 0, bytes);
            exc = (fc == FC_READ_COILS)
                ? h->read_coils(a, q, &resp[3])
                : h->read_discrete_inputs(a, q, &resp[3]);
            if (exc == MB_OK) body_len = 3u + bytes;
            break;
        }
        case FC_WRITE_SINGLE_COIL: {
            if (req_len < 8) { exc = MB_EXC_ILLEGAL_DATA_VALUE; break; }
            uint16_t a = ((uint16_t)req[2] << 8) | req[3];
            uint16_t v = ((uint16_t)req[4] << 8) | req[5];
            if (v != 0x0000 && v != 0xFF00) { exc = MB_EXC_ILLEGAL_DATA_VALUE; break; }
            exc = h->write_single_coil(a, v == 0xFF00);
            if (exc == MB_OK) {
                /* echo */
                memcpy(resp, req, 6);
                body_len = 6;
            }
            break;
        }
        case FC_WRITE_MULTIPLE_COILS: {
            if (req_len < 9) { exc = MB_EXC_ILLEGAL_DATA_VALUE; break; }
            uint16_t a  = ((uint16_t)req[2] << 8) | req[3];
            uint16_t q  = ((uint16_t)req[4] << 8) | req[5];
            uint8_t  bc = req[6];
            if (q == 0 || q > 1968 || bc != (q + 7) / 8 || req_len < 9u + bc) {
                exc = MB_EXC_ILLEGAL_DATA_VALUE; break;
            }
            exc = h->write_multiple_coils(a, q, &req[7]);
            if (exc == MB_OK) {
                /* echo addr + qty */
                memcpy(resp, req, 6);
                body_len = 6;
            }
            break;
        }
        default:
            exc = MB_EXC_ILLEGAL_FUNCTION;
            break;
    }

    if (is_broadcast) {
        /* Apply but never reply. */
        return false;
    }
    if (exc != MB_OK) {
        *resp_len = build_exception(resp, slave_id, fc, exc);
        return true;
    }
    *resp_len = finalize(resp, body_len);
    return true;
}
```

- [ ] **Step 4: Wire host test build**

Append to `tests/host/CMakeLists.txt`:

```cmake
add_executable(test_modbus_rtu
    test_modbus_rtu.c
    ../../Core/Src/app/modbus_rtu.c)
target_include_directories(test_modbus_rtu PRIVATE ../../Core/Inc ../../Core/Src/app)
```

- [ ] **Step 5: Run host tests — all 4 must pass**

```powershell
cd tests\host
cmake --build build
.\build\Debug\test_dip_decode.exe
.\build\Debug\test_feedback_debounce.exe
.\build\Debug\test_modbus_crc.exe
.\build\Debug\test_modbus_rtu.exe
```

Expected: each prints its own OK line. If `test_modbus_rtu` fails on a particular case, the assertion line number tells you which behaviour broke — fix the dispatch logic and re-run.

- [ ] **Step 6: Build firmware**

```powershell
cd ..\..
.\build.bat
```

Expected: clean build.

- [ ] **Step 7: Commit**

```powershell
git add Core/Src/app/modbus_rtu.* tests/host/test_modbus_rtu.c tests/host/CMakeLists.txt
git commit -m "feat(modbus): RTU protocol layer with 13-case host test suite"
```

---

## Task 17: Application glue (modbus_app) — FC handlers + transport bridge

**Files:**
- Create: `Core/Src/app/modbus_app.h`
- Create: `Core/Src/app/modbus_app.c`
- Modify: `Makefile`

This module is the only piece of Modbus code that touches hardware modules (`relay`, `feedback`, `mb_uart`). It defines 4 FC handlers that satisfy the `mb_handlers_t` contract from Task 16, and a poll function that drains the UART frame buffer and pumps it through `mb_rtu_process`.

- [ ] **Step 1: `Core/Src/app/modbus_app.h`**

```c
#ifndef APP_MODBUS_APP_H
#define APP_MODBUS_APP_H

#include <stdint.h>

void modbus_app_init(uint8_t slave_id);
void modbus_app_poll(void);

#endif
```

- [ ] **Step 2: `Core/Src/app/modbus_app.c`**

```c
#include "modbus_app.h"
#include "main.h"
#include "mb_uart.h"
#include "modbus_rtu.h"
#include "relay.h"
#include "feedback.h"
#include <string.h>

static uint8_t s_slave_id;
static uint8_t s_resp[APP_MODBUS_FRAME_BUF];

/* ---- FC handlers ---- */
static mb_error_t app_read_coils(uint16_t addr, uint16_t qty, uint8_t *out)
{
    if ((uint32_t)addr + qty > APP_RELAY_COUNT) return MB_EXC_ILLEGAL_DATA_ADDRESS;
    memset(out, 0, (qty + 7u) / 8u);
    for (uint16_t i = 0; i < qty; ++i) {
        if (relay_get((uint8_t)(addr + i))) {
            out[i / 8u] |= (uint8_t)(1u << (i % 8u));
        }
    }
    return MB_OK;
}

static mb_error_t app_write_single_coil(uint16_t addr, bool value)
{
    if (addr >= APP_RELAY_COUNT) return MB_EXC_ILLEGAL_DATA_ADDRESS;
    relay_set((uint8_t)addr, value);
    return MB_OK;
}

static mb_error_t app_write_multiple_coils(uint16_t addr, uint16_t qty, const uint8_t *in)
{
    if ((uint32_t)addr + qty > APP_RELAY_COUNT) return MB_EXC_ILLEGAL_DATA_ADDRESS;
    for (uint16_t i = 0; i < qty; ++i) {
        bool v = ((in[i / 8u] >> (i % 8u)) & 1u) != 0u;
        relay_set((uint8_t)(addr + i), v);
    }
    return MB_OK;
}

static mb_error_t app_read_discrete_inputs(uint16_t addr, uint16_t qty, uint8_t *out)
{
    if ((uint32_t)addr + qty > APP_FEEDBACK_COUNT) return MB_EXC_ILLEGAL_DATA_ADDRESS;
    memset(out, 0, (qty + 7u) / 8u);
    for (uint16_t i = 0; i < qty; ++i) {
        if (feedback_get((uint8_t)(addr + i))) {
            out[i / 8u] |= (uint8_t)(1u << (i % 8u));
        }
    }
    return MB_OK;
}

static const mb_handlers_t s_handlers = {
    .read_coils           = app_read_coils,
    .write_single_coil    = app_write_single_coil,
    .write_multiple_coils = app_write_multiple_coils,
    .read_discrete_inputs = app_read_discrete_inputs,
};

/* ---- public ---- */
void modbus_app_init(uint8_t slave_id) { s_slave_id = slave_id; }

void modbus_app_poll(void)
{
    size_t rx_len;
    if (!mb_uart_rx_ready(&rx_len)) return;

    /* Snapshot incoming frame; release DMA so the next request can arrive
     * while we build the response. */
    uint8_t scratch[APP_MODBUS_FRAME_BUF];
    size_t  n = (rx_len < sizeof(scratch)) ? rx_len : sizeof(scratch);
    memcpy(scratch, mb_uart_rx_buffer(), n);
    mb_uart_rx_release();

    size_t resp_len = sizeof(s_resp);
    if (mb_rtu_process(scratch, n, s_slave_id, &s_handlers, s_resp, &resp_len)) {
        mb_uart_send(s_resp, resp_len);
    }
    /* else: silent drop (broadcast, bad CRC, wrong slave-id) */
}
```

- [ ] **Step 3: Add to Makefile**

In `C_SOURCES`:

```makefile
  Core/Src/app/modbus_app.c \
```

- [ ] **Step 4: Build (don't flash yet — Task 18 wires it into main)**

```powershell
.\build.bat
```

Expected: clean build.

- [ ] **Step 5: Commit**

```powershell
git add Core/Src/app/modbus_app.* Makefile
git commit -m "feat(modbus): app glue (FC handlers + transport bridge)"
```

---

## Task 18: Wire Modbus into main — full super-loop, build only

**Files:**
- Modify: `Core/Src/main.c`

- [ ] **Step 1: Replace `main.c` with the full super-loop**

```c
#include "main.h"
#include "heartbeat.h"
#include "mb_uart.h"
#include "relay.h"
#include "feedback.h"
#include "dip_switch.h"
#include "modbus_app.h"

static void SystemClock_Config(void);
void Error_Handler(void) { __disable_irq(); while (1) {} }

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    /* Outputs FIRST (defaults to OFF) — never let a relay glitch during boot. */
    relay_init();
    relay_apply();

    feedback_init();
    heartbeat_init(HEARTBEAT_NORMAL);
    mb_uart_init();

    uint8_t slave_id = dip_switch_read();
    if (slave_id == 0) {
        /* Config error: ignore Modbus, fast-blink LED forever. */
        heartbeat_set_mode(HEARTBEAT_FAULT);
        while (1) {
            heartbeat_tick();
        }
    }

    modbus_app_init(slave_id);

    while (1) {
        feedback_scan();
        relay_apply();
        modbus_app_poll();
        heartbeat_tick();
    }
}

static void SystemClock_Config(void) { /* keep body from Task 6 */ }
```

- [ ] **Step 2: Build**

```powershell
.\build.bat
```

Expected: clean build. Sizes: flash usage in the tens of KB, RAM well under 30 KB.

- [ ] **Step 3: Commit**

```powershell
git add Core/Src/main.c
git commit -m "feat: complete super-loop with Modbus, relays, feedback, DIP"
```

---

## Task 19: Flash and on-board sanity

**Files:**
- Create: `flash.bat`

- [ ] **Step 1: Write `flash.bat`**

```bat
@echo off
call "%~dp0env.bat"
STM32_Programmer_CLI -c port=SWD freq=4000 -d build\dio_card.bin 0x08000000 -rst
```

- [ ] **Step 2: Flash**

```powershell
.\flash.bat
```

Expected: ends with `File download complete` and `RUNNING Program`.

- [ ] **Step 3: Visual sanity check**

- All 4 DIP switches open → board fast-blinks (5 Hz) on PF3, ignores any Modbus traffic.
- Close DIP1 only → LED slow-blinks (1 Hz); slave should respond at ID 1.
- All relays OFF on boot (no clicks).

**STOP** and confirm with the developer before bench-testing Modbus.

- [ ] **Step 4: Commit**

```powershell
git add flash.bat
git commit -m "build: add flash.bat one-liner"
```

---

## Task 20: Build hygiene pass

**Files:**
- Various (audit only — only fix what's broken)

- [ ] **Step 1: Full clean build with strict flags**

```powershell
.\build.bat clean
.\build.bat
```

Expected: zero warnings, zero errors. Build artifacts: `build/dio_card.elf`, `.bin`, `.hex`, `.map`.

- [ ] **Step 2: Inspect size**

The `size` line should show numbers like:
```
   text    data     bss     dec     hex filename
  ~20000      ~50    ~600  ~20650    ~50aa build/dio_card.elf
```

Anything dramatically larger (e.g. 100+ KB Flash) → grep for accidentally-included HAL modules in `Makefile`, or for missing `--gc-sections` in LDFLAGS.

- [ ] **Step 3: Verify no file under `Core/Src/app/` exceeds 200 lines**

```powershell
for /f %F in ('dir /b Core\Src\app\*.c Core\Src\app\*.h') do @echo %F: && find /c /v "" "Core\Src\app\%F"
```

If any file is over 200 lines, factor it. Likely candidate: `mb_uart.c` — split TX/RX into separate files if needed.

- [ ] **Step 4: Commit (only if anything changed)**

```powershell
git add -A
git commit -m "chore: build hygiene pass (warnings clean, size verified)"
```

---

## Task 21: Bench Modbus master tests

**Files:**
- Create: `tests/mbpoll_smoke.md`

The board is now ready. We exercise the Modbus surface from a PC-side master. Install `mbpoll` (https://github.com/epsilonrt/mbpoll/releases — Windows build) or use the QModMaster GUI.

Assume the board's slave ID is set to `1` (close DIP1 only) and the USB-RS485 adapter enumerates as `COM5`. Replace `COM5` with the actual port on your machine.

- [ ] **Step 1: Write `tests/mbpoll_smoke.md`**

```markdown
# DIO Card — Modbus RTU smoke tests

Setup:
- DIO Card slave ID = 1 (DIP1 closed, others open)
- USB-RS485 adapter on `COM5`
- 9600 8N1, no RTS/CTS

## Coil tests (FC 1, 5, 15)

1. Read 7 coils starting at 0 (expect all zeros on boot):
   `mbpoll -m rtu -a 1 -b 9600 -t 0 -r 1 -c 7 -P none COM5`
   → Expected: `[0] = 0, [1] = 0, ... [6] = 0`

2. Energise relay 1 (audible click):
   `mbpoll -m rtu -a 1 -b 9600 -t 0 -r 1 -P none COM5 1`
   → Expected: success; relay 1 closes.

3. Read back coil 0:
   `mbpoll -m rtu -a 1 -b 9600 -t 0 -r 1 -c 1 -P none COM5`
   → Expected: `[0] = 1`

4. De-energise relay 1:
   `mbpoll -m rtu -a 1 -b 9600 -t 0 -r 1 -P none COM5 0`

5. Write multiple coils (FC 15) — turn on R1, R3, R5:
   `mbpoll -m rtu -a 1 -b 9600 -t 0 -r 1 -P none COM5 1 0 1 0 1 0 0`
   → Expected: relays 1, 3, 5 close audibly.

6. Out-of-range write (FC 5 to coil 7 — only 0..6 are valid):
   `mbpoll -m rtu -a 1 -b 9600 -t 0 -r 8 -P none COM5 1`
   → Expected: exception 02 (Illegal Data Address).

## Discrete input tests (FC 2)

7. Read all 12 feedbacks:
   `mbpoll -m rtu -a 1 -b 9600 -t 1 -r 1 -c 12 -P none COM5`
   → Expected: all zeros if no contactor wired; `[0..11] = 0`.

8. Short FB1 (PD0) to GND, re-read:
   → Expected: `[0] = 1`, others 0.

9. Out-of-range read (FC 2 at address 12):
   `mbpoll -m rtu -a 1 -b 9600 -t 1 -r 13 -c 1 -P none COM5`
   → Expected: exception 02.

## Negative tests

10. Wrong slave ID:
    `mbpoll -m rtu -a 7 -b 9600 -t 0 -r 1 -P none COM5 1`
    → Expected: timeout (no response).

11. Unsupported FC (FC 3 Read Holding Registers):
    `mbpoll -m rtu -a 1 -b 9600 -t 4 -r 1 -c 1 -P none COM5`
    → Expected: exception 01 (Illegal Function).

## Stability

12. Loop FC 2 read for 60 seconds:
    `mbpoll -m rtu -a 1 -b 9600 -t 1 -r 1 -c 12 -1 -l 1000 -P none COM5` (with `-1` for "loop forever", break after 60 s)
    → Expected: no missed responses, LED keeps a steady 1 Hz blink.
```

- [ ] **Step 2: Run each test, mark results**

Walk through all 12. If any fails, debug — most likely culprits:
- Slave ID mismatch (re-check DIP wiring at Task 9 polarity)
- DE/RE timing (scope PB2 vs. TX byte stream — DE should drop within ~1 bit time after the last stop bit)
- `modbus_rtu` packing/endianness bug (re-run host tests after every edit; they cover all 4 FCs + exceptions + broadcast + bad-CRC)
- USART AF macro wrong (PB10/PB11 not actually USART3)

- [ ] **Step 3: Commit the test doc**

```powershell
git add tests/mbpoll_smoke.md
git commit -m "test: bench smoke tests for Modbus RTU surface"
```

---

## Task 22: README

**Files:**
- Create: `README.md`

- [ ] **Step 1: Write `README.md`**

```markdown
# Quench DIO Card — Modbus RTU Slave Firmware

STM32C092CBTX firmware that exposes 7 relays and 12 contactor-feedback
inputs to a Modbus RTU master over RS-485.

## Quick start

1. Clone with submodules:
   ```
   git clone --recursive <repo-url>
   ```
2. Build:
   ```
   .\build.bat
   ```
3. Flash (ST-LINK over SWD must be wired):
   ```
   .\flash.bat
   ```

## Pin map

See `docs/superpowers/specs/2026-06-02-dio-card-modbus-slave-design.md` §3.

| Resource | MCU pin |
|---|---|
| Relays 1-7 | PA15, PA12, PA11, PA10, PC7, PC6, PA9 (all active-low) |
| Feedbacks 1-12 | PD0-3, PB3-9, PC13 (active-low w/ pull-up) |
| DIP1-4 (slave ID) | PA8, PB15, PB14, PB13 (DIP1 = LSB) |
| USART3 TX / RX / DE | PB10 / PB11 / PB2 |
| Heartbeat LED | PF3 |

## Modbus map

| FC | Range | Maps to |
|---|---|---|
| 0x01 Read Coils | 0..6 | Relays 1-7 |
| 0x05 Write Single Coil | 0..6 | Relay 1-7 |
| 0x0F Write Multiple Coils | 0..6 | Relays 1-7 |
| 0x02 Read Discrete Inputs | 0..11 | Feedbacks 1-12 |

Out-of-range → exception 0x02. Unsupported FC → exception 0x01.

Serial: 9600 8N1.

## DIP → slave ID

DIP1 = LSB (weight 1), DIP4 = MSB (weight 8). All switches open
(slave_id = 0) puts the board in **config error** state — it ignores
Modbus traffic and fast-blinks the heartbeat LED at 5 Hz.

## Toolchain

Built and flashed using STM32CubeIDE 1.19.0's bundled `arm-none-eabi-gcc`,
`make`, and `STM32_Programmer_CLI`. See `env.bat` for absolute paths.

## Testing

- **Host-side unit tests** for pure-logic helpers: `tests/host/` (see CMakeLists).
- **Bench smoke tests** against a PC Modbus master: `tests/mbpoll_smoke.md`.
```

- [ ] **Step 2: Commit**

```powershell
git add README.md
git commit -m "docs: README with quick-start, pin map, Modbus map"
```

---

## Task 23: Final verification — all acceptance criteria

- [ ] **Step 1: Re-run clean build + flash**

```powershell
.\build.bat clean
.\build.bat
.\flash.bat
```

Expected: zero warnings, flash success.

- [ ] **Step 2: Re-walk the spec §10 acceptance criteria**

For each item, check:

1. `make -j` builds cleanly under `-Wall -Wextra -Werror`, no warnings. → confirmed by Step 1.
2. `STM32_Programmer_CLI` flashes and resets successfully. → confirmed by Step 1.
3. Heartbeat LED on PF3 blinks at 1 Hz (or 5 Hz if DIP=0). → verify visually.
4. With DIP=N (1-15), board answers Modbus frames at slave ID N. → confirmed by Task 19 bench tests.
5. No file under `Core/Src/app/` exceeds 200 lines. → confirmed in Task 18.
6. README documents pin map, Modbus map, build/flash, DIP-to-ID. → confirmed in Task 20.
7. Spec, code, and inputs committed to git with meaningful history. → `git log --oneline` should show ~20 commits, one per task.

- [ ] **Step 3: Final commit (if anything was missed)**

```powershell
git add -A
git status   # should report "nothing to commit, working tree clean"
git log --oneline
```

- [ ] **Step 4: Hand back to the user**

Print a summary: total commits, flash/RAM usage, list of bench tests that passed, any deviations from the spec. Wait for user confirmation that the work is done.

---

## Self-Review Notes (for the plan author)

**Spec coverage map:**
- Spec §1 purpose → Tasks 15-18
- Spec §2 hardware target → Tasks 3-6
- Spec §3.1 relays → Task 10
- Spec §3.2 feedbacks → Task 11
- Spec §3.3 DIP switch → Task 9
- Spec §3.4 USART/DE-RE → Tasks 12-14
- Spec §3.5 heartbeat LED → Task 7
- Spec §3.6 unused pins → handled implicitly (no init code = HAL default analog) — not a separate task
- Spec §4 Modbus protocol (FCs, exceptions, slave ID, broadcast, CRC) → Tasks 15, 16, 17
- Spec §5 architecture (incl. modbus_rtu vs modbus_app split) → Tasks 5, 15-18
- Spec §6 layout → Tasks 1, 5, ongoing
- Spec §7 build/flash → Tasks 5, 19, 23
- Spec §8 test strategy → Tasks 9, 11, 15, 16 (host), 21 (bench), 19 (LED sanity)
- Spec §9 risks → DE/RE timing handled in Task 14 IRQ; hand-rolled-bugs risk mitigated by 13-case host test in Task 16
- Spec §10 acceptance criteria → Task 23
- Spec §11 end-to-end command sequence → Tasks 2, 5, 19

**Known imprecisions to verify during execution:**
- `PIN_UART_AF` value (Task 8) — confirm against STM32C092 datasheet/header before Task 12.
- IRQ handler name `USART3_LPUART1_IRQHandler` (Task 13) — confirm against the actual startup file symbol.
- `PLLR` field on STM32C0 PLL (Task 6) — confirm the field exists in the HAL header; some C0 PLLs lack it.
- CRC vectors hand-verified during plan review: `0x01 0x02` → 0xE181, `0x01 0x03 0x00 0x00 0x00 0x0A` → 0xCDC5 (LSB-first on the wire).

These are not blockers — each is a 30-second verification when the relevant task is reached.

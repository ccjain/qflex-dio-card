# DIO Card - STM32C092CBTX Modbus RTU Slave firmware
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
  Core/Src/app/heartbeat.c \
  Core/Src/app/dip_switch.c \
  Core/Src/app/relay.c \
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
	@mkdir -p $(BUILD_DIR)

clean:
	@rm -rf $(BUILD_DIR)

-include $(OBJS:.o=.d)

.PHONY: all clean

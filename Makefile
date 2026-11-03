# Command line build (arm-none-eabi-gcc). STM32CubeIDE uses .cproject.
#   make            -> build/stm32f030f4_dali.elf / .hex / .bin + size report
#   make clean

TARGET   := stm32f030f4_dali
BUILD    := build
PREFIX   ?= arm-none-eabi-
CC       := $(PREFIX)gcc
OBJCOPY  := $(PREFIX)objcopy
SIZE     := $(PREFIX)size

SRCS := $(wildcard Core/Src/*.c) $(wildcard App/Src/*.c)
ASMS := Core/Startup/startup_stm32f030f4px.s

INCS := -ICore/Inc -IApp/Inc \
        -IDrivers/CMSIS/Device/ST/STM32F0xx/Include -IDrivers/CMSIS/Include

MCU    := -mcpu=cortex-m0 -mthumb
DEFS   := -DSTM32F030x6
CFLAGS := $(MCU) $(DEFS) $(INCS) -std=c99 -Os -g3 \
          -ffunction-sections -fdata-sections -fno-common \
          -Wall -Wextra -Wconversion -Wshadow -Wundef -Wstrict-prototypes
LDFLAGS := $(MCU) -TSTM32F030F4PX_FLASH.ld --specs=nano.specs \
           -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(TARGET).map -Wl,--print-memory-usage

OBJS := $(addprefix $(BUILD)/,$(SRCS:.c=.o)) $(addprefix $(BUILD)/,$(ASMS:.s=.o))

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).hex $(BUILD)/$(TARGET).bin
	$(SIZE) $(BUILD)/$(TARGET).elf

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: %.s
	@mkdir -p $(dir $@)
	$(CC) $(MCU) -c $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $@

$(BUILD)/$(TARGET).hex: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

clean:
	rm -rf $(BUILD)

.PHONY: all clean

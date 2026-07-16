
ROOT := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))
UNAME_S := $(shell uname -s)

ifeq ($(UNAME_S),Darwin)
ARM_SDK_PREFIX:=$(ROOT)/../tools/macos/xpack-arm-none-eabi-gcc-10.3.1-2.3/bin/arm-none-eabi-
else
ARM_SDK_PREFIX:=arm-none-eabi-
endif

CC = $(ARM_SDK_PREFIX)gcc
OBJCOPY = $(ARM_SDK_PREFIX)objcopy
SIZE = $(ARM_SDK_PREFIX)size

TARGET = test_pb2
BUILD_DIR = $(ROOT)/build

CFLAGS = -mcpu=cortex-m4 -mthumb -O3 -ffunction-sections -fdata-sections -Wall -DAT32F421C8T7
CFLAGS += -I.

LDFLAGS = -mcpu=cortex-m4 -mthumb -T$(ROOT)/link.ld -Wl,--gc-sections -specs=nano.specs -specs=nosys.specs

SRC = \
	$(ROOT)/main.c \
	$(ROOT)/startup.s

OBJ = $(BUILD_DIR)/main.o \
	$(BUILD_DIR)/startup.o

.PHONY: all clean

all: $(BUILD_DIR)/$(TARGET).bin $(BUILD_DIR)/$(TARGET).hex

$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/main.o: $(ROOT)/main.c | $(BUILD_DIR)
	@echo Compiling main.c
	@$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/startup.o: $(ROOT)/startup.s | $(BUILD_DIR)
	@echo Compiling startup.s
	@$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJ)
	@echo Linking $@
	@$(CC) $(LDFLAGS) $(OBJ) -o $@
	@$(SIZE) $@

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	@echo Generating $@
	@$(OBJCOPY) -O binary $< $@

$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	@echo Generating $@
	@$(OBJCOPY) -O ihex $< $@

clean:
	@rm -rf $(BUILD_DIR)


TARGET := drone_stm32
SRC_DIR := src
BUILD_DIR := build

CC := arm-none-eabi-gcc

CFLAGS := -mcpu=cortex-m3 -mthumb -O2 -g -Wall -Wextra -mno-unaligned-access -fno-builtin
CFLAGS += -DSTM32F103C8 -DSTM32F10X_MD
CFLAGS += -I$(SRC_DIR)
CFLAGS += -ffunction-sections -fdata-sections

LDFLAGS := -T $(SRC_DIR)/stm32f103c8.ld -nostdlib -lgcc -Wl,--gc-sections

SOURCES := $(wildcard $(SRC_DIR)/*.c)
OBJECTS := $(patsubst $(SRC_DIR)/%.c,$(BUILD_DIR)/%.o,$(SOURCES))

.PHONY: all clean flash

all: $(TARGET).elf $(TARGET).bin

$(TARGET).elf: $(OBJECTS)
	$(CC) -o $@ $^ $(LDFLAGS)

$(TARGET).bin: $(TARGET).elf
	arm-none-eabi-objcopy -O binary $< $@

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR):
	mkdir -p $@

flash: $(TARGET).bin
	st-flash write $< 0x08000000

clean:
	rm -rf $(BUILD_DIR) $(TARGET).elf $(TARGET).bin

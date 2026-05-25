TARGET := milkv_drone
MPU_TEST_TARGET := mpu_test
SRC_DIR := src
BUILD_DIR := build/duo-riscv64
USE_RT ?= 0

ifeq ($(strip $(TOOLCHAIN_PREFIX)),)
$(error TOOLCHAIN_PREFIX is not set. Run "source /path/to/duo-examples/envsetup.sh" and select Product=2, Arch=2)
endif

ifeq ($(strip $(CFLAGS)),)
$(error CFLAGS is not set. Run Milk-V duo-examples/envsetup.sh before make)
endif

ifeq ($(strip $(LDFLAGS)),)
$(error LDFLAGS is not set. Run Milk-V duo-examples/envsetup.sh before make)
endif

ifneq ($(strip $(CHIP)),CV181X)
$(error CHIP must be CV181X for Milk-V Duo 256M / SG2002. Re-run envsetup.sh and select Product=2)
endif

ifneq (,$(findstring riscv64-linux-gnu,$(TOOLCHAIN_PREFIX)))
$(error Do not use Ubuntu riscv64-linux-gnu toolchain. Run duo-examples/envsetup.sh and select Product=2, Arch=2)
endif

ifeq (,$(findstring riscv64-unknown-linux-musl-,$(TOOLCHAIN_PREFIX)))
$(error TOOLCHAIN_PREFIX must point to Milk-V RISCV64 musl toolchain from duo-examples; got "$(TOOLCHAIN_PREFIX)")
endif

override CC := $(TOOLCHAIN_PREFIX)gcc
override CXX := $(TOOLCHAIN_PREFIX)g++

SOURCES := $(sort $(wildcard $(SRC_DIR)/*.cpp))
MAIN_SOURCES := $(filter-out $(SRC_DIR)/mpu_test.cpp,$(SOURCES))
MPU_TEST_SOURCES := $(SRC_DIR)/mpu_test.cpp $(SRC_DIR)/imu_mpu6050.cpp
OBJECTS := $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/%.o,$(MAIN_SOURCES))
MPU_TEST_OBJECTS := $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/%.o,$(MPU_TEST_SOURCES))
DEPENDS := $(sort $(OBJECTS:.o=.d) $(MPU_TEST_OBJECTS:.o=.d))
THREAD_FLAGS := -pthread

CPPFLAGS += -I$(SRC_DIR)
CXXFLAGS += $(CFLAGS) $(THREAD_FLAGS) -std=c++17 -Wall -Wextra -Wpedantic -MMD -MP
LDLIBS += $(THREAD_FLAGS)

ifeq ($(USE_RT),1)
LDLIBS += -lrt
endif

.PHONY: all clean print-toolchain

all: $(TARGET) $(MPU_TEST_TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) -o $@ $^ $(LDFLAGS) $(LDLIBS)

$(MPU_TEST_TARGET): $(MPU_TEST_OBJECTS)
	$(CXX) -o $@ $^ $(LDFLAGS) $(LDLIBS)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR):
	mkdir -p $@

print-toolchain:
	@echo "CHIP=$(CHIP)"
	@echo "TOOLCHAIN_PREFIX=$(TOOLCHAIN_PREFIX)"
	@echo "CXX=$(CXX)"
	@echo "CFLAGS=$(CFLAGS)"
	@echo "LDFLAGS=$(LDFLAGS)"

clean:
	rm -rf $(BUILD_DIR) $(TARGET) $(MPU_TEST_TARGET)

-include $(DEPENDS)

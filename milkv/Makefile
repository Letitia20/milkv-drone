TARGET := milkv_drone
MPU_TEST_TARGET := mpu_test
IBUS_READ_TARGET := ibus_read
SRC_DIR := src
BUILD_DIR := build/duo-riscv64
HOST_BUILD_DIR := build/host-tests
HOST_CXX ?= g++
HOST_AUTOSTART_TEST := $(HOST_BUILD_DIR)/autostart_script_test.exe
USE_RT ?= 0

ifneq ($(filter host-test,$(MAKECMDGOALS)),host-test)
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
endif

override CC := $(TOOLCHAIN_PREFIX)gcc
override CXX := $(TOOLCHAIN_PREFIX)g++

SOURCES := $(sort $(wildcard $(SRC_DIR)/*.cpp))
STANDALONE_SOURCES := $(SRC_DIR)/mpu_test.cpp $(SRC_DIR)/ibus_read.cpp $(SRC_DIR)/ibus_reader.cpp
MAIN_SOURCES := $(filter-out $(STANDALONE_SOURCES),$(SOURCES))
MPU_TEST_SOURCES := $(SRC_DIR)/mpu_test.cpp $(SRC_DIR)/imu_mpu6050.cpp
IBUS_READ_SOURCES := $(SRC_DIR)/ibus_read.cpp $(SRC_DIR)/ibus_receiver.cpp
OBJECTS := $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/%.o,$(MAIN_SOURCES))
MPU_TEST_OBJECTS := $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/%.o,$(MPU_TEST_SOURCES))
IBUS_READ_OBJECTS := $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/%.o,$(IBUS_READ_SOURCES))
DEPENDS := $(sort $(OBJECTS:.o=.d) $(MPU_TEST_OBJECTS:.o=.d) $(IBUS_READ_OBJECTS:.o=.d))
THREAD_FLAGS := -pthread

CPPFLAGS += -I$(SRC_DIR)
CXXFLAGS += $(CFLAGS) $(THREAD_FLAGS) -std=c++17 -Wall -Wextra -Wpedantic -MMD -MP
LDLIBS += $(THREAD_FLAGS)

ifeq ($(USE_RT),1)
LDLIBS += -lrt
endif

.PHONY: all clean print-toolchain host-test

all: $(TARGET) $(MPU_TEST_TARGET) $(IBUS_READ_TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) -o $@ $^ $(LDFLAGS) $(LDLIBS)

$(MPU_TEST_TARGET): $(MPU_TEST_OBJECTS)
	$(CXX) -o $@ $^ $(LDFLAGS) $(LDLIBS)

$(IBUS_READ_TARGET): $(IBUS_READ_OBJECTS)
	$(CXX) -o $@ $^ $(LDFLAGS) $(LDLIBS)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR):
	mkdir -p $@

host-test: $(HOST_AUTOSTART_TEST)
	./$(HOST_AUTOSTART_TEST)

$(HOST_AUTOSTART_TEST): tests/autostart_script_test.cpp | $(HOST_BUILD_DIR)
	$(HOST_CXX) -std=c++17 -Wall -Wextra -Wpedantic -I $(SRC_DIR) $< -o $@

$(HOST_BUILD_DIR):
	mkdir -p $@

print-toolchain:
	@echo "CHIP=$(CHIP)"
	@echo "TOOLCHAIN_PREFIX=$(TOOLCHAIN_PREFIX)"
	@echo "CXX=$(CXX)"
	@echo "CFLAGS=$(CFLAGS)"
	@echo "LDFLAGS=$(LDFLAGS)"

clean:
	rm -rf $(BUILD_DIR) $(TARGET) $(MPU_TEST_TARGET) $(IBUS_READ_TARGET)

-include $(DEPENDS)

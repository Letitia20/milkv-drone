# Milk-V Only Autostart Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the Milk-V drone controller deployable so the board can boot directly into remote-control flight readiness without a USB data-cable command session.

**Architecture:** Keep `milkv_drone` as the single runtime on Milk-V. The process reads iBUS from the configured UART, drives ESC PWM directly through sysfs, and keeps the existing arm, throttle-low, RC timeout, IMU, and low-voltage safety checks. Deployment installs `/mnt/system/auto.sh` so Milk-V starts the controller at boot and logs output for troubleshooting.

**Tech Stack:** POSIX shell for Milk-V installation, GNU Make for host-side convenience targets, C++17 controller code and existing lightweight C++ tests.

---

### Task 1: Startup Script Static Test

**Files:**
- Create: `milkv/tests/autostart_script_test.cpp`
- Modify: `milkv/Makefile`
- Create: `milkv/install_autostart.sh`

- [ ] **Step 1: Write the failing test**

Create `milkv/tests/autostart_script_test.cpp` that opens `install_autostart.sh` and checks for:

```cpp
expectContains(script, "AUTO_SH=\"/mnt/system/auto.sh\"", "targets Milk-V startup script");
expectContains(script, "APP=\"/root/milkv_drone\"", "runs installed controller binary");
expectContains(script, "IBUS_DEVICE=\"${IBUS_DEVICE:-/dev/ttyS1}\"", "defaults to iBUS UART");
expectContains(script, "BAUDRATE=\"${BAUDRATE:-115200}\"", "defaults to 115200 baud");
expectContains(script, "\"${APP}\" \"${IBUS_DEVICE}\" \"${BAUDRATE}\"", "starts controller with device and baud");
expectContains(script, "chmod +x \"${AUTO_SH}\"", "marks auto.sh executable");
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `g++ -std=c++17 -Wall -Wextra -Wpedantic -I src tests/autostart_script_test.cpp -o build/host-tests/autostart_script_test.exe && ./build/host-tests/autostart_script_test.exe`

Expected: fail because `install_autostart.sh` does not exist or does not contain the required startup content.

- [ ] **Step 3: Implement the minimal script**

Create `milkv/install_autostart.sh` with defaults for `/root/milkv_drone`, `/dev/ttyS1`, `115200`, optional `PWM_CHIP`, and a generated `/mnt/system/auto.sh` that starts the controller once at boot and writes logs to `/root/milkv_drone.log`.

- [ ] **Step 4: Wire the test into Makefile**

Add host-test variables and a `host-test` target that builds and runs the new script test without requiring the Milk-V cross toolchain.

- [ ] **Step 5: Run the test to verify it passes**

Run: `make host-test`

Expected: pass with `autostart_script_test passed`.

### Task 2: Milk-V-only Documentation

**Files:**
- Modify: `milkv/README.md`

- [ ] **Step 1: Update architecture text**

Change the architecture to state that Milk-V is the active controller for the current setup.

- [ ] **Step 2: Update upload and autostart instructions**

Document:

```sh
scp milkv_drone install_autostart.sh root@192.168.42.1:/root/
ssh root@192.168.42.1
sh /root/install_autostart.sh
reboot
```

- [ ] **Step 3: Preserve safety constraints**

Keep the no-propeller bench-test warnings and list the runtime arm conditions: valid RC, arm switch high, throttle low before arming, IMU valid, no low-voltage latch, and no failsafe.

- [ ] **Step 4: Verify text references**

Run: `Select-String -Path D:\drone\milkv\README.md -Pattern 'MOT,1000|UART Protocol'`

Expected: no stale text from the old UART motor-command path.

### Task 3: Full Verification

**Files:**
- Read: `git diff --check`
- Read: `git diff -- milkv/install_autostart.sh milkv/Makefile milkv/README.md milkv/tests/autostart_script_test.cpp`

- [ ] **Step 1: Run whitespace check**

Run: `git diff --check`

Expected: exit 0.

- [ ] **Step 2: Run host test**

Run: `make host-test`

Expected: exit 0 and `autostart_script_test passed`.

- [ ] **Step 3: Review final diff**

Run: `git diff -- milkv/install_autostart.sh milkv/Makefile milkv/README.md milkv/tests/autostart_script_test.cpp`

Expected: changes are limited to the autostart deployment path, Makefile host test, and Milk-V-only documentation.

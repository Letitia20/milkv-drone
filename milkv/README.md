# milkv_drone

Milk-V Duo 256M / SG2002 based drone controller project.

This repository contains the Milk-V side C++17 controller. The current runtime
architecture is Milk-V-only:

```text
Milk-V Duo 256M: controller, iBUS receiver input, IMU, filter, PID, motor mixing,
                 ESC PWM output, telemetry log, startup script
```

The previous STM32 coprocessor path is not required for this setup. Because
Linux is not a hard-realtime flight controller, keep early tests conservative:
remove propellers for software checks, keep the bench PWM cap in place, and use
a physical power cutoff during bring-up.

## Current Status

Implemented on the Milk-V side:

- C++17 codebase with multiple `src/*.cpp` and `src/*.hpp` files.
- 100 Hz main loop.
- Direct iBUS receiver input on `/dev/ttyS1`.
- Direct sysfs PWM output for four ESCs.
- RC parsing, arm switch handling, throttle-low arming gate, and failsafe logic.
- MPU6050 Linux I2C interface on `/dev/i2c-2` with I2C address support for
  `0x68` and `0x69`; `WHO_AM_I` accepts `0x68` or observed ID value `0x70`.
- Complementary filter attitude estimation.
- Gyro bias calibration on startup: 200 samples, 10 ms interval.
- RC stick deadband and softened throttle curve.
- RC roll/pitch/yaw commands feed the three-axis PID control path.
- 10 Hz attitude and IMU telemetry log output.
- PID class.
- X-layout quad motor mixer.
- Safety state: RC timeout, failsafe flag, low voltage latch, invalid IMU state,
  disarm switch, throttle-low arm gate, and safe-stop on exit.
- Milk-V boot autostart installer for `/mnt/system/auto.sh`.
- Makefile build using the official Milk-V `duo-examples/envsetup.sh`
  environment.

Current safety behavior:

- ESC output stays at `1000us` unless RC is fresh, failsafe is clear, IMU is
  valid, voltage safety has not latched, SWA/CH5 is in the normal position,
  CH6 has been cycled through disarm, and throttle was low before arming.
- Motor output is capped at `1800us`.
- Startup only makes the controller ready for remote arming; it does not
  automatically arm or take off.

## Hardware Architecture

Milk-V-only system wiring:

```text
3S battery XT60 -> PDB
PDB -> four ESC power inputs
PDB or external 5V BEC -> IA6B receiver
External 5V BEC -> Milk-V Duo 256M + camera + USB WiFi
IA6B iBUS -> Milk-V GP3 / UART1_RX
Milk-V PWM pins -> four ESC signal pins
MPU6050 / BMP280 / OLED -> Milk-V I2C
Optional battery divider -> verified Milk-V ADC-capable input
All signal grounds -> common ground
```

Important safety rules:

- Remove propellers during all software, UART, PWM, and bench tests.
- Do not install propellers until boot autostart, RC failsafe, arm/disarm, and
  low-throttle behavior have all been checked repeatedly.
- Keep a physical battery disconnect available during every powered test.
- Do not connect battery voltage directly to ADC pins. Use a verified resistor
  divider and common ground.
- Do not power Milk-V, camera, and WiFi from an overloaded PDB 5V rail. Use an
  external BEC with enough current margin.
- Do not mount bare boards directly on carbon fiber without insulation.

## Pinmux Notes

The `xmdjy/milkv-car` project is useful as a reference for Milk-V project
organization, `duo-pinmux`, wiringX, I2C, PWM examples, `scp` upload, autostart,
and PID structure.

For this drone project, only borrow the Milk-V engineering patterns. Do not copy
the car project's DC motor PWM control as ESC control.

Final MPU6050 wiring for this project:

```text
MPU6050 VCC -> Milk-V 3.3V
MPU6050 GND -> Milk-V GND
MPU6050 SDA -> Milk-V GP10 / IIC2_SDA
MPU6050 SCL -> Milk-V GP11 / IIC2_SCL
MPU6050 AD0 -> floating
MPU6050 INT -> floating
```

Programs use the I2C device path:

```text
/dev/i2c-2
```

The MPU6050 I2C address is `0x68` by default. Some modules report `WHO_AM_I`
as `0x70`; that is handled inside the driver and should not be passed as the
I2C address:

```sh
/root/mpu_test /dev/i2c-2 0x68
```

Before wiring, verify pin functions on the board:

```sh
duo-pinmux -p
duo-pinmux -l
duo-pinmux -r GP10
duo-pinmux -r GP11
```

Do not assign the same Milk-V pins to both UART and I2C. The current hardware
pin assignment is intentionally left for board-side confirmation with
`duo-pinmux -l` and `duo-pinmux -r`.

Example I2C pinmux commands from the car project style:

```sh
duo-pinmux -w GP10/IIC2_SDA
duo-pinmux -w GP11/IIC2_SCL
```

Only use those commands after confirming the names with `duo-pinmux -l` on the
actual board image.

## RC Input And Safety

PWM values are clamped to:

```text
minimum: 1000us
middle:  1500us
maximum: 2000us
```

The current Milk-V-only program treats channel 5 as the SWA emergency stop:

```text
SWA normal:       ch5 ~= 1000
SWA emergency:    ch5 ~= 2000
emergency stop:   ch5 > 1500
arm request:      ch6 > 1500 after ch6 has been low
disarm request:   ch6 <= 1500
throttle low:     ch3 <= 1100
RC timeout:       200 ms
low voltage:      < 9600 mV
```

## Build

Use the official Milk-V `duo-examples` environment. Do not use Ubuntu's default
`riscv64-linux-gnu-g++`, and do not build a Windows `.exe`.

```sh
git clone https://github.com/milkv-duo/duo-examples.git
cd duo-examples
source envsetup.sh
```

Select:

```text
Product: 2
Arch:    2
```

Then build this project:

```sh
cd /path/to/milkv
make
```

Optional diagnostics:

```sh
make print-toolchain
```

Clean:

```sh
make clean
```

If a future dependency needs `librt`, build with:

```sh
make USE_RT=1
```

The Makefile checks that:

- `TOOLCHAIN_PREFIX`, `CFLAGS`, and `LDFLAGS` came from `envsetup.sh`.
- `CHIP=CV181X`, matching Milk-V Duo 256M / SG2002.
- the toolchain prefix is the official RISCV64 musl toolchain style,
  `riscv64-unknown-linux-musl-`.
- Ubuntu `riscv64-linux-gnu-*` is rejected.

## Upload and Run

Upload to Milk-V over USB Ethernet:

```sh
scp milkv_drone mpu_test setup_esc_pwm.sh install_autostart.sh root@192.168.42.1:/root/
```

Log in and run:

```sh
ssh root@192.168.42.1
chmod +x /root/milkv_drone
/root/milkv_drone [ibus_device] [baudrate]
```

Example:

```sh
/root/milkv_drone /dev/ttyS1 115200
```

Current `milkv_drone` IMU behavior:

```text
- tries /dev/i2c-2 at I2C address 0x68 first, then falls back to allowed auto-detect
- accepts WHO_AM_I values 0x68 and 0x70
- prints "Keep the board still, calibrating gyro..."
- samples gyro 200 times at 10 ms intervals
- prints calibrated gx/gy/gz bias
- runs the complementary filter at 100 Hz with real dt_s
- prints attitude + raw accel/gyro log at 10 Hz
- writes ESC PWM directly and keeps outputs at 1000us until all arm conditions
  are true
- logs `emergency_stop=0/1` and `swa_us=<ch5>` in heartbeat and attitude lines
- forces all ESC outputs to 1000us with reason `emergency_stop` whenever
  SWA/CH5 is above 1500us
```

Before connecting PID or motor output, run the standalone MPU6050 bring-up test:

```sh
chmod +x /root/mpu_test
/root/mpu_test /dev/i2c-2 0x68
```

It prints only:

```text
ax ay az gx gy gz
```

Keep the board flat first and confirm `az` shows gravity, then rotate the board
and confirm `ax`, `ay`, `az`, `gx`, `gy`, and `gz` change.

If you use the default Milk-V image login, the password is commonly:

```text
milkv
```

## Autostart

After bench testing without propellers, install Milk-V boot autostart:

```sh
chmod +x /root/install_autostart.sh
sh /root/install_autostart.sh
reboot
```

Optional overrides at install time:

```sh
IBUS_DEVICE=/dev/ttyS1 BAUDRATE=115200 PWM_CHIP=pwmchip4 sh /root/install_autostart.sh
```

The installer writes `/mnt/system/auto.sh`. On each boot, it runs
`/root/setup_esc_pwm.sh` when present, starts `/root/milkv_drone`, and appends
logs to `/root/milkv_drone.log`.

## Bench Test Checklist

Run these tests before enabling real motor output:

```text
1. Milk-V boots and the program prints a 100 Hz startup message.
2. Program exits cleanly on Ctrl-C and sends several safe-stop commands.
3. ESC PWM stays at `1000us` while disarmed.
4. RC channel values update once the IA6B receiver is active.
5. Pulling the receiver signal triggers failsafe and forces all ESC outputs to `1000us`.
6. SWA/CH5 at about 2000us triggers emergency stop and forces all ESC outputs to `1000us`.
7. Returning SWA/CH5 to about 1000us does not re-arm until the normal arm flow is repeated.
8. CH6 arms only after it has been low, throttle is low, and all safety conditions are true.
9. Low voltage telemetry latches disarm when battery sensing is valid.
10. After installing autostart, rebooting Milk-V starts `milkv_drone` without a USB command session.
11. No propellers are installed during every test above.
```

## Roadmap

Suggested next development order:

```text
1. Verify GP10/GP11 IIC2 pinmux and run `mpu_test` on `/dev/i2c-2 0x68`.
2. Bench-test `milkv_drone` gyro calibration and 10 Hz attitude logging without propellers.
3. Install autostart and confirm reboot starts `milkv_drone` without USB commands.
4. Tune RC deadband, throttle curve, and three-axis PID gains on the real airframe.
5. Keep validating three-axis PID outputs through the X-layout motor mixer.
6. Keep motor output capped during early no-prop bench tests.
7. Add telemetry for IMU, attitude, RC, battery, arm state, and safety reason.
8. Only after repeated no-prop tests, prepare a cautious tethered test.
```

## Repository Layout

```text
milkv/
  Makefile
  README.md
  src/
    main.cpp
    loop_rate.hpp
    serial.hpp
    serial.cpp
    protocol.hpp
    protocol.cpp
    imu_mpu6050.hpp
    imu_mpu6050.cpp
    complementary_filter.hpp
    complementary_filter.cpp
    pid.hpp
    pid.cpp
    motor_mixer.hpp
    motor_mixer.cpp
  tests/
    imu_mpu6050_address_test.cpp
    imu_mpu6050_decode_test.cpp
    complementary_filter_validity_test.cpp
```

# milkv_drone

Milk-V Duo 256M / SG2002 based drone controller project.

This repository contains the Milk-V side C++17 controller skeleton. The intended
flight-control architecture is:

```text
Milk-V Duo 256M: main controller, IMU, filter, PID, motor mixing, telemetry
STM32F103: realtime ESC PWM, iBUS receiver, emergency stop, failsafe
```

The Milk-V board is still the main controller required by the assignment. The
STM32 is used as a realtime safety and IO coprocessor because Linux is not a
hard-realtime PWM source. Do not drive the ESCs directly from Milk-V as the only
safety mechanism.

## Current Status

Implemented on the Milk-V side:

- C++17 codebase with multiple `src/*.cpp` and `src/*.hpp` files.
- 100 Hz main loop.
- UART communication with the STM32 side.
- RC and battery telemetry parsing.
- `ARM`, `DISARM`, and motor command encoding.
- MPU6050 Linux I2C interface on `/dev/i2c-2` with I2C address support for
  `0x68` and `0x69`; `WHO_AM_I` accepts `0x68` or observed ID value `0x70`.
- Complementary filter attitude estimation.
- Gyro bias calibration on startup: 200 samples, 10 ms interval.
- 10 Hz attitude and IMU telemetry log output.
- PID class.
- X-layout quad motor mixer.
- Safety state skeleton: RC timeout, failsafe flag, low voltage latch, invalid
  IMU state, disarm switch, and safe-stop on exit.
- Makefile build using the official Milk-V `duo-examples/envsetup.sh`
  environment.

Current safety behavior:

- Motor output is intentionally fixed at `1000us` in this skeleton.
- The program still sends `ARM` / `DISARM` state to exercise the UART protocol.
- Real throttle, PID output, and motor mixing are still not connected to motor
  output and must stay disabled until bench tests pass without propellers.

## Hardware Architecture

Recommended system wiring:

```text
3S battery XT60 -> PDB
PDB -> four ESC power inputs
PDB 5V -> STM32F103 + IA6B receiver
External 5V BEC -> Milk-V Duo 256M + camera + USB WiFi
Milk-V UART -> STM32 USART
IA6B iBUS -> STM32 UART RX
STM32 timer PWM pins -> four ESC signal pins
MPU6050 / BMP280 / OLED -> Milk-V I2C
DHT22 or status IO -> Milk-V GPIO
Battery divider -> STM32 ADC
All signal grounds -> common ground
```

Important safety rules:

- Remove propellers during all software, UART, PWM, and bench tests.
- STM32 must independently stop all ESC outputs at `1000us` if Milk-V dies,
  UART messages stop, the receiver failsafes, or the emergency switch is active.
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

## UART Protocol

Milk-V sends commands to STM32:

```text
ARM\n
DISARM\n
MOT,<m1_us>,<m2_us>,<m3_us>,<m4_us>\n
```

STM32 sends telemetry to Milk-V:

```text
RC,<ch1>,<ch2>,<ch3>,<ch4>,<ch5>,<ch6>,<failsafe>\n
BAT,<voltage_mv>\n
```

PWM values are clamped to:

```text
minimum: 1000us
middle:  1500us
maximum: 2000us
```

The current Milk-V program treats channel 5 as the arm switch:

```text
arm request:    ch5 >= 1800
disarm request: ch5 <= 1200
throttle low:   ch3 <= 1100
RC timeout:     500 ms
low voltage:    < 9600 mV
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
scp milkv_drone mpu_test root@192.168.42.1:/root/
```

Log in and run:

```sh
ssh root@192.168.42.1
chmod +x /root/milkv_drone
/root/milkv_drone <uart_device> [baudrate]
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
- keeps MOT fixed at 1000,1000,1000,1000
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

After bench testing, the program can be started from Milk-V's system startup
script:

```sh
vi /mnt/system/auto.sh
```

Add a line like:

```sh
/root/milkv_drone /dev/ttyS1 115200 &
```

Then:

```sh
chmod +x /mnt/system/auto.sh
reboot
```

Do not enable autostart until safe-stop behavior has been verified on the bench
without propellers.

## Bench Test Checklist

Run these tests before enabling real motor output:

```text
1. Milk-V boots and the program prints a 100 Hz startup message.
2. Program exits cleanly on Ctrl-C and sends several safe-stop commands.
3. STM32 receives DISARM and MOT,1000,1000,1000,1000.
4. RC telemetry updates once the IA6B receiver is active.
5. Pulling the receiver signal triggers failsafe on the STM32.
6. Stopping Milk-V UART messages makes STM32 force all ESC outputs to 1000us.
7. Low voltage telemetry latches disarm on the Milk-V side.
8. Arm switch only arms when throttle is low and all safety conditions are true.
9. No propellers are installed during every test above.
```

## Roadmap

Suggested next development order:

```text
1. Verify GP10/GP11 IIC2 pinmux and run `mpu_test` on `/dev/i2c-2 0x68`.
2. Bench-test `milkv_drone` gyro calibration and 10 Hz attitude logging without propellers.
3. Implement STM32 firmware: iBUS RX, ESC PWM, UART parser, independent failsafe.
4. Add throttle curve and RC deadband.
5. Feed three-axis PID outputs into the X-layout motor mixer.
6. Keep motor output capped or fixed during early no-prop bench tests.
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

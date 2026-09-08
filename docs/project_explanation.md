# Drone Project Explanation

本文档用于快速理解这个仓库：它是什么、代码怎么分层、飞控主循环如何工作、Milk-V 与 STM32 两套代码分别承担什么角色，以及如何构建和验证。

## 1. 项目定位

这是一个小型四旋翼无人机飞控项目。

**当前主路径：Milk-V Duo 256M 直接做飞控主控**
   - 代码位置：`milkv/`
   - 语言：C++17
   - 核心职责：读取 iBUS 遥控输入、读取 MPU6050 IMU、姿态解算、PID 控制、四轴混控、通过 Linux sysfs PWM 直接输出到四个 ESC。
   - 附加功能：低电压检测、GP2 蜂鸣器报警、HC-05 蓝牙遥测、自启动脚本。


```text
iBUS 遥控器 -> Milk-V UART1 -> 遥控通道解析
MPU6050 -> Milk-V I2C2 -> 姿态解算
遥控目标 + 姿态反馈 -> PID -> X 型四旋翼混控 -> sysfs PWM -> 4 个 ESC
电池/蜂鸣器/蓝牙遥测 -> 安全与观测辅助
```

## 2. 目录结构

```text
D:\drone
+-- docs/
|   +-- superpowers/plans/             历史实现计划文档
+-- milkv/
|   +-- Makefile                       Milk-V 主程序、工具程序、host test 构建入口
|   +-- README.md                      Milk-V 侧部署和硬件说明
|   +-- setup_esc_pwm.sh               Milk-V 引脚复用和 PWM 初始化脚本
|   +-- install_autostart.sh           安装开机自启动脚本
|   +-- src/                           Milk-V C++17 源码
|   +-- tests/                         Milk-V 侧主机单元测试
+-- stm32/
|   +-- Makefile                       STM32 裸机构建入口
|   +-- src/                           STM32 C 源码、启动文件、链接脚本
+-- tests/                             早期/根目录 host test
```

仓库里还存在 `build/`、`analysis-build/`、`.exe`、`.elf`、`.bin` 等构建产物。理解项目时主要看 `milkv/src/`、`milkv/tests/`、`stm32/src/` 和两个 `Makefile`。

## 3. Milk-V 主控架构

Milk-V 的入口是 `milkv/src/main.cpp`。主程序启动后按下面顺序初始化：

1. 解析启动参数：
   - 默认 iBUS 设备：`/dev/ttyS1`
   - 默认蓝牙设备：`/dev/ttyS0`
   - 可选参数：`--pwm-chip`、`--battery-file`、`--bluetooth-device`、`--bluetooth-baud`、`--no-bluetooth`、`--test-pid`
2. 打开 iBUS UART。
3. 初始化四路 ESC sysfs PWM。
4. 初始化 GP2 低电压蜂鸣器。
5. 初始化 HC-05 蓝牙遥测串口。
6. 让 ESC 保持 1000us 低油门 5 秒。
7. 初始化 MPU6050，并做 200 次陀螺仪零偏校准。
8. 创建互补滤波器、100Hz 定频循环、遥控映射配置、三轴 PID。
9. 进入 100Hz 主循环。

主循环每一轮完成：

```text
读取最新 iBUS 帧
轮询电池电压
读取 IMU
扣除陀螺仪零偏
互补滤波得到 roll/pitch/yaw
判断 RC 超时、failsafe、急停、IMU 是否有效、低电压锁存
将遥控 PWM 通道转换为归一化飞行指令
判断解锁/上锁状态
armed 时运行 roll/pitch/yaw PID
汇总安全状态和控制量
computeMotorOutput() 得到四路 PWM
写入 ESC sysfs PWM
周期性打印日志和发送蓝牙遥测
sleep 到 100Hz 周期
```

退出时会关闭蜂鸣器、关闭蓝牙、把 ESC 拉回最低油门并 disable PWM。

## 4. Milk-V 核心模块

### 4.1 iBUS 接收

相关文件：

- `milkv/src/ibus_receiver.hpp`
- `milkv/src/ibus_receiver.cpp`

iBUS 帧长度是 32 字节，命令字是 `0x40`，包含 14 路通道。当前主程序只取前 6 路：

```text
CH1 roll
CH2 pitch
CH3 throttle
CH4 yaw
CH5 SWA / 急停
CH6 arm switch / 解锁开关
```

遥控通道正常范围按 `800..2200us` 做 sanity check。超过 200ms 没有新帧，主程序就认为 RC 不新鲜并进入 failsafe。

### 4.2 遥控映射

相关文件：

- `milkv/src/flight_control.hpp`
- `milkv/src/flight_control.cpp`

输入通道是常见的 `1000..2000us` PWM 风格值。模块会转换成：

- roll/pitch/yaw stick：`[-1, 1]`
- throttle：`[0, 1]`
- roll/pitch 目标角度：默认最大 `20deg`
- yaw 目标角速度：默认最大 `90deg/s`

摇杆中位附近有死区，默认 `0.05`。油门还有低油门死区，默认 `0.04`，并使用平方曲线让低油门段更柔和。

### 4.3 IMU 与姿态解算

相关文件：

- `milkv/src/imu_mpu6050.hpp`
- `milkv/src/imu_mpu6050.cpp`
- `milkv/src/complementary_filter.hpp`
- `milkv/src/complementary_filter.cpp`

MPU6050 默认挂在：

```text
I2C device: /dev/i2c-2
default address: 0x68
alternate address: 0x69
```

IMU 读取 14 字节寄存器数据，换算得到：

```text
ax/ay/az: g
gx/gy/gz: deg/s
temperature: Celsius
```

姿态解算使用互补滤波：

```text
roll_acc  = atan2(ay, az)
pitch_acc = atan2(-ax, sqrt(ay^2 + az^2))
angle     = alpha * gyro_integrated_angle + (1 - alpha) * accel_angle
```

默认 `alpha = 0.98`。由于 MPU6050 没有磁力计，yaw 当前只由 `gz` 积分得到，会随时间漂移。

### 4.4 PID 控制

相关文件：

- `milkv/src/pid.hpp`
- `milkv/src/pid.cpp`
- `milkv/src/main.cpp`

主程序里有三组 PID：

```text
roll:  角度闭环，目标 roll_target_deg，反馈 attitude.roll_deg
pitch: 角度闭环，目标 pitch_target_deg，反馈 attitude.pitch_deg
yaw:   角速度闭环，目标 yaw_rate_target_dps，反馈 imu_sample.gz_dps
```

当前增益是台架测试初始值，输出单位按项目约定映射为电机 PWM 修正量 `us`。PID 输出限制在 `[-400, 400]`，积分限制为 `200`。

### 4.5 四轴混控

相关文件：

- `milkv/src/motor_mixer.hpp`
- `milkv/src/motor_mixer.cpp`

混控采用 X 型四旋翼公式：

```text
m1 左前 = T + P + R - Y
m2 右前 = T + P - R + Y
m3 左后 = T - P + R + Y
m4 右后 = T - P - R - Y
```

其中：

- `T` 是基础油门 PWM。
- `R/P/Y` 是 roll/pitch/yaw 修正量。
- 最终会裁剪到安全 PWM 范围。

### 4.6 电机输出安全逻辑

相关文件：

- `milkv/src/motor_output_logic.hpp`
- `milkv/src/motor_output_logic.cpp`

`computeMotorOutput()` 是电机输出的集中裁决点。它会根据安全状态决定是否允许输出。

会直接保持 `1000us` 的情况：

- 急停触发。
- 未解锁。
- RC 无效。
- failsafe。
- IMU 无效。
- 油门为 0。

低电压触发后不会直接断电，而是进入保守降高：

```text
四路电机统一输出 1200us
motor_output_enabled_reason = low_voltage_descent
```

正常输出时，基础油门按：

```text
base_us = 1000 + throttle * (1800 - 1000)
```

当前电机上限是 `1800us`，低于 ESC 常见 `2000us` 上限，适合早期台架和谨慎试飞。

### 4.7 ESC sysfs PWM

相关文件：

- `milkv/src/esc_pwm_sysfs.hpp`
- `milkv/src/esc_pwm_sysfs.cpp`
- `milkv/setup_esc_pwm.sh`

ESC 控制使用 Linux `/sys/class/pwm`：

```text
period_ns = 20,000,000   # 50Hz
duty_ns   = pulse_us * 1000
```

默认四路映射：

```text
ESC1 左前 -> GP4  / PWM_5 / pwmchip4 pwm1
ESC2 右前 -> GP5  / PWM_6 / pwmchip4 pwm2
ESC3 左后 -> GP12 / PWM_4 / pwmchip4 pwm0
ESC4 右后 -> GP6  / PWM_9 / pwmchip8 pwm1
```

上板前需要运行 `setup_esc_pwm.sh` 配置 pinmux。

### 4.8 电池、蜂鸣器、蓝牙遥测

相关文件：

- `milkv/src/battery_monitor.*`
- `milkv/src/buzzer_gpio.*`
- `milkv/src/bluetooth_telemetry.*`
- `milkv/src/serial.*`

电池监控当前支持从文本源读取：

```text
BAT,<millivolts>
```

或纯数字毫伏值。低电压阈值在主程序中是 `9600mV`。触发后状态会锁存，防止电压回弹后重新解锁。

蜂鸣器默认使用 GP2，对低电压做 500ms 周期提示：

```text
前 250ms 响
后 250ms 停
```

蓝牙遥测输出给 HC-05，默认 `/dev/ttyS0`、`9600 baud`，格式：

```text
TEL,roll,pitch,yaw,battery_mv,low_voltage,armed,function_sensor_temp_c
```

其中 `function_sensor_temp_c` 当前复用 MPU6050 温度。

## 5. Milk-V 安全解锁逻辑

主程序的关键安全条件：

```text
RC 新鲜：最近 200ms 内收到有效 iBUS 帧
SWA/CH5：>1500us 视为急停
CH6：>1500us 请求解锁，<=1500us 上锁
油门低位：CH3 <=1100us 才允许从未解锁进入解锁
IMU：初始化和当前采样都必须有效
低电压：触发后锁存，不允许再次正常解锁
```

解锁开关还有一个防上电误解锁设计：CH6 必须先处于低位，之后再拨到高位，才会被认为是有效解锁请求。

安全优先级可以理解为：

```text
急停 / RC 丢失 / failsafe / IMU 无效 / 解锁条件不满足
  -> 四路 1000us

已解锁但低电压锁存
  -> 四路 1200us 保守降高

已解锁且所有条件正常且有油门
  -> PID + 混控输出
```

## 6. STM32 固件说明

STM32 入口是 `stm32/src/main.c`。它是裸机程序，直接操作寄存器和外设。

模块划分：

```text
ibus.c/h     解析 FlySky iBUS
uart.c/h     USART2 与 Milk-V 文本行通信
pwm.c/h      TIM3 输出四路 50Hz ESC PWM
adc.c/h      PA4 ADC 读取电池分压并换算 mV
regs.h       STM32F103 寄存器定义
startup.c    启动代码和中断向量
*.ld         链接脚本
```

STM32 方案中的文本协议包括：

```text
RC,ch1,ch2,ch3,ch4,ch5,ch6,failsafe
BAT,<millivolts>
MOT,m1,m2,m3,m4
ARM
DISARM
```

它的安全逻辑是：

- 物理急停 PA1 触发时停机。
- iBUS failsafe 时停机。
- 超过 500ms 未收到 Milk-V 指令时停机。
- 未 armed 时停机。

这套固件可以作为备选硬件架构参考，但当前 Milk-V 主程序已经直接读取 iBUS 并直接输出 ESC PWM，所以阅读项目时应优先理解 Milk-V 路径。

## 7. 构建方式

### 7.1 Milk-V

`milkv/Makefile` 会构建：

```text
milkv_drone   主飞控程序
mpu_test      MPU6050 测试程序
ibus_read     iBUS 读取测试程序
```

板端构建需要 Milk-V 官方 `duo-examples/envsetup.sh` 提供的 RISC-V musl 工具链。Makefile 会主动检查：

- `TOOLCHAIN_PREFIX` 是否设置。
- `CFLAGS`/`LDFLAGS` 是否来自环境。
- `CHIP` 是否是 `CV181X`。
- 防止误用 Ubuntu 默认 `riscv64-linux-gnu` 工具链。

常用命令：

```sh
cd milkv
make
make clean
```

### 7.2 Milk-V host tests

主机侧测试入口：

```sh
cd milkv
make host-test
```

当前 Makefile 中的 host-test 覆盖：

```text
autostart_script_test
battery_monitor_test
bluetooth_telemetry_test
buzzer_gpio_test
flight_control_test
motor_output_logic_test
```

`milkv/tests/` 里还保留了更多模块测试文件，例如 IMU 解码、iBUS 解析、ESC sysfs、互补滤波等；有些未接入当前 Makefile 的 `host-test` 目标。

### 7.3 STM32

STM32 构建依赖 ARM GCC：

```sh
cd stm32
make
make flash
make clean
```

输出产物：

```text
drone_stm32.elf
drone_stm32.bin
```

烧录目标地址是 `0x08000000`。

## 8. 上板运行路径

Milk-V 侧典型流程：

```sh
scp milkv_drone mpu_test ibus_read setup_esc_pwm.sh install_autostart.sh root@192.168.42.1:/root/
ssh root@192.168.42.1
chmod +x /root/setup_esc_pwm.sh /root/install_autostart.sh /root/milkv_drone
/root/setup_esc_pwm.sh
/root/milkv_drone /dev/ttyS1
```

如果要模拟电池输入：

```sh
/root/milkv_drone /dev/ttyS1 --battery-file /tmp/drone_battery_mv
```

如果要指定蓝牙：

```sh
/root/milkv_drone /dev/ttyS1 --bluetooth-device /dev/ttyS0 --bluetooth-baud 9600
```

关闭蓝牙：

```sh
/root/milkv_drone /dev/ttyS1 --no-bluetooth
```

安装自启动：

```sh
sh /root/install_autostart.sh
reboot
```

安装后系统会生成或更新 `/mnt/system/auto.sh`，启动日志通常写入 `/root/milkv_drone.log`。

## 9. 建议阅读顺序

如果你是第一次接手这个项目，建议按这个顺序读：

1. `milkv/src/main.cpp`
   - 先看整体初始化和 100Hz 主循环。
2. `milkv/src/motor_output_logic.cpp`
   - 理解所有安全条件如何影响最终电机输出。
3. `milkv/src/flight_control.cpp`
   - 理解遥控通道如何变成目标角度、目标角速度和油门。
4. `milkv/src/complementary_filter.cpp`
   - 理解 IMU 姿态估计。
5. `milkv/src/motor_mixer.cpp`
   - 理解四轴 X 型混控公式。
6. `milkv/src/esc_pwm_sysfs.cpp`
   - 理解 Milk-V 如何把 PWM 写到 Linux sysfs。
7. `milkv/tests/*_test.cpp`
   - 用测试确认模块的边界和预期行为。
8. `stm32/src/main.c`
   - 最后再看旧/辅助 STM32 协议路径。

## 10. 关键风险和调试重点

### Linux 非硬实时

Milk-V 跑 Linux，100Hz 主循环不是硬实时。台架阶段必须观察循环稳定性、PWM 写入延迟和系统负载。

### Yaw 会漂移

MPU6050 没有磁力计，yaw 只靠陀螺仪积分。长时间运行必然漂移，适合短期姿态观察，不适合作为长期航向基准。

### 混控方向必须台架验证

X 型混控公式写得清楚，但真实飞机还取决于：

- IMU 安装方向。
- 电机编号。
- 螺旋桨方向。
- ESC 信号线顺序。
- roll/pitch/yaw 正方向定义。

必须先拆桨验证“机体向某方向倾斜时，对应电机补偿方向是否正确”。

### 低电压策略是保守降高，不是断电

低电压锁存后输出四路 `1200us`。这比直接断电温和，但也意味着真实飞行中会继续给电机一个固定小油门。实际阈值、降高油门和触发条件都要结合电池、机架、重量重新验证。

### 当前测试覆盖不等于可飞

host tests 能验证纯逻辑和解析，但不能覆盖：

- Milk-V 实际 pinmux 名称。
- `/sys/class/pwm` 在目标镜像上的真实路径。
- IMU 噪声、安装方向和振动。
- ESC 校准。
- RC failsafe 的真实接收机行为。
- Linux 调度延迟。

因此上电顺序应是：

```text
无桨静态检查
无桨电机方向检查
无桨 PID 修正方向检查
限油门台架检查
小油门系留测试
空旷场地低高度试飞
```

## 11. 当前项目状态总结

当前仓库的主线已经具备一个完整四旋翼飞控骨架：

- 有遥控输入。
- 有 IMU 姿态估计。
- 有三轴 PID。
- 有 X 型四轴混控。
- 有四路 ESC PWM 输出。
- 有急停、failsafe、IMU 无效、低油门解锁、低电压锁存等安全保护。
- 有低电压蜂鸣器和蓝牙遥测。
- 有主机侧单元测试覆盖部分核心逻辑。

下一步最值得做的不是继续堆功能，而是做真实硬件验证闭环：

1. 确认 Milk-V 引脚复用和 sysfs PWM 路径。
2. 确认四个 ESC 输出顺序。
3. 确认 IMU 安装方向与姿态符号。
4. 确认混控补偿方向。
5. 在无桨状态下记录日志，调小/调稳 PID 初始参数。
6. 把更多已有测试接入 `make host-test`，让 CI/本地验证更完整。

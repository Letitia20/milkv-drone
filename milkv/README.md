# Milk-V Duo 256 无人机飞控项目

本目录是基于 Milk-V Duo 256M / SG2002 的无人机飞控程序。当前方案以
Milk-V 为主控，主要代码使用 C++17 编写，负责遥控输入、IMU 姿态解算、
PID 控制、电机混控、ESC PWM 输出、低电压报警和 HC-05 蓝牙遥测。

```text
Milk-V Duo 256M:
  iBUS 遥控输入 -> MPU6050 姿态采集 -> 互补滤波 -> 三轴 PID
  -> X 型四轴混控 -> sysfs PWM 输出到四个 ESC
  -> GP2 蜂鸣器低电压报警
  -> GP0/GP1 连接 HC-05 蓝牙模块发送遥测
```

安全提醒：Linux 不是硬实时飞控系统，早期调试必须拆下螺旋桨，保留
PWM 上限，手边准备实体断电开关，只在空旷无人场地进行上电和试飞。

## 当前完成情况

- Milk-V 端 C++17 飞控主程序，主循环频率为 100Hz。
- iBUS 接收机输入，默认设备 `/dev/ttyS1`。
- MPU6050 I2C 驱动，默认设备 `/dev/i2c-2`，地址支持 `0x68` 和 `0x69`。
- 陀螺仪上电零偏校准：200 次采样，每次间隔 10ms。
- 互补滤波姿态解算，输出 roll、pitch、yaw。
- roll、pitch、yaw 三轴 PID 控制。
- 遥控器摇杆死区和平方油门曲线。
- X 型四轴电机混控。
- 四路 ESC sysfs PWM 输出。
- SWA/CH5 急停，CH6 解锁/上锁。
- 遥控超时、failsafe、IMU 无效、低油门解锁门限等安全状态。
- 低电压检测：低于 `9600mV` 后锁定低电压状态。
- GP2 蜂鸣器低电压报警：500ms 周期，前 250ms 响，后 250ms 停。
- 低电压受控降高：低电压后四路电机限制到 `1200us`。
- HC-05 蓝牙遥测：默认 `/dev/ttyS0`、9600 baud，使用 GP0/GP1。
- Milk-V 开机自启动脚本安装到 `/mnt/system/auto.sh`。
- 主机侧测试覆盖电池解析、蜂鸣器节奏、蓝牙遥测、油门曲线、电机输出逻辑等。

## 硬件连接

```text
3S 电池 XT60 -> PDB
PDB -> 四个 ESC 电源输入
PDB 或独立 5V BEC -> IA6B 接收机
独立 5V BEC -> Milk-V Duo 256M + 摄像头/蓝牙/无线模块
IA6B iBUS -> Milk-V GP3 / UART1_RX
Milk-V PWM 引脚 -> 四个 ESC 信号线
MPU6050 VCC -> Milk-V 3.3V
MPU6050 GND -> Milk-V GND
MPU6050 SDA -> Milk-V GP10 / IIC2_SDA
MPU6050 SCL -> Milk-V GP11 / IIC2_SCL
蜂鸣器信号 -> Milk-V GP2 / GPIO 输出
HC-05 TX -> Milk-V GP1 / UART0_RX
HC-05 RX -> Milk-V GP0 / UART0_TX
所有信号地 -> 共地
```

### Milk-V 引脚总表

| 模块 | Milk-V 引脚 | 复用功能 | Linux/程序侧 | 备注 |
| --- | --- | --- | --- | --- |
| ESC1 左前 | GP4 / Pin 6 | PWM_5 | pwmchip4/pwm1 | X 型四轴电机 1 |
| ESC2 右前 | GP5 / Pin 7 | PWM_6 | pwmchip4/pwm2 | X 型四轴电机 2 |
| ESC3 左后 | GP12 / Pin 16 | PWM_4 | pwmchip4/pwm0 | X 型四轴电机 3 |
| ESC4 右后 | GP6 / Pin 9 | PWM_9 | pwmchip8/pwm1 | X 型四轴电机 4 |
| iBUS 接收机 | GP3 | UART1_RX | /dev/ttyS1 | 115200 8N1 raw |
| 低电压蜂鸣器 | GP2 | GPIO | /sys/class/gpio/gpio2 | 不再用于 ESC |
| HC-05 TX -> Milk-V RX | GP1 | UART0_RX | /dev/ttyS0 | 蓝牙遥测输入脚 |
| HC-05 RX <- Milk-V TX | GP0 | UART0_TX | /dev/ttyS0 | HC-05 RX 需确认 3.3V 兼容 |
| MPU6050 SDA | GP10 | IIC2_SDA | /dev/i2c-2 | 地址 0x68/0x69 |
| MPU6050 SCL | GP11 | IIC2_SCL | /dev/i2c-2 | 地址 0x68/0x69 |

ESC 不占用 GP2，GP2 固定留给低电压蜂鸣器。不要使用 GP26/GP27 做 ESC 信号线，
它们是 1.8V 引脚。

注意：

- 不要把电池电压直接接到 ADC 引脚，必须使用确认过的分压电路。
- HC-05 的 RX 若不是 3.3V 兼容，需要做电平转换或限流保护。
- 不要把 Milk-V、摄像头、蓝牙和 WiFi 都接在电流不足的 PDB 5V 输出上。
- 裸板不要直接贴在碳纤维机架上，必须绝缘固定。
- 所有无桨测试通过前，不安装螺旋桨。

## 引脚复用

运行 `setup_esc_pwm.sh` 会配置以下引脚：

```sh
duo-pinmux -w GP4/PWM_5
duo-pinmux -w GP5/PWM_6
duo-pinmux -w GP12/PWM_4
duo-pinmux -w GP6/PWM_9
duo-pinmux -w GP3/UART1_RX
duo-pinmux -w GP2/GPIO
duo-pinmux -w GP0/UART0_TX
duo-pinmux -w GP1/UART0_RX
duo-pinmux -w GP10/IIC2_SDA
duo-pinmux -w GP11/IIC2_SCL
```

上板前建议先查看实际系统支持的复用名称：

```sh
duo-pinmux -p
duo-pinmux -l
duo-pinmux -r GP0
duo-pinmux -r GP1
duo-pinmux -r GP2
duo-pinmux -r GP3
duo-pinmux -r GP4
duo-pinmux -r GP5
duo-pinmux -r GP6
duo-pinmux -r GP10
duo-pinmux -r GP11
duo-pinmux -r GP12
```

## 关键公式

### 1. IMU 原始值换算

MPU6050 连续读取 14 字节数据，换算公式为：

```text
ax_g = raw_ax / 16384.0
ay_g = raw_ay / 16384.0
az_g = raw_az / 16384.0
gx_dps = raw_gx / 131.0
gy_dps = raw_gy / 131.0
gz_dps = raw_gz / 131.0
temperature_c = raw_temp / 340.0 + 36.53
```

### 2. 加速度姿态角

```text
roll_acc = atan2(ay, az) * 180 / pi
pitch_acc = atan2(-ax, sqrt(ay^2 + az^2)) * 180 / pi
```

### 3. 陀螺仪积分

```text
angle_gyro(k) = angle(k-1) + gyro_dps * dt
```

### 4. 互补滤波

```text
angle = alpha * angle_gyro + (1 - alpha) * angle_acc
```

当前默认 `alpha = 0.98`。yaw 因为 MPU6050 没有磁力计，暂时只由 `gz` 积分。

### 5. PID 控制

```text
error = setpoint - measurement
integral = integral + error * dt
derivative = (error - previous_error) / dt
output = Kp * error + Ki * integral + Kd * derivative
```

roll/pitch 使用姿态角闭环，yaw 使用角速度闭环。PID 输出单位在本项目中映射为
电机 PWM 修正量，单位为 `us`。

### 6. 遥控死区

```text
abs(x) <= d 时：output = 0
abs(x) > d 时：output = sign(x) * (abs(x) - d) / (1 - d)
```

### 7. 平方油门曲线

```text
throttle <= d 时：output = 0
throttle > d 时：output = ((throttle - d) / (1 - d))^2
```

平方曲线让低油门段更柔和，减少电机突然启动冲击。

### 8. 油门到 PWM

```text
base_us = 1000 + throttle * (motor_max_us - 1000)
```

当前早期测试上限为：

```text
motor_max_us = 1800
```

### 9. X 型四轴混控

电机顺序：

```text
m1 左前 = T + P + R - Y
m2 右前 = T + P - R + Y
m3 左后 = T - P + R + Y
m4 右后 = T - P - R - Y
```

其中 `T` 为基础油门，`R/P/Y` 分别为 roll、pitch、yaw 修正量。

### 10. ESC PWM 周期

ESC 常用 50Hz 控制周期：

```text
period = 20ms = 20,000,000ns
duty_cycle_ns = pulse_us * 1000
```

### 11. 低电压报警与降高

```text
低电压阈值：battery_mv < 9600
蜂鸣器周期：phase = elapsed_ms % 500
phase < 250 时蜂鸣器打开，否则关闭
低电压降高 PWM：m1 = m2 = m3 = m4 = 1200us
```

低电压锁定后不再允许重新解锁起飞；若已经飞行，则进入保守油门降高状态。

## 遥控与安全逻辑

```text
SWA 正常：       ch5 ~= 1000
SWA 急停：       ch5 ~= 2000
急停判断：       ch5 > 1500
解锁请求：       ch6 > 1500，且 ch6 曾经处于低位
上锁请求：       ch6 <= 1500
低油门解锁门限： ch3 <= 1100
遥控超时：       200ms
低电压：         battery_mv < 9600
```

电机输出保持 `1000us`，除非同时满足：

- 遥控数据新鲜；
- failsafe 未触发；
- IMU 有效；
- SWA/CH5 没有急停；
- CH6 解锁流程正确；
- 解锁前油门处于低位；
- 没有低电压锁定后重新解锁。

安全优先级：急停、未解锁、遥控失效、failsafe、IMU 无效都会优先停桨；
低电压在已解锁飞行时进入 `1200us` 降高。

## 蓝牙遥测

HC-05 默认参数：

```text
设备：/dev/ttyS0
波特率：9600
引脚：GP0/GP1
周期：10Hz
```

输出格式：

```text
TEL,<roll_deg>,<pitch_deg>,<yaw_deg>,<battery_mv>,<low_voltage>,<armed>,<function_sensor_temp_c>
```

示例：

```text
TEL,1.23,-2.35,30.00,9500,1,0,28.75
```

这里的 `function_sensor_temp_c` 使用 MPU6050 温度作为一路功能传感器数据，
可用手机蓝牙串口助手直接显示或记录。

## 构建方法

必须使用 Milk-V 官方 `duo-examples` 环境，不要使用 Ubuntu 默认
`riscv64-linux-gnu-g++`：

```sh
git clone https://github.com/milkv-duo/duo-examples.git
cd duo-examples
source envsetup.sh
```

选择：

```text
Product: 2
Arch:    2
```

然后构建：

```sh
cd /path/to/milkv
make
```

清理：

```sh
make clean
```

主机侧测试：

```sh
make host-test
```

注意：在 Windows PowerShell 中如果没有 `make`，可以用 `g++` 单独编译测试，
或在 Linux/WSL/Milk-V 构建环境中运行。

## 上传和运行

上传到 Milk-V：

```sh
scp milkv_drone mpu_test ibus_read setup_esc_pwm.sh install_autostart.sh root@192.168.42.1:/root/
```

登录运行：

```sh
ssh root@192.168.42.1
chmod +x /root/milkv_drone
chmod +x /root/setup_esc_pwm.sh
/root/setup_esc_pwm.sh
/root/milkv_drone /dev/ttyS1 115200
```

带电池文件模拟输入：

```sh
/root/milkv_drone /dev/ttyS1 115200 --battery-file /tmp/drone_battery_mv
```

带蓝牙参数：

```sh
/root/milkv_drone /dev/ttyS1 115200 --bluetooth-device /dev/ttyS0 --bluetooth-baud 9600
```

关闭蓝牙：

```sh
/root/milkv_drone /dev/ttyS1 115200 --no-bluetooth
```

## 自启动

无桨台架测试通过后安装自启动：

```sh
chmod +x /root/install_autostart.sh
sh /root/install_autostart.sh
reboot
```

可选覆盖参数：

```sh
IBUS_DEVICE=/dev/ttyS1 BAUDRATE=115200 PWM_CHIP=pwmchip4 BLUETOOTH_DEVICE=/dev/ttyS0 BLUETOOTH_BAUD=9600 sh /root/install_autostart.sh
```

安装后每次开机会执行 `/mnt/system/auto.sh`，自动运行
`/root/setup_esc_pwm.sh` 并启动 `/root/milkv_drone`，日志写入：

```text
/root/milkv_drone.log
```

## 无桨台架测试清单

1. Milk-V 能启动程序，并打印 100Hz 启动信息。
2. `Ctrl-C` 退出时四路 ESC 都回到 `1000us`。
3. 未解锁时 ESC 一直保持 `1000us`。
4. iBUS 接收机通道值能实时变化。
5. 拔掉接收机信号后触发 failsafe，四路输出回到 `1000us`。
6. SWA/CH5 打到高位后触发急停，四路输出回到 `1000us`。
7. 急停恢复后不能自动重新解锁，必须重新走解锁流程。
8. CH6 只有在油门低位且安全条件满足时才能解锁。
9. 写入低电压文件后，GP2 蜂鸣器按周期鸣叫。
10. 低电压后电机输出限制为 `1200us`，不是从正常油门直接断电。
11. HC-05 在手机蓝牙串口助手中能看到 `TEL,...` 遥测行。
12. 安装自启动后，重启 Milk-V 能自动运行 `milkv_drone`。
13. 上述测试全部不安装螺旋桨。

## 目录结构

```text
milkv/
  Makefile
  README.md
  setup_esc_pwm.sh
  install_autostart.sh
  src/
    main.cpp                    主飞控循环
    imu_mpu6050.*               MPU6050 驱动和原始值换算
    complementary_filter.*      互补滤波姿态解算
    pid.*                       PID 控制器
    flight_control.*            遥控死区、油门曲线、目标角生成
    motor_mixer.*               X 型四轴混控
    motor_output_logic.*        安全状态和电机输出逻辑
    esc_pwm_sysfs.*             sysfs PWM 输出
    battery_monitor.*           电池电压输入解析
    buzzer_gpio.*               GP2 蜂鸣器
    bluetooth_telemetry.*       HC-05 遥测编码
    ibus_receiver.*             iBUS 接收机解析
    serial.*                    POSIX 串口封装
  tests/
    *_test.cpp                  主机侧测试
```

## 后续实物验证

- 确认 GP0/GP1/GP2 在实际镜像上的 pinmux 名称正确。
- 确认 HC-05 电平兼容。
- 标定 MPU6050 安装方向和陀螺零偏。
- 无桨验证 PID 输出方向：向右倾斜时，左侧电机应补偿升高，右侧应降低。
- 低电压报警和 `1200us` 降高逻辑必须先在无桨状态验证。
- 只有在所有台架测试通过后，才能进行系留试飞和小油门调参。

#!/bin/sh
set -eu

# 功能：配置 Milk-V 引脚复用。
# PWM 用于四个 ESC，UART1 接 iBUS，I2C2 接 MPU6050，GP2 接蜂鸣器，UART0 接 HC-05。
echo "Configuring Milk-V Duo pinmux for ESC PWM, iBUS UART, MPU6050 I2C, buzzer, and HC-05..."

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

echo "Pinmux configured:"
duo-pinmux -r GP4
duo-pinmux -r GP5
duo-pinmux -r GP12
duo-pinmux -r GP6
duo-pinmux -r GP3
duo-pinmux -r GP2
duo-pinmux -r GP0
duo-pinmux -r GP1
duo-pinmux -r GP10
duo-pinmux -r GP11

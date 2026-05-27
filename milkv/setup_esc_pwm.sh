#!/bin/sh
set -eu

echo "Configuring Milk-V Duo pinmux for ESC PWM, iBUS UART, and MPU6050 I2C..."

duo-pinmux -w GP4/PWM_5
duo-pinmux -w GP5/PWM_6
duo-pinmux -w GP12/PWM_4
duo-pinmux -w GP6/PWM_9

duo-pinmux -w GP3/UART1_RX
duo-pinmux -w GP10/IIC2_SDA
duo-pinmux -w GP11/IIC2_SCL

echo "Pinmux configured:"
duo-pinmux -r GP4
duo-pinmux -r GP5
duo-pinmux -r GP12
duo-pinmux -r GP6
duo-pinmux -r GP3
duo-pinmux -r GP10
duo-pinmux -r GP11

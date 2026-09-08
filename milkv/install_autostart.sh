#!/bin/sh
set -eu

# 功能：把 milkv_drone 写入 Milk-V 开机自启动脚本 /mnt/system/auto.sh。
# 生成的 auto.sh 会先配置 pinmux，再后台启动飞控并写入日志。
AUTO_SH="/mnt/system/auto.sh"
APP="/root/milkv_drone"
PINMUX_SCRIPT="/root/setup_esc_pwm.sh"
LOG="/root/milkv_drone.log"
IBUS_DEVICE="${IBUS_DEVICE:-/dev/ttyS1}"
BAUDRATE="${BAUDRATE:-115200}"
PWM_CHIP="${PWM_CHIP:-}"
BLUETOOTH_DEVICE="${BLUETOOTH_DEVICE:-/dev/ttyS0}"
BLUETOOTH_BAUD="${BLUETOOTH_BAUD:-9600}"

if [ ! -f "${APP}" ]; then
    echo "missing ${APP}; upload milkv_drone to /root first" >&2
    exit 1
fi

chmod +x "${APP}"
if [ -f "${PINMUX_SCRIPT}" ]; then
    chmod +x "${PINMUX_SCRIPT}"
fi

mkdir -p "$(dirname "${AUTO_SH}")"

{
    echo "#!/bin/sh"
    echo "APP=\"${APP}\""
    echo "PINMUX_SCRIPT=\"${PINMUX_SCRIPT}\""
    echo "LOG=\"${LOG}\""
    echo "IBUS_DEVICE=\"${IBUS_DEVICE}\""
    echo "BAUDRATE=\"${BAUDRATE}\""
    echo "PWM_CHIP=\"${PWM_CHIP}\""
    echo "BLUETOOTH_DEVICE=\"${BLUETOOTH_DEVICE}\""
    echo "BLUETOOTH_BAUD=\"${BLUETOOTH_BAUD}\""
    echo ""
    echo 'if [ -x "${PINMUX_SCRIPT}" ]; then'
    echo '    "${PINMUX_SCRIPT}" >> "${LOG}" 2>&1'
    echo 'fi'
    echo ""
    echo 'if pidof milkv_drone >/dev/null 2>&1; then'
    echo '    exit 0'
    echo 'fi'
    echo ""
    echo 'if [ -n "${PWM_CHIP}" ]; then'
    echo '    "${APP}" "${IBUS_DEVICE}" "${BAUDRATE}" --pwm-chip "${PWM_CHIP}" --bluetooth-device "${BLUETOOTH_DEVICE}" --bluetooth-baud "${BLUETOOTH_BAUD}" >> "${LOG}" 2>&1 &'
    echo 'else'
    echo '    "${APP}" "${IBUS_DEVICE}" "${BAUDRATE}" --bluetooth-device "${BLUETOOTH_DEVICE}" --bluetooth-baud "${BLUETOOTH_BAUD}" >> "${LOG}" 2>&1 &'
    echo 'fi'
} > "${AUTO_SH}"

chmod +x "${AUTO_SH}"

echo "Installed Milk-V drone autostart at ${AUTO_SH}"
echo "Runtime command will use iBUS ${IBUS_DEVICE}, baud ${BAUDRATE}"
echo "HC-05 telemetry will use ${BLUETOOTH_DEVICE}, baud ${BLUETOOTH_BAUD}"
if [ -n "${PWM_CHIP}" ]; then
    echo "Forced PWM chip: ${PWM_CHIP}"
fi
echo "Reboot the Milk-V after bench checks: reboot"

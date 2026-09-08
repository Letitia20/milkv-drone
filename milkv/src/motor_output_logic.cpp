#include "motor_output_logic.hpp"

#include "motor_mixer.hpp"
#include "protocol.hpp"

#include <algorithm>
#include <cmath>

namespace drone {
namespace {

constexpr int kLowVoltageDescentUs = 1200;

int clampBenchPwm(int value) {
    return std::clamp(value, protocol::kPwmMinUs, kMotorMaxUs);
}

int throttleToBaseUs(float throttle) {
    const float safe_throttle = std::clamp(throttle, 0.0f, 1.0f);
    // 油门到 PWM 基础量：base_us = 1000 + throttle * (motor_max_us - 1000)
    const double base = static_cast<double>(protocol::kPwmMinUs) +
                        static_cast<double>(safe_throttle) *
                            static_cast<double>(kMotorMaxUs - protocol::kPwmMinUs);
    return clampBenchPwm(static_cast<int>(std::lround(base)));
}

int stickToMixUs(float stick, int scale_us) {
    const float safe_stick = std::clamp(stick, -1.0f, 1.0f);
    // 摇杆归一化量转混控修正量：mix_us = stick * scale_us
    return static_cast<int>(std::lround(static_cast<double>(safe_stick) *
                                        static_cast<double>(scale_us)));
}

int correctionToMixUs(double correction_us) {
    const int max_mix_us = kMotorMaxUs - protocol::kPwmMinUs;
    return std::clamp(static_cast<int>(std::lround(correction_us)), -max_mix_us, max_mix_us);
}

}  // namespace

bool isEmergencyStopSwaUs(int swa_us) {
    return swa_us > kEmergencyStopSwaThresholdUs;
}

MotorOutputResult computeMotorOutput(const MotorOutputInput& input) {
    MotorOutputResult result;
    // 低电压触发后不直接断电，改为 1200us 受控降高。
    result.base_us = input.low_voltage ? kLowVoltageDescentUs : throttleToBaseUs(input.throttle);
    if (input.use_pid_corrections) {
        result.roll_mix_us = correctionToMixUs(input.roll_correction_us);
        result.pitch_mix_us = correctionToMixUs(input.pitch_correction_us);
        result.yaw_mix_us = correctionToMixUs(input.yaw_correction_us);
    } else {
        result.roll_mix_us = stickToMixUs(input.roll_cmd, 200);
        result.pitch_mix_us = stickToMixUs(input.pitch_cmd, 200);
        result.yaw_mix_us = stickToMixUs(input.yaw_cmd, 150);
    }

    if (input.emergency_stop) {
        result.motor_output_enabled_reason = "emergency_stop";
        return result;
    }
    if (!input.armed) {
        result.motor_output_enabled_reason = "disabled_not_armed";
        return result;
    }
    if (!input.rc_valid) {
        result.motor_output_enabled_reason = "disabled_rc_invalid";
        return result;
    }
    if (input.failsafe) {
        result.motor_output_enabled_reason = "disabled_failsafe";
        return result;
    }
    if (input.invalid_imu) {
        result.motor_output_enabled_reason = "disabled_invalid_imu";
        return result;
    }
    if (input.throttle <= 0.0f) {
        result.motor_output_enabled_reason = "disabled_throttle_zero";
        return result;
    }
    if (input.low_voltage) {
        // 低电压降高时关闭姿态混控修正，四路保持同一保守油门。
        result.mixer_before_clamp = {result.base_us, result.base_us, result.base_us, result.base_us};
        result.motors_after_clamp = result.mixer_before_clamp;
        result.motor_output_enabled_reason = "low_voltage_descent";
        return result;
    }

    MotorMixer mixer;
    result.mixer_before_clamp = mixer
                                    .mix(result.base_us,
                                         result.roll_mix_us,
                                         result.pitch_mix_us,
                                         result.yaw_mix_us)
                                    .asArray();

    for (std::size_t i = 0; i < result.motors_after_clamp.size(); ++i) {
        result.motors_after_clamp[i] = clampBenchPwm(result.mixer_before_clamp[i]);
    }

    result.motor_output_enabled_reason = "enabled";
    return result;
}

}  // namespace drone

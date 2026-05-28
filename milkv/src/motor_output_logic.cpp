#include "motor_output_logic.hpp"

#include "motor_mixer.hpp"
#include "protocol.hpp"

#include <algorithm>
#include <cmath>

namespace drone {
namespace {

int clampBenchPwm(int value) {
    return std::clamp(value, protocol::kPwmMinUs, kMotorMaxUs);
}

int throttleToBaseUs(float throttle) {
    const float safe_throttle = std::clamp(throttle, 0.0f, 1.0f);
    const double base = static_cast<double>(protocol::kPwmMinUs) +
                        static_cast<double>(safe_throttle) *
                            static_cast<double>(kMotorMaxUs - protocol::kPwmMinUs);
    return clampBenchPwm(static_cast<int>(std::lround(base)));
}

int stickToMixUs(float stick, int scale_us) {
    const float safe_stick = std::clamp(stick, -1.0f, 1.0f);
    return static_cast<int>(std::lround(static_cast<double>(safe_stick) *
                                        static_cast<double>(scale_us)));
}

}  // namespace

MotorOutputResult computeMotorOutput(const MotorOutputInput& input) {
    MotorOutputResult result;
    result.base_us = throttleToBaseUs(input.throttle);
    result.roll_mix_us = stickToMixUs(input.roll_cmd, 200);
    result.pitch_mix_us = stickToMixUs(input.pitch_cmd, 200);
    result.yaw_mix_us = stickToMixUs(input.yaw_cmd, 150);

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

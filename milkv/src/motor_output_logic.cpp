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
                        static_cast<double>(safe_throttle) * 1000.0;
    return clampBenchPwm(static_cast<int>(std::lround(base)));
}

}  // namespace

MotorOutputResult computeMotorOutput(const MotorOutputInput& input) {
    MotorOutputResult result;
    result.base_us = throttleToBaseUs(input.throttle);

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
    if (input.throttle <= 0.0f) {
        result.motor_output_enabled_reason = "disabled_throttle_zero";
        return result;
    }

    MotorMixer mixer;
    result.mixer_before_clamp = mixer
                                    .mix(result.base_us,
                                         input.roll_correction_us,
                                         input.pitch_correction_us,
                                         input.yaw_correction_us)
                                    .asArray();

    for (std::size_t i = 0; i < result.motors_after_clamp.size(); ++i) {
        result.motors_after_clamp[i] = clampBenchPwm(result.mixer_before_clamp[i]);
    }

    result.motor_output_enabled_reason = "enabled";
    return result;
}

}  // namespace drone

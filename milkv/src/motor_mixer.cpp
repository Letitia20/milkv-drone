#include "motor_mixer.hpp"

#include <algorithm>
#include <cmath>

namespace drone {

std::array<int, 4> MotorOutputs::asArray() const {
    return {m1, m2, m3, m4};
}

MotorOutputs MotorMixer::mix(double throttle_us,
                             double roll_correction_us,
                             double pitch_correction_us,
                             double yaw_correction_us) const {
    const double throttle = std::clamp(
        throttle_us, static_cast<double>(kMinPwmUs), static_cast<double>(kMaxPwmUs));

    // X quad layout:
    // m1 front-left, m2 front-right, m3 rear-right, m4 rear-left.
    MotorOutputs outputs;
    outputs.m1 = clampToPwm(throttle + pitch_correction_us + roll_correction_us - yaw_correction_us);
    outputs.m2 = clampToPwm(throttle + pitch_correction_us - roll_correction_us + yaw_correction_us);
    outputs.m3 = clampToPwm(throttle - pitch_correction_us - roll_correction_us - yaw_correction_us);
    outputs.m4 = clampToPwm(throttle - pitch_correction_us + roll_correction_us + yaw_correction_us);
    return outputs;
}

int MotorMixer::clampToPwm(double value) {
    const double clamped = std::clamp(
        value, static_cast<double>(kMinPwmUs), static_cast<double>(kMaxPwmUs));
    return static_cast<int>(std::lround(clamped));
}

}  // namespace drone

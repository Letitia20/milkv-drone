#include "motor_mixer.hpp"

#include <cmath>

namespace drone {

std::array<int, 4> MotorOutputs::asArray() const {
    return {m1, m2, m3, m4};
}

MotorOutputs MotorMixer::mix(double throttle_us,
                             double roll_mix_us,
                             double pitch_mix_us,
                             double yaw_mix_us) const {
    // X quad layout, ESC order: m1 left-front, m2 right-front,
    // m3 left-rear, m4 right-rear.
    MotorOutputs outputs;
    outputs.m1 = static_cast<int>(std::lround(throttle_us + pitch_mix_us + roll_mix_us - yaw_mix_us));
    outputs.m2 = static_cast<int>(std::lround(throttle_us + pitch_mix_us - roll_mix_us + yaw_mix_us));
    outputs.m3 = static_cast<int>(std::lround(throttle_us - pitch_mix_us + roll_mix_us + yaw_mix_us));
    outputs.m4 = static_cast<int>(std::lround(throttle_us - pitch_mix_us - roll_mix_us - yaw_mix_us));
    return outputs;
}

}  // namespace drone

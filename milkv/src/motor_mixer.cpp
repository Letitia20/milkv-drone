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
    // X 型四轴混控公式，电机顺序：
    // m1 左前 = T + P + R - Y
    // m2 右前 = T + P - R + Y
    // m3 左后 = T - P + R + Y
    // m4 右后 = T - P - R - Y
    MotorOutputs outputs;
    outputs.m1 = static_cast<int>(std::lround(throttle_us + pitch_mix_us + roll_mix_us - yaw_mix_us));
    outputs.m2 = static_cast<int>(std::lround(throttle_us + pitch_mix_us - roll_mix_us + yaw_mix_us));
    outputs.m3 = static_cast<int>(std::lround(throttle_us - pitch_mix_us + roll_mix_us + yaw_mix_us));
    outputs.m4 = static_cast<int>(std::lround(throttle_us - pitch_mix_us - roll_mix_us - yaw_mix_us));
    return outputs;
}

}  // namespace drone

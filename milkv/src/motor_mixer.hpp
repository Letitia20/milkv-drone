#pragma once

#include <array>

namespace drone {

struct MotorOutputs {
    int m1 {1000};
    int m2 {1000};
    int m3 {1000};
    int m4 {1000};

    std::array<int, 4> asArray() const;
};

class MotorMixer {
public:
    static constexpr int kMinPwmUs = 1000;
    static constexpr int kMaxPwmUs = 1800;

    MotorOutputs mix(double throttle_us,
                     double roll_mix_us,
                     double pitch_mix_us,
                     double yaw_mix_us) const;
};

}  // namespace drone

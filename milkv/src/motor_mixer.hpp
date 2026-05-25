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
    static constexpr int kMaxPwmUs = 2000;

    MotorOutputs mix(double throttle_us,
                     double roll_correction_us,
                     double pitch_correction_us,
                     double yaw_correction_us) const;

private:
    static int clampToPwm(double value);
};

}  // namespace drone

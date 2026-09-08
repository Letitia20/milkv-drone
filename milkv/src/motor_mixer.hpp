#pragma once

#include <array>

namespace drone {

// 四个电机输出，单位为 ESC PWM 脉宽 us。
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

    // X 型四轴混控输入：
    // throttle_us 为基础油门，roll/pitch/yaw_mix_us 为三个轴的控制修正量。
    MotorOutputs mix(double throttle_us,
                     double roll_mix_us,
                     double pitch_mix_us,
                     double yaw_mix_us) const;
};

}  // namespace drone

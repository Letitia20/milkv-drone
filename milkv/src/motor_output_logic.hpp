#pragma once

#include <array>
#include <string>

namespace drone {

constexpr int kMotorMaxUs = 1800;

struct MotorOutputInput {
    bool armed {false};
    bool rc_valid {false};
    bool failsafe {true};
    bool invalid_imu {false};
    bool mode {false};
    float throttle {0.0f};
    float roll_cmd {0.0f};
    float pitch_cmd {0.0f};
    float yaw_cmd {0.0f};
};

struct MotorOutputResult {
    int motor_max_us {kMotorMaxUs};
    int base_us {1000};
    int roll_mix_us {0};
    int pitch_mix_us {0};
    int yaw_mix_us {0};
    std::array<int, 4> mixer_before_clamp {1000, 1000, 1000, 1000};
    std::array<int, 4> motors_after_clamp {1000, 1000, 1000, 1000};
    std::string motor_output_enabled_reason {"disabled_unknown"};
};

MotorOutputResult computeMotorOutput(const MotorOutputInput& input);

}  // namespace drone

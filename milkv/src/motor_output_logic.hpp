#pragma once

#include <array>
#include <string>

namespace drone {

constexpr int kMotorMaxUs = 1800;

struct MotorOutputInput {
    bool armed {false};
    bool rc_valid {false};
    bool failsafe {true};
    bool mode {false};
    float throttle {0.0f};
    double roll_correction_us {0.0};
    double pitch_correction_us {0.0};
    double yaw_correction_us {0.0};
};

struct MotorOutputResult {
    int base_us {1000};
    std::array<int, 4> mixer_before_clamp {1000, 1000, 1000, 1000};
    std::array<int, 4> motors_after_clamp {1000, 1000, 1000, 1000};
    std::string motor_output_enabled_reason {"disabled_unknown"};
};

MotorOutputResult computeMotorOutput(const MotorOutputInput& input);

}  // namespace drone

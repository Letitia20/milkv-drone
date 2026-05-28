#include "motor_output_logic.hpp"

#include <array>
#include <iostream>
#include <string>

namespace {

bool expectArray(const std::array<int, 4>& actual,
                 const std::array<int, 4>& expected,
                 const std::string& label) {
    if (actual == expected) {
        return true;
    }

    std::cerr << label << " expected [" << expected[0] << ',' << expected[1] << ','
              << expected[2] << ',' << expected[3] << "] got [" << actual[0] << ','
              << actual[1] << ',' << actual[2] << ',' << actual[3] << "]\n";
    return false;
}

}  // namespace

int main() {
    drone::MotorOutputInput input;
    input.armed = true;
    input.rc_valid = true;
    input.failsafe = false;
    input.mode = true;
    input.throttle = 0.39f;

    const drone::MotorOutputResult active = drone::computeMotorOutput(input);
    if (!expectArray(active.motors_after_clamp, {1390, 1390, 1390, 1390},
                     "armed throttle output")) {
        return 1;
    }
    if (active.base_us != 1390) {
        std::cerr << "base_us expected 1390 got " << active.base_us << "\n";
        return 1;
    }
    if (active.motor_output_enabled_reason != "enabled") {
        std::cerr << "enabled reason mismatch: " << active.motor_output_enabled_reason << "\n";
        return 1;
    }

    input.throttle = 1.0f;
    input.roll_correction_us = 250.0;
    input.pitch_correction_us = 250.0;
    const drone::MotorOutputResult max_limited = drone::computeMotorOutput(input);
    if (!expectArray(max_limited.motors_after_clamp, {1800, 1800, 1300, 1800},
                     "motor max clamp output")) {
        return 1;
    }
    if (max_limited.base_us != 1800) {
        std::cerr << "base_us expected 1800 got " << max_limited.base_us << "\n";
        return 1;
    }

    input.roll_correction_us = 0.0;
    input.pitch_correction_us = 0.0;
    input.throttle = 0.0f;
    const drone::MotorOutputResult zero_throttle = drone::computeMotorOutput(input);
    if (!expectArray(zero_throttle.motors_after_clamp, {1000, 1000, 1000, 1000},
                     "zero throttle output")) {
        return 1;
    }
    if (zero_throttle.motor_output_enabled_reason != "disabled_throttle_zero") {
        std::cerr << "zero throttle reason mismatch: "
                  << zero_throttle.motor_output_enabled_reason << "\n";
        return 1;
    }

    input.throttle = 0.39f;
    input.armed = false;
    const drone::MotorOutputResult disarmed = drone::computeMotorOutput(input);
    if (!expectArray(disarmed.motors_after_clamp, {1000, 1000, 1000, 1000},
                     "disarmed output")) {
        return 1;
    }
    if (disarmed.motor_output_enabled_reason != "disabled_not_armed") {
        std::cerr << "disarmed reason mismatch: " << disarmed.motor_output_enabled_reason << "\n";
        return 1;
    }

    input.armed = true;
    input.failsafe = true;
    const drone::MotorOutputResult failsafe = drone::computeMotorOutput(input);
    if (!expectArray(failsafe.motors_after_clamp, {1000, 1000, 1000, 1000},
                     "failsafe output")) {
        return 1;
    }
    if (failsafe.motor_output_enabled_reason != "disabled_failsafe") {
        std::cerr << "failsafe reason mismatch: " << failsafe.motor_output_enabled_reason << "\n";
        return 1;
    }

    return 0;
}

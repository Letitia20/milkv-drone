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
    if (!expectArray(active.motors_after_clamp, {1312, 1312, 1312, 1312},
                     "armed throttle output")) {
        return 1;
    }
    if (active.base_us != 1312) {
        std::cerr << "base_us expected 1312 got " << active.base_us << "\n";
        return 1;
    }
    if (active.motor_output_enabled_reason != "enabled") {
        std::cerr << "enabled reason mismatch: " << active.motor_output_enabled_reason << "\n";
        return 1;
    }

    input.throttle = 0.8f;
    input.roll_cmd = 0.5f;
    input.pitch_cmd = -0.25f;
    input.yaw_cmd = 0.4f;
    const drone::MotorOutputResult manual_mix = drone::computeMotorOutput(input);
    if (manual_mix.base_us != 1640) {
        std::cerr << "manual mix base_us expected 1640 got " << manual_mix.base_us << "\n";
        return 1;
    }
    if (manual_mix.roll_mix_us != 100 || manual_mix.pitch_mix_us != -50 ||
        manual_mix.yaw_mix_us != 60) {
        std::cerr << "manual mix values expected [100,-50,60] got ["
                  << manual_mix.roll_mix_us << ',' << manual_mix.pitch_mix_us << ','
                  << manual_mix.yaw_mix_us << "]\n";
        return 1;
    }
    if (!expectArray(manual_mix.mixer_before_clamp, {1630, 1550, 1850, 1530},
                     "manual mix before clamp")) {
        return 1;
    }
    if (!expectArray(manual_mix.motors_after_clamp, {1630, 1550, 1800, 1530},
                     "manual mix after clamp")) {
        return 1;
    }

    input.throttle = 1.0f;
    input.roll_cmd = 1.0f;
    input.pitch_cmd = 1.0f;
    input.yaw_cmd = 0.0f;
    const drone::MotorOutputResult max_limited = drone::computeMotorOutput(input);
    if (!expectArray(max_limited.motors_after_clamp, {1800, 1800, 1800, 1400},
                     "motor max clamp output")) {
        return 1;
    }
    if (max_limited.base_us != 1800) {
        std::cerr << "base_us expected 1800 got " << max_limited.base_us << "\n";
        return 1;
    }

    input.roll_cmd = 0.0f;
    input.pitch_cmd = 0.0f;
    input.yaw_cmd = 0.0f;
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

    input.failsafe = false;
    input.invalid_imu = true;
    input.throttle = 0.8f;
    input.roll_cmd = 1.0f;
    input.pitch_cmd = 1.0f;
    input.yaw_cmd = 1.0f;
    const drone::MotorOutputResult invalid_imu = drone::computeMotorOutput(input);
    if (!expectArray(invalid_imu.motors_after_clamp, {1000, 1000, 1000, 1000},
                     "invalid imu output")) {
        return 1;
    }
    if (invalid_imu.motor_output_enabled_reason != "disabled_invalid_imu") {
        std::cerr << "invalid imu reason mismatch: "
                  << invalid_imu.motor_output_enabled_reason << "\n";
        return 1;
    }

    return 0;
}

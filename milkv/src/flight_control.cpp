#include "flight_control.hpp"

#include <algorithm>
#include <cmath>

namespace drone {
namespace {

float clampUnit(float value) {
    return std::clamp(value, -1.0f, 1.0f);
}

float clampThrottle(float value) {
    return std::clamp(value, 0.0f, 1.0f);
}

float normStickUs(int value_us) {
    return clampUnit((static_cast<float>(value_us) - 1500.0f) / 500.0f);
}

float normThrottleUs(int value_us) {
    return clampThrottle((static_cast<float>(value_us) - 1000.0f) / 1000.0f);
}

}  // namespace

float applyDeadband(float value, float deadband) {
    const float safe_deadband = std::clamp(deadband, 0.0f, 0.95f);
    const float clamped = clampUnit(value);
    const float magnitude = std::abs(clamped);

    if (magnitude <= safe_deadband) {
        return 0.0f;
    }

    const float rescaled = (magnitude - safe_deadband) / (1.0f - safe_deadband);
    return std::copysign(rescaled, clamped);
}

float applyThrottleCurve(float throttle, const FlightControlConfig& config) {
    const float raw = clampThrottle(throttle);
    const float cutoff = std::clamp(config.throttle_deadband, 0.0f, 0.95f);
    if (raw <= cutoff) {
        return 0.0f;
    }

    // 平方油门曲线：低油门更柔和，高油门仍能达到满输出。
    const float normalized = (raw - cutoff) / (1.0f - cutoff);
    return normalized * normalized;
}

PilotCommand makePilotCommand(int roll_us,
                              int pitch_us,
                              int throttle_us,
                              int yaw_us,
                              const FlightControlConfig& config) {
    PilotCommand command;
    command.roll_stick = applyDeadband(normStickUs(roll_us), config.stick_deadband);
    command.pitch_stick = applyDeadband(normStickUs(pitch_us), config.stick_deadband);
    command.yaw_stick = applyDeadband(normStickUs(yaw_us), config.stick_deadband);
    command.throttle = applyThrottleCurve(normThrottleUs(throttle_us), config);

    // 姿态目标换算：
    // roll_target = roll_stick * max_angle
    // pitch_target = pitch_stick * max_angle
    // yaw_rate_target = yaw_stick * max_yaw_rate
    command.roll_target_deg = command.roll_stick * config.max_angle_deg;
    command.pitch_target_deg = command.pitch_stick * config.max_angle_deg;
    command.yaw_rate_target_dps = command.yaw_stick * config.max_yaw_rate_dps;
    return command;
}

}  // namespace drone

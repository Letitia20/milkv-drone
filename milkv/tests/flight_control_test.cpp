// 测试目标：验证遥控死区、平方油门曲线和目标姿态角换算公式。
#include "flight_control.hpp"

#include <cmath>
#include <iostream>

namespace {

bool expectNear(double actual, double expected, double tolerance, const char* label) {
    if (std::abs(actual - expected) <= tolerance) {
        return true;
    }

    std::cerr << label << " expected " << expected << " got " << actual << "\n";
    return false;
}

}  // namespace

int main() {
    drone::FlightControlConfig config;
    config.stick_deadband = 0.05f;
    config.throttle_deadband = 0.04f;
    config.max_angle_deg = 20.0;
    config.max_yaw_rate_dps = 90.0;

    if (!expectNear(drone::applyDeadband(0.03f, config.stick_deadband), 0.0, 0.0001,
                    "small positive stick deadband")) {
        return 1;
    }
    if (!expectNear(drone::applyDeadband(-0.03f, config.stick_deadband), 0.0, 0.0001,
                    "small negative stick deadband")) {
        return 1;
    }
    if (!expectNear(drone::applyDeadband(0.55f, 0.10f), 0.5, 0.0001,
                    "positive stick rescale")) {
        return 1;
    }
    if (!expectNear(drone::applyDeadband(-0.55f, 0.10f), -0.5, 0.0001,
                    "negative stick rescale")) {
        return 1;
    }

    if (!expectNear(drone::applyThrottleCurve(0.02f, config), 0.0, 0.0001,
                    "throttle cutoff")) {
        return 1;
    }
    const float mid_throttle = drone::applyThrottleCurve(0.50f, config);
    if (!(mid_throttle > 0.0f && mid_throttle < 0.50f)) {
        std::cerr << "mid throttle should be softened below linear, got " << mid_throttle << "\n";
        return 1;
    }
    if (!expectNear(drone::applyThrottleCurve(1.0f, config), 1.0, 0.0001,
                    "full throttle remains full scale")) {
        return 1;
    }

    const drone::PilotCommand centered = drone::makePilotCommand(1510, 1490, 1000, 1508, config);
    if (!expectNear(centered.roll_target_deg, 0.0, 0.0001, "centered roll target") ||
        !expectNear(centered.pitch_target_deg, 0.0, 0.0001, "centered pitch target") ||
        !expectNear(centered.yaw_rate_target_dps, 0.0, 0.0001, "centered yaw target") ||
        !expectNear(centered.throttle, 0.0, 0.0001, "low throttle target")) {
        return 1;
    }

    const drone::PilotCommand full = drone::makePilotCommand(2000, 1000, 2000, 2000, config);
    if (!expectNear(full.roll_target_deg, 20.0, 0.0001, "full roll target") ||
        !expectNear(full.pitch_target_deg, -20.0, 0.0001, "full pitch target") ||
        !expectNear(full.yaw_rate_target_dps, 90.0, 0.0001, "full yaw target") ||
        !expectNear(full.throttle, 1.0, 0.0001, "full throttle target")) {
        return 1;
    }

    return 0;
}

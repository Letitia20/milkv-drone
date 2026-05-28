#pragma once

namespace drone {

struct FlightControlConfig {
    float stick_deadband {0.05f};
    float throttle_deadband {0.04f};
    double max_angle_deg {20.0};
    double max_yaw_rate_dps {90.0};
};

struct PilotCommand {
    float roll_stick {0.0f};
    float pitch_stick {0.0f};
    float throttle {0.0f};
    float yaw_stick {0.0f};
    double roll_target_deg {0.0};
    double pitch_target_deg {0.0};
    double yaw_rate_target_dps {0.0};
};

float applyDeadband(float value, float deadband);
float applyThrottleCurve(float throttle, const FlightControlConfig& config);
PilotCommand makePilotCommand(int roll_us,
                              int pitch_us,
                              int throttle_us,
                              int yaw_us,
                              const FlightControlConfig& config);

}  // namespace drone

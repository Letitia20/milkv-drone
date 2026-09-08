#pragma once

namespace drone {

// 遥控输入到飞控目标的配置。
// stick_deadband 避免中位附近抖动，throttle_deadband 避免电机突然启动。
struct FlightControlConfig {
    float stick_deadband {0.05f};
    float throttle_deadband {0.04f};
    double max_angle_deg {20.0};
    double max_yaw_rate_dps {90.0};
};

// 遥控器四通道被归一化后的飞行指令。
// roll/pitch/yaw stick 范围 [-1, 1]，throttle 范围 [0, 1]。
struct PilotCommand {
    float roll_stick {0.0f};
    float pitch_stick {0.0f};
    float throttle {0.0f};
    float yaw_stick {0.0f};
    double roll_target_deg {0.0};
    double pitch_target_deg {0.0};
    double yaw_rate_target_dps {0.0};
};

// 死区公式：
// abs(x) <= d 时输出 0，否则输出 sign(x) * (abs(x) - d) / (1 - d)。
float applyDeadband(float value, float deadband);

// 油门曲线公式：
// throttle <= d 时输出 0，否则 y = ((throttle - d) / (1 - d))^2。
float applyThrottleCurve(float throttle, const FlightControlConfig& config);
PilotCommand makePilotCommand(int roll_us,
                              int pitch_us,
                              int throttle_us,
                              int yaw_us,
                              const FlightControlConfig& config);

}  // namespace drone

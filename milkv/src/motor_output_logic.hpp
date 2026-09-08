#pragma once

#include <array>
#include <string>

namespace drone {

constexpr int kMotorMaxUs = 1800;
constexpr int kEmergencyStopSwaThresholdUs = 1500;

// SWA/CH5 急停判断：通道值大于 1500us 即进入紧急停桨。
bool isEmergencyStopSwaUs(int swa_us);

// 电机输出安全逻辑的输入状态。所有安全条件集中到这里判断，
// 便于测试急停、未解锁、遥控失效、IMU 无效和低电压降高。
struct MotorOutputInput {
    bool armed {false};
    bool rc_valid {false};
    bool failsafe {true};
    bool invalid_imu {false};
    bool emergency_stop {false};
    bool low_voltage {false};
    bool mode {false};
    float throttle {0.0f};
    float roll_cmd {0.0f};
    float pitch_cmd {0.0f};
    float yaw_cmd {0.0f};
    bool use_pid_corrections {false};
    double roll_correction_us {0.0};
    double pitch_correction_us {0.0};
    double yaw_correction_us {0.0};
};

// 电机输出计算结果：包含最终 PWM，也保留中间量，方便串口日志和调参。
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

// 功能：根据安全状态、油门、摇杆或 PID 修正量计算四路电机 PWM。
MotorOutputResult computeMotorOutput(const MotorOutputInput& input);

}  // namespace drone

#pragma once

#include <array>
#include <string>

namespace drone::protocol {

// 遥控和电调常用 PWM 脉宽范围，单位 us。
constexpr int kPwmMinUs = 1000;
constexpr int kPwmMidUs = 1500;
constexpr int kPwmMaxUs = 2000;

struct RcData {
    std::array<int, 6> channels {kPwmMidUs, kPwmMidUs, kPwmMinUs,
                                 kPwmMidUs, kPwmMinUs, kPwmMinUs};
    bool failsafe {true};
    bool valid {false};
};

// 电池电压，单位 mV。
struct BatteryData {
    int voltage_mv {0};
    bool valid {false};
};

// 文本协议：
// RC,ch1,ch2,ch3,ch4,ch5,ch6,failsafe
// BAT,<millivolts>
bool parseRcLine(const std::string& line, RcData& out);
bool parseBatteryLine(const std::string& line, BatteryData& out);

int clampPwm(int value);

std::string encodeMotLine(const std::array<int, 4>& motors);
std::string encodeArmLine();
std::string encodeDisarmLine();

}  // namespace drone::protocol

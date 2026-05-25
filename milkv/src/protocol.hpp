#pragma once

#include <array>
#include <string>

namespace drone::protocol {

constexpr int kPwmMinUs = 1000;
constexpr int kPwmMidUs = 1500;
constexpr int kPwmMaxUs = 2000;

struct RcData {
    std::array<int, 6> channels {kPwmMidUs, kPwmMidUs, kPwmMinUs,
                                 kPwmMidUs, kPwmMinUs, kPwmMinUs};
    bool failsafe {true};
    bool valid {false};
};

struct BatteryData {
    int voltage_mv {0};
    bool valid {false};
};

bool parseRcLine(const std::string& line, RcData& out);
bool parseBatteryLine(const std::string& line, BatteryData& out);

int clampPwm(int value);

std::string encodeMotLine(const std::array<int, 4>& motors);
std::string encodeArmLine();
std::string encodeDisarmLine();

}  // namespace drone::protocol

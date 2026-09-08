#include "protocol.hpp"

#include <algorithm>
#include <cerrno>
#include <climits>
#include <cstdlib>
#include <sstream>
#include <vector>

namespace drone::protocol {
namespace {

std::string trim(const std::string& text) {
    const auto begin = text.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return {};
    }
    const auto end = text.find_last_not_of(" \t\r\n");
    return text.substr(begin, end - begin + 1);
}

std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> parts;
    std::stringstream ss(line);
    std::string item;
    while (std::getline(ss, item, ',')) {
        parts.push_back(trim(item));
    }
    return parts;
}

bool parseIntStrict(const std::string& text, int& out) {
    const std::string value_text = trim(text);
    if (value_text.empty()) {
        return false;
    }

    errno = 0;
    char* end = nullptr;
    const long value = std::strtol(value_text.c_str(), &end, 10);
    if (errno != 0 || end == value_text.c_str() || *end != '\0') {
        return false;
    }
    if (value < INT_MIN || value > INT_MAX) {
        return false;
    }

    out = static_cast<int>(value);
    return true;
}

}  // namespace

bool parseRcLine(const std::string& line, RcData& out) {
    // RC 输入格式：RC,ch1,ch2,ch3,ch4,ch5,ch6,failsafe
    const auto parts = splitCsv(line);
    if (parts.size() != 8 || parts[0] != "RC") {
        return false;
    }

    RcData parsed;
    for (std::size_t i = 0; i < parsed.channels.size(); ++i) {
        if (!parseIntStrict(parts[i + 1], parsed.channels[i])) {
            return false;
        }
    }

    int failsafe = 1;
    if (!parseIntStrict(parts[7], failsafe)) {
        return false;
    }

    parsed.failsafe = (failsafe != 0);
    parsed.valid = true;
    out = parsed;
    return true;
}

bool parseBatteryLine(const std::string& line, BatteryData& out) {
    // 电池输入格式：BAT,<millivolts>，例如 BAT,9500。
    const auto parts = splitCsv(line);
    if (parts.size() != 2 || parts[0] != "BAT") {
        return false;
    }

    int voltage_mv = 0;
    if (!parseIntStrict(parts[1], voltage_mv) || voltage_mv < 0) {
        return false;
    }

    out.voltage_mv = voltage_mv;
    out.valid = true;
    return true;
}

int clampPwm(int value) {
    return std::clamp(value, kPwmMinUs, kPwmMaxUs);
}

std::string encodeMotLine(const std::array<int, 4>& motors) {
    // 电机输出格式：MOT,m1,m2,m3,m4，四路均会限制在 1000..2000us。
    std::ostringstream oss;
    oss << "MOT";
    for (const int motor : motors) {
        oss << ',' << clampPwm(motor);
    }
    return oss.str();
}

std::string encodeArmLine() {
    return "ARM";
}

std::string encodeDisarmLine() {
    return "DISARM";
}

}  // namespace drone::protocol

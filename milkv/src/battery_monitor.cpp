#include "battery_monitor.hpp"

#include <exception>
#include <fstream>
#include <sstream>
#include <utility>

namespace drone {

BatteryMonitor::BatteryMonitor(std::string source_path)
    : source_path_(std::move(source_path)) {}

void BatteryMonitor::setSourcePath(std::string source_path) {
    source_path_ = std::move(source_path);
}

bool BatteryMonitor::enabled() const {
    return !source_path_.empty();
}

const std::string& BatteryMonitor::lastError() const {
    return last_error_;
}

bool BatteryMonitor::poll(protocol::BatteryData& out) {
    if (!enabled()) {
        last_error_.clear();
        return false;
    }

    std::ifstream in(source_path_);
    if (!in) {
        last_error_ = "cannot open battery source: " + source_path_;
        return false;
    }

    std::ostringstream contents;
    contents << in.rdbuf();
    const std::string text = contents.str();

    // 优先解析遥测格式 BAT,<millivolts>，便于 STM32 或外部采样模块发送。
    protocol::BatteryData parsed;
    if (protocol::parseBatteryLine(text, parsed)) {
        out = parsed;
        last_error_.clear();
        return true;
    }

    // 兼容纯数字格式，方便在 Milk-V 上用文件快速模拟低电压。
    int voltage_mv = 0;
    try {
        std::size_t consumed = 0;
        voltage_mv = std::stoi(text, &consumed);
        while (consumed < text.size() &&
               (text[consumed] == ' ' || text[consumed] == '\t' ||
                text[consumed] == '\r' || text[consumed] == '\n')) {
            ++consumed;
        }
        if (consumed != text.size() || voltage_mv < 0) {
            last_error_ = "invalid battery source value: " + source_path_;
            return false;
        }
    } catch (const std::exception&) {
        last_error_ = "invalid battery source value: " + source_path_;
        return false;
    }

    out.voltage_mv = voltage_mv;
    out.valid = true;
    last_error_.clear();
    return true;
}

}  // namespace drone

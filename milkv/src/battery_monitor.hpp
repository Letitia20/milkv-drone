#pragma once

#include "protocol.hpp"

#include <string>

namespace drone {

// 电池监控输入封装。当前支持两种文本格式：
// 1. BAT,<millivolts>
// 2. 纯数字 millivolts
class BatteryMonitor {
public:
    BatteryMonitor() = default;
    explicit BatteryMonitor(std::string source_path);

    void setSourcePath(std::string source_path);
    bool enabled() const;
    bool poll(protocol::BatteryData& out);

    const std::string& lastError() const;

private:
    std::string source_path_;
    std::string last_error_;
};

}  // namespace drone

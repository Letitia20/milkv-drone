// 测试目标：验证 BAT,<millivolts> 和纯数字电压文件能正确解析，非法输入会被拒绝。
#include "battery_monitor.hpp"

#include <cstdio>
#include <fstream>
#include <iostream>

namespace {

bool writeFile(const char* path, const char* contents) {
    std::ofstream out(path);
    out << contents;
    return static_cast<bool>(out);
}

}  // namespace

int main() {
    const char* valid_path = "build/host-tests/battery_valid.txt";
    const char* invalid_path = "build/host-tests/battery_invalid.txt";
    const char* missing_path = "build/host-tests/battery_missing.txt";

    std::remove(missing_path);

    if (!writeFile(valid_path, "BAT,9500\n")) {
        std::cerr << "failed to write valid battery fixture\n";
        return 1;
    }
    if (!writeFile(invalid_path, "BAT,-1\n")) {
        std::cerr << "failed to write invalid battery fixture\n";
        return 1;
    }

    drone::BatteryMonitor monitor(valid_path);
    drone::protocol::BatteryData battery;
    if (!monitor.poll(battery)) {
        std::cerr << "valid battery file should update battery data: "
                  << monitor.lastError() << "\n";
        return 1;
    }
    if (!battery.valid || battery.voltage_mv != 9500) {
        std::cerr << "expected valid 9500 mV battery reading, got valid="
                  << battery.valid << " voltage=" << battery.voltage_mv << "\n";
        return 1;
    }

    drone::BatteryMonitor invalid_monitor(invalid_path);
    battery = {};
    if (invalid_monitor.poll(battery)) {
        std::cerr << "invalid battery file should not update battery data\n";
        return 1;
    }
    if (battery.valid) {
        std::cerr << "invalid battery file should leave data invalid\n";
        return 1;
    }

    drone::BatteryMonitor missing_monitor(missing_path);
    if (missing_monitor.poll(battery)) {
        std::cerr << "missing battery file should not update battery data\n";
        return 1;
    }

    return 0;
}

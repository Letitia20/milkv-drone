// 测试目标：验证 ESC sysfs PWM 导出、周期和 duty_cycle 写入逻辑。
#include "esc_pwm_sysfs.hpp"

#include <iostream>
#include <string>

int main() {
    const auto& configs = drone::escPwmConfigs();
    if (configs.size() != 4) {
        std::cerr << "Expected four ESC PWM configs\n";
        return 1;
    }

    if (configs[0].esc_id != 1 || std::string(configs[0].gpio) != "GP4" ||
        configs[0].pin != 6 || configs[0].pwm_channel != 5 ||
        configs[0].chip_base != 4 || configs[0].local_channel != 1) {
        std::cerr << "ESC1 mapping is wrong\n";
        return 1;
    }
    if (configs[1].esc_id != 2 || std::string(configs[1].gpio) != "GP5" ||
        configs[1].pin != 7 || configs[1].pwm_channel != 6 ||
        configs[1].chip_base != 4 || configs[1].local_channel != 2) {
        std::cerr << "ESC2 mapping is wrong\n";
        return 1;
    }
    if (configs[2].esc_id != 3 || std::string(configs[2].gpio) != "GP12" ||
        configs[2].pin != 16 || configs[2].pwm_channel != 4 ||
        configs[2].chip_base != 4 || configs[2].local_channel != 0) {
        std::cerr << "ESC3 mapping is wrong\n";
        return 1;
    }
    if (configs[3].esc_id != 4 || std::string(configs[3].gpio) != "GP6" ||
        configs[3].pin != 9 || configs[3].pwm_channel != 9 ||
        configs[3].chip_base != 8 || configs[3].local_channel != 1) {
        std::cerr << "ESC4 mapping is wrong\n";
        return 1;
    }

    if (drone::clampEscPulseUs(900) != 1000 || drone::clampEscPulseUs(1200) != 1200 ||
        drone::clampEscPulseUs(2500) != 2000) {
        std::cerr << "ESC pulse clamp is wrong\n";
        return 1;
    }

    if (drone::escPulseUsToNs(1200) != 1200000) {
        std::cerr << "ESC pulse conversion is wrong\n";
        return 1;
    }

    return 0;
}

// 测试目标：验证低电压蜂鸣器 500ms 周期、250ms 响/250ms 停的节奏公式。
#include "buzzer_gpio.hpp"

#include <chrono>
#include <iostream>

int main() {
    using namespace std::chrono_literals;

    if (drone::lowVoltageBuzzerOn(false, 0ms)) {
        std::cerr << "buzzer should stay off when low voltage is clear\n";
        return 1;
    }
    if (!drone::lowVoltageBuzzerOn(true, 0ms)) {
        std::cerr << "buzzer should turn on at the start of a low-voltage beep period\n";
        return 1;
    }
    if (!drone::lowVoltageBuzzerOn(true, 249ms)) {
        std::cerr << "buzzer should stay on during the first half of the beep period\n";
        return 1;
    }
    if (drone::lowVoltageBuzzerOn(true, 250ms)) {
        std::cerr << "buzzer should turn off during the second half of the beep period\n";
        return 1;
    }
    if (!drone::lowVoltageBuzzerOn(true, 500ms)) {
        std::cerr << "buzzer should repeat every 500 ms while low voltage is latched\n";
        return 1;
    }

    return 0;
}

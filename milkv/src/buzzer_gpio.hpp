#pragma once

#include <chrono>
#include <string>

namespace drone {

// 蜂鸣器节奏函数：
// low_voltage=false 时关闭；
// low_voltage=true 时 500ms 一个周期，前 250ms 响、后 250ms 停。
bool lowVoltageBuzzerOn(bool low_voltage, std::chrono::milliseconds elapsed);

// 通过 sysfs GPIO 控制 GP2 蜂鸣器。
class BuzzerGpio {
public:
    explicit BuzzerGpio(int gpio_number = 2, std::string gpio_root = "/sys/class/gpio");
    ~BuzzerGpio();

    BuzzerGpio(const BuzzerGpio&) = delete;
    BuzzerGpio& operator=(const BuzzerGpio&) = delete;

    bool initialize();
    bool set(bool on);
    void off();

    const std::string& lastError() const;

private:
    std::string gpioPath() const;
    bool writeTextFile(const std::string& path, const std::string& value);
    bool pathExists(const std::string& path) const;

    int gpio_number_;
    std::string gpio_root_;
    bool ready_ {false};
    std::string last_error_;
};

}  // namespace drone

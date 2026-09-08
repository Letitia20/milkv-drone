#include "buzzer_gpio.hpp"

#include <cerrno>
#include <cstring>
#include <fstream>
#include <sys/stat.h>
#include <thread>
#include <utility>

namespace drone {

bool lowVoltageBuzzerOn(bool low_voltage, std::chrono::milliseconds elapsed) {
    if (!low_voltage) {
        return false;
    }
    constexpr auto kPeriod = std::chrono::milliseconds(500);
    constexpr auto kOnTime = std::chrono::milliseconds(250);
    // 周期公式：phase = elapsed_ms % 500，phase < 250 时蜂鸣器打开。
    const auto phase = elapsed.count() % kPeriod.count();
    return phase < kOnTime.count();
}

BuzzerGpio::BuzzerGpio(int gpio_number, std::string gpio_root)
    : gpio_number_(gpio_number), gpio_root_(std::move(gpio_root)) {}

BuzzerGpio::~BuzzerGpio() {
    off();
}

const std::string& BuzzerGpio::lastError() const {
    return last_error_;
}

bool BuzzerGpio::initialize() {
    const std::string path = gpioPath();
    if (!pathExists(path)) {
        if (!writeTextFile(gpio_root_ + "/export", std::to_string(gpio_number_))) {
            return false;
        }
        for (int i = 0; i < 50 && !pathExists(path); ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (!pathExists(path)) {
            last_error_ = "exported GPIO" + std::to_string(gpio_number_) +
                          " but sysfs node did not appear";
            return false;
        }
    }
    if (!writeTextFile(path + "/direction", "out")) {
        return false;
    }
    ready_ = true;
    return set(false);
}

bool BuzzerGpio::set(bool on) {
    if (!ready_) {
        last_error_ = "buzzer GPIO is not initialized";
        return false;
    }
    return writeTextFile(gpioPath() + "/value", on ? "1" : "0");
}

void BuzzerGpio::off() {
    if (ready_) {
        set(false);
    }
}

std::string BuzzerGpio::gpioPath() const {
    return gpio_root_ + "/gpio" + std::to_string(gpio_number_);
}

bool BuzzerGpio::writeTextFile(const std::string& path, const std::string& value) {
    std::ofstream out(path);
    if (!out) {
        last_error_ = "open " + path + " failed: " + std::strerror(errno);
        return false;
    }
    out << value;
    if (!out) {
        last_error_ = "write " + path + " failed: " + std::strerror(errno);
        return false;
    }
    last_error_.clear();
    return true;
}

bool BuzzerGpio::pathExists(const std::string& path) const {
    struct stat st {};
    return ::stat(path.c_str(), &st) == 0;
}

}  // namespace drone

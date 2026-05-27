#include "esc_pwm_sysfs.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fstream>
#include <sys/stat.h>
#include <thread>
#include <utility>

namespace drone {
namespace {

constexpr std::array<EscPwmConfig, 4> kEscPwmConfigs {{
    // ESC1 = left front = GP4 = Pin 6 = global PWM5 = pwmchip4/pwm1
    {1, "left_front", "GP4", 6, 5, kEscPwmLowChipBase, 1},
    // ESC2 = right front = GP5 = Pin 7 = global PWM6 = pwmchip4/pwm2
    {2, "right_front", "GP5", 7, 6, kEscPwmLowChipBase, 2},
    // ESC3 = left rear = GP12 = Pin 16 = global PWM4 = pwmchip4/pwm0
    {3, "left_rear", "GP12", 16, 4, kEscPwmLowChipBase, 0},
    // ESC4 = right rear = GP6 = Pin 9 = global PWM9 = pwmchip8/pwm1
    {4, "right_rear", "GP6", 9, 9, kEscPwmHighChipBase, 1},
}};

bool pathExists(const std::string& path) {
    struct stat st {};
    return ::stat(path.c_str(), &st) == 0;
}

std::string joinPath(const std::string& a, const std::string& b) {
    if (a.empty() || a.back() == '/') {
        return a + b;
    }
    return a + "/" + b;
}

std::string pwmChipNameFromBase(int base) {
    return "pwmchip" + std::to_string(base);
}

bool pathLooksLikeChipBase(const std::string& chip_path, int chip_base) {
    const std::string expected = pwmChipNameFromBase(chip_base);
    return chip_path == expected ||
           (chip_path.size() >= expected.size() &&
            chip_path.compare(chip_path.size() - expected.size(), expected.size(), expected) == 0);
}

}  // namespace

const std::array<EscPwmConfig, 4>& escPwmConfigs() {
    return kEscPwmConfigs;
}

int clampEscPulseUs(int pulse_us) {
    return std::clamp(pulse_us, kEscPwmMinUs, kEscPwmMaxUs);
}

int escPulseUsToNs(int pulse_us) {
    return clampEscPulseUs(pulse_us) * 1000;
}

EscPwmSysfs::EscPwmSysfs(std::string pwm_root) : pwm_root_(std::move(pwm_root)) {}

EscPwmSysfs::~EscPwmSysfs() {
    stopAll();
    disableAll();
}

const std::string& EscPwmSysfs::lastError() const {
    return last_error_;
}

bool EscPwmSysfs::initialize(const std::string& forced_chip) {
    for (std::size_t i = 0; i < channels_.size(); ++i) {
        channels_[i].config = kEscPwmConfigs[i];
        channels_[i].chip_path = findPwmChip(channels_[i].config.chip_base, forced_chip);
        if (channels_[i].chip_path.empty()) {
            return false;
        }
        if (!configureChannel(channels_[i])) {
            return false;
        }
    }

    return stopAll();
}

bool EscPwmSysfs::setEscUs(int esc_id, int pulse_us) {
    if (esc_id < 1 || esc_id > static_cast<int>(channels_.size())) {
        last_error_ = "ESC id must be 1..4";
        return false;
    }

    ChannelState& channel = channels_[static_cast<std::size_t>(esc_id - 1)];
    if (!channel.ready) {
        last_error_ = "ESC" + std::to_string(esc_id) + " PWM is not initialized";
        return false;
    }

    return writeIntFile(joinPath(channel.pwm_path, "duty_cycle"), escPulseUsToNs(pulse_us));
}

bool EscPwmSysfs::stopAll() {
    bool ok = true;
    for (const auto& config : kEscPwmConfigs) {
        ok = setEscUs(config.esc_id, kEscPwmMinUs) && ok;
    }
    return ok;
}

void EscPwmSysfs::disableAll() {
    for (auto& channel : channels_) {
        if (!channel.ready || channel.pwm_path.empty()) {
            continue;
        }
        writeIntFile(joinPath(channel.pwm_path, "enable"), 0);
        channel.ready = false;
    }
}

bool EscPwmSysfs::configureChannel(ChannelState& channel) {
    if (!exportChannel(channel)) {
        return false;
    }

    writeIntFile(joinPath(channel.pwm_path, "enable"), 0);

    const std::string polarity_path = joinPath(channel.pwm_path, "polarity");
    if (pathExists(polarity_path)) {
        writeTextFile(polarity_path, "normal");
    }

    if (!writeIntFile(joinPath(channel.pwm_path, "period"), kEscPwmPeriodNs)) {
        return false;
    }
    if (!writeIntFile(joinPath(channel.pwm_path, "duty_cycle"), escPulseUsToNs(kEscPwmMinUs))) {
        return false;
    }
    if (!writeIntFile(joinPath(channel.pwm_path, "enable"), 1)) {
        return false;
    }

    channel.ready = true;
    return true;
}

bool EscPwmSysfs::exportChannel(ChannelState& channel) {
    channel.pwm_path = joinPath(channel.chip_path,
                                "pwm" + std::to_string(channel.config.local_channel));
    if (pathExists(channel.pwm_path)) {
        return true;
    }

    if (!writeIntFile(joinPath(channel.chip_path, "export"), channel.config.local_channel)) {
        return false;
    }

    for (int i = 0; i < 50; ++i) {
        if (pathExists(channel.pwm_path)) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    last_error_ = "exported global PWM" + std::to_string(channel.config.pwm_channel) +
                  " as local pwm" + std::to_string(channel.config.local_channel) +
                  " but sysfs node did not appear";
    return false;
}

std::string EscPwmSysfs::findPwmChip(int chip_base, const std::string& forced_chip) {
    if (!forced_chip.empty() && pathLooksLikeChipBase(forced_chip, chip_base)) {
        const std::string chip_path = forced_chip.rfind("/", 0) == 0
            ? forced_chip
            : joinPath(pwm_root_, forced_chip);
        if (!chipHasRequiredChannels(chip_path)) {
            last_error_ = "forced PWM chip must expose local pwm0..pwm3: " + chip_path;
            return {};
        }
        return chip_path;
    }

    const std::string expected_chip = joinPath(pwm_root_, pwmChipNameFromBase(chip_base));
    if (chipHasRequiredChannels(expected_chip)) {
        return expected_chip;
    }

    last_error_ = "no " + pwmChipNameFromBase(chip_base) +
                  " under " + pwm_root_ + " exposes local pwm0..pwm3";
    return {};
}

bool EscPwmSysfs::chipHasRequiredChannels(const std::string& chip_path) {
    int npwm = 0;
    return readIntFile(joinPath(chip_path, "npwm"), npwm) && npwm >= 4;
}

bool EscPwmSysfs::writeTextFile(const std::string& path, const std::string& value) {
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
    return true;
}

bool EscPwmSysfs::writeIntFile(const std::string& path, int value) {
    return writeTextFile(path, std::to_string(value));
}

bool EscPwmSysfs::readIntFile(const std::string& path, int& value) {
    std::ifstream in(path);
    if (!in) {
        last_error_ = "open " + path + " failed: " + std::strerror(errno);
        return false;
    }
    in >> value;
    if (!in) {
        last_error_ = "read " + path + " failed: " + std::strerror(errno);
        return false;
    }
    return true;
}

}  // namespace drone

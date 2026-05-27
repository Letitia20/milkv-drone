#pragma once

#include <array>
#include <string>

namespace drone {

constexpr int kEscPwmPeriodNs = 20000000;
constexpr int kEscPwmMinUs = 1000;
constexpr int kEscPwmMaxUs = 2000;
constexpr int kEscPwmLowChipBase = 4;
constexpr int kEscPwmHighChipBase = 8;

struct EscPwmConfig {
    int esc_id;
    const char* position;
    const char* gpio;
    int pin;
    int pwm_channel;
    int chip_base;
    int local_channel;
};

const std::array<EscPwmConfig, 4>& escPwmConfigs();

int clampEscPulseUs(int pulse_us);
int escPulseUsToNs(int pulse_us);

class EscPwmSysfs {
public:
    explicit EscPwmSysfs(std::string pwm_root = "/sys/class/pwm");
    ~EscPwmSysfs();

    EscPwmSysfs(const EscPwmSysfs&) = delete;
    EscPwmSysfs& operator=(const EscPwmSysfs&) = delete;

    bool initialize(const std::string& forced_chip = {});
    bool setEscUs(int esc_id, int pulse_us);
    bool stopAll();
    void disableAll();

    const std::string& lastError() const;

private:
    struct ChannelState {
        EscPwmConfig config {};
        std::string chip_path;
        std::string pwm_path;
        bool ready {false};
    };

    bool configureChannel(ChannelState& channel);
    bool exportChannel(ChannelState& channel);
    std::string findPwmChip(int chip_base, const std::string& forced_chip);
    bool chipHasRequiredChannels(const std::string& chip_path);
    bool writeTextFile(const std::string& path, const std::string& value);
    bool writeIntFile(const std::string& path, int value);
    bool readIntFile(const std::string& path, int& value);

    std::string pwm_root_;
    std::array<ChannelState, 4> channels_ {};
    std::string last_error_;
};

}  // namespace drone

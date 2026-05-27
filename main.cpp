#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <vector>

namespace {

constexpr int kPeriodNs = 20'000'000;       // 50 Hz = 20 ms
constexpr int kLowThrottleUs = 1000;        // 1.0 ms
constexpr int kMidThrottleUs = 1500;        // 1.5 ms
constexpr int kHighThrottleUs = 2000;       // 2.0 ms
constexpr int kDefaultTestPulseUs = 1150;   // Safe spin test pulse
constexpr int kMaxDefaultTestPulseUs = 1200;

constexpr const char* kSysfsPwmRoot = "/sys/class/pwm";

std::atomic<bool> g_running {true};

struct EscConfig {
    int esc_id;
    const char* position;
    const char* gpio;
    int pin;
    int pwm_channel;
    const char* prop_note;
};

// Motor mapping is intentionally fixed. Do not reuse PWM5 for another pin.
constexpr std::array<EscConfig, 4> kEscConfigs {{
    // ESC1 = 左上 = GP4  = Pin 6  = PWM5 = 左上黑色反牙
    {1, "左上", "GP4", 6, 5, "黑色反牙"},
    // ESC2 = 右上 = GP5  = Pin 7  = PWM6 = 右上白色正牙
    {2, "右上", "GP5", 7, 6, "白色正牙"},
    // ESC3 = 左下 = GP12 = Pin 16 = PWM4 = 左下白色正牙
    {3, "左下", "GP12", 16, 4, "白色正牙"},
    // ESC4 = 右下 = GP2  = Pin 4  = PWM7 = 右下黑色反牙
    {4, "右下", "GP2", 4, 7, "黑色反牙"},
}};

struct PwmChannel {
    EscConfig config {};
    std::string chip_path;
    std::string pwm_path;
    bool exported_here {false};
    bool ready {false};
};

std::array<PwmChannel, 4> g_pwm_channels {};
std::string g_forced_chip;
int g_test_pulse_us = kDefaultTestPulseUs;

bool path_exists(const std::string& path) {
    struct stat st {};
    return ::stat(path.c_str(), &st) == 0;
}

std::string join_path(const std::string& a, const std::string& b) {
    if (a.empty() || a.back() == '/') {
        return a + b;
    }
    return a + "/" + b;
}

std::string read_text_file(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("cannot read " + path + ": " + std::strerror(errno));
    }

    std::string value;
    std::getline(in, value);
    return value;
}

int read_int_file(const std::string& path) {
    return std::stoi(read_text_file(path));
}

void write_text_file(const std::string& path, const std::string& value) {
    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("cannot open " + path + ": " + std::strerror(errno));
    }

    out << value;
    if (!out) {
        throw std::runtime_error("cannot write " + path + ": " + std::strerror(errno));
    }
}

void write_int_file(const std::string& path, int value) {
    write_text_file(path, std::to_string(value));
}

std::vector<std::string> list_pwmchips() {
    std::vector<std::string> chips;
    DIR* dir = ::opendir(kSysfsPwmRoot);
    if (!dir) {
        throw std::runtime_error(std::string("cannot open ") + kSysfsPwmRoot +
                                 ": " + std::strerror(errno));
    }

    while (dirent* ent = ::readdir(dir)) {
        const std::string name = ent->d_name;
        if (name.rfind("pwmchip", 0) == 0) {
            chips.push_back(join_path(kSysfsPwmRoot, name));
        }
    }

    ::closedir(dir);
    std::sort(chips.begin(), chips.end());
    return chips;
}

bool chip_has_required_channels(const std::string& chip_path) {
    try {
        const int npwm = read_int_file(join_path(chip_path, "npwm"));
        return npwm > 7;
    } catch (const std::exception&) {
        return false;
    }
}

std::string find_pwm_chip_path() {
    if (!g_forced_chip.empty()) {
        std::string chip = g_forced_chip;
        if (chip.rfind("/sys/", 0) != 0) {
            chip = join_path(kSysfsPwmRoot, chip);
        }
        if (!path_exists(chip)) {
            throw std::runtime_error("forced PWM chip does not exist: " + chip);
        }
        if (!chip_has_required_channels(chip)) {
            throw std::runtime_error("forced PWM chip has fewer than 8 channels: " + chip);
        }
        return chip;
    }

    const auto chips = list_pwmchips();
    for (const auto& chip : chips) {
        if (chip_has_required_channels(chip)) {
            return chip;
        }
    }

    std::cerr << "No pwmchip with PWM4/PWM5/PWM6/PWM7 was found under "
              << kSysfsPwmRoot << ".\n"
              << "Try: ls -l /sys/class/pwm && cat /sys/class/pwm/pwmchip*/npwm\n"
              << "If the chip number is known, run with --chip pwmchipX.\n";
    throw std::runtime_error("cannot find suitable pwmchip");
}

void export_pwm_if_needed(PwmChannel& ch) {
    ch.pwm_path = join_path(ch.chip_path, "pwm" + std::to_string(ch.config.pwm_channel));
    if (path_exists(ch.pwm_path)) {
        return;
    }

    write_int_file(join_path(ch.chip_path, "export"), ch.config.pwm_channel);
    ch.exported_here = true;

    for (int i = 0; i < 50; ++i) {
        if (path_exists(ch.pwm_path)) {
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    throw std::runtime_error("export succeeded but PWM path did not appear: " + ch.pwm_path);
}

void configure_pwm_channel(PwmChannel& ch) {
    export_pwm_if_needed(ch);

    const std::string enable = join_path(ch.pwm_path, "enable");
    const std::string period = join_path(ch.pwm_path, "period");
    const std::string duty = join_path(ch.pwm_path, "duty_cycle");
    const std::string polarity = join_path(ch.pwm_path, "polarity");

    if (path_exists(enable)) {
        try {
            write_int_file(enable, 0);
        } catch (const std::exception&) {
            // Some drivers reject disabling an already-disabled channel; continue to configure.
        }
    }

    if (path_exists(polarity)) {
        try {
            write_text_file(polarity, "normal");
        } catch (const std::exception&) {
            // Polarity is optional in sysfs PWM.
        }
    }

    write_int_file(period, kPeriodNs);
    write_int_file(duty, kLowThrottleUs * 1000);
    write_int_file(enable, 1);
    ch.ready = true;
}

void initialize_pwm_outputs() {
    const std::string chip = find_pwm_chip_path();
    std::cout << "Using PWM chip: " << chip << "\n";

    for (std::size_t i = 0; i < kEscConfigs.size(); ++i) {
        g_pwm_channels[i].config = kEscConfigs[i];
        g_pwm_channels[i].chip_path = chip;
        configure_pwm_channel(g_pwm_channels[i]);
    }
}

void disable_all_pwm() {
    for (auto& ch : g_pwm_channels) {
        if (!ch.ready || ch.pwm_path.empty()) {
            continue;
        }

        try {
            write_int_file(join_path(ch.pwm_path, "enable"), 0);
        } catch (const std::exception& e) {
            std::cerr << "Warning: failed to disable ESC" << ch.config.esc_id
                      << " PWM" << ch.config.pwm_channel << ": " << e.what() << "\n";
        }
        ch.ready = false;
    }
}

void sleep_checked(std::chrono::milliseconds duration) {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (g_running && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void print_mapping() {
    std::cout << "Fixed ESC mapping:\n";
    for (const auto& esc : kEscConfigs) {
        std::cout << "  ESC" << esc.esc_id << " = " << esc.position
                  << " = " << esc.gpio
                  << " = Pin " << esc.pin
                  << " = PWM" << esc.pwm_channel
                  << " = " << esc.prop_note << "\n";
    }
    std::cout << "Do not use GP26/GP27 for ESC signals; they are 1.8V pins.\n";
    std::cout << "Do not map GP4 and GP13 to PWM5 at the same time.\n";
}

void handle_signal(int) {
    g_running = false;
}

class CleanupGuard {
public:
    ~CleanupGuard();
};

void usage(const char* argv0) {
    std::cout
        << "Usage: sudo " << argv0 << " [--chip pwmchipX] [--pulse-us 1100|1150|1200]\n\n"
        << "The program arms all ESCs at 1000us for 5 seconds, then tests ESC1..ESC4 one at a time.\n"
        << "Default test pulse is " << kDefaultTestPulseUs << "us and is capped at "
        << kMaxDefaultTestPulseUs << "us.\n";
}

void parse_args(int argc, char* argv[]) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--chip" && i + 1 < argc) {
            g_forced_chip = argv[++i];
        } else if (arg == "--pulse-us" && i + 1 < argc) {
            g_test_pulse_us = std::stoi(argv[++i]);
        } else if (arg == "--help" || arg == "-h") {
            usage(argv[0]);
            std::exit(0);
        } else {
            throw std::runtime_error("unknown or incomplete argument: " + arg);
        }
    }
}

}  // namespace

[[maybe_unused]] const std::array<EscConfig, 4>& esc_configs() {
    return kEscConfigs;
}

int us_to_ns(int pulse_us) {
    return pulse_us * 1000;
}

int clamp_esc_pulse_us(int pulse_us) {
    return std::max(kLowThrottleUs, std::min(kHighThrottleUs, pulse_us));
}

int clamp_test_pulse_us(int pulse_us) {
    return std::max(kLowThrottleUs, std::min(kMaxDefaultTestPulseUs, pulse_us));
}

void set_esc_us(int esc_id, int pulse_us) {
    if (esc_id < 1 || esc_id > static_cast<int>(g_pwm_channels.size())) {
        throw std::runtime_error("ESC id must be 1..4");
    }

    PwmChannel& ch = g_pwm_channels[static_cast<std::size_t>(esc_id - 1)];
    if (!ch.ready) {
        throw std::runtime_error("ESC" + std::to_string(esc_id) + " PWM is not initialized");
    }

    const int safe_us = clamp_esc_pulse_us(pulse_us);
    write_int_file(join_path(ch.pwm_path, "duty_cycle"), us_to_ns(safe_us));
}

void arm_all_esc() {
    std::cout << "Arming all ESCs at " << kLowThrottleUs << "us for 5 seconds...\n";
    for (const auto& esc : kEscConfigs) {
        set_esc_us(esc.esc_id, kLowThrottleUs);
    }
    sleep_checked(std::chrono::seconds(5));
}

void stop_all_esc() {
    for (const auto& esc : kEscConfigs) {
        try {
            set_esc_us(esc.esc_id, kLowThrottleUs);
        } catch (const std::exception& e) {
            std::cerr << "Warning: failed to set ESC" << esc.esc_id
                      << " to low throttle: " << e.what() << "\n";
        }
    }
}

void test_each_motor() {
    const int test_us = clamp_test_pulse_us(g_test_pulse_us);
    if (test_us != g_test_pulse_us) {
        std::cout << "Requested test pulse " << g_test_pulse_us
                  << "us was capped to " << test_us << "us.\n";
    }

    std::cout << "Safety test: all motors stay at " << kLowThrottleUs
              << "us except the one currently being tested.\n";

    stop_all_esc();
    sleep_checked(std::chrono::milliseconds(500));

    for (const auto& esc : kEscConfigs) {
        if (!g_running) {
            break;
        }

        std::cout << "Testing ESC" << esc.esc_id << " (" << esc.position
                  << ", " << esc.gpio << "/Pin " << esc.pin
                  << "/PWM" << esc.pwm_channel << ") at " << test_us
                  << "us for 2 seconds...\n";

        stop_all_esc();
        set_esc_us(esc.esc_id, test_us);
        sleep_checked(std::chrono::seconds(2));
        set_esc_us(esc.esc_id, kLowThrottleUs);
        sleep_checked(std::chrono::milliseconds(700));
    }

    stop_all_esc();
}

CleanupGuard::~CleanupGuard() {
    std::cout << "Stopping all ESCs at " << kLowThrottleUs << "us and disabling PWM...\n";
    stop_all_esc();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    disable_all_pwm();
}

#ifndef ESC_PWM_UNIT_TEST
int main(int argc, char* argv[]) {
    try {
        parse_args(argc, argv);

        std::signal(SIGINT, handle_signal);
        std::signal(SIGTERM, handle_signal);

        std::cout << "Milk-V Duo / Duo256M four-ESC sysfs PWM test\n";
        std::cout << "PWM: 50Hz, period=" << kPeriodNs << "ns, low="
                  << kLowThrottleUs << "us, mid=" << kMidThrottleUs
                  << "us, high=" << kHighThrottleUs << "us\n";
        std::cout << "REMOVE ALL PROPELLERS before running this test.\n";
        print_mapping();

        CleanupGuard cleanup;
        initialize_pwm_outputs();

        arm_all_esc();
        if (g_running) {
            test_each_motor();
        }

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        std::cerr << "Make sure PWM sysfs is enabled, pinmux is set to PWM, and run with sudo/root.\n";
        return 1;
    }
}
#endif

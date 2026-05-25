#include "imu_mpu6050.hpp"
#include "loop_rate.hpp"

#include <atomic>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

std::atomic<bool> g_running {true};

void handleSignal(int) {
    g_running = false;
}

double parseRateHz(const char* text, double fallback) {
    if (text == nullptr) {
        return fallback;
    }

    char* end = nullptr;
    const double value = std::strtod(text, &end);
    if (end == text || *end != '\0' || value <= 0.0) {
        return fallback;
    }
    return value;
}

bool parseI2cAddress(const char* text, std::uint8_t& address) {
    if (text == nullptr) {
        return false;
    }

    char* end = nullptr;
    const unsigned long value = std::strtoul(text, &end, 0);
    if (end == text || *end != '\0' || value > std::numeric_limits<std::uint8_t>::max()) {
        return false;
    }

    address = static_cast<std::uint8_t>(value);
    return drone::isAllowedMpu6050Address(address);
}

}  // namespace

int main(int argc, char* argv[]) {
    std::printf("mpu_test version: 2026-05-25-v2 (verbose init)\n");
    std::printf("Compiled: %s %s\n", __DATE__, __TIME__);

    const std::string i2c_device = (argc >= 2) ? argv[1] : drone::kMpu6050DefaultI2cDevice;
    std::vector<std::uint8_t> addresses(
        drone::kMpu6050AllowedAddresses.begin(), drone::kMpu6050AllowedAddresses.end());
    double rate_hz = 20.0;

    if (argc >= 3) {
        std::uint8_t parsed_address = 0;
        if (parseI2cAddress(argv[2], parsed_address)) {
            addresses = {parsed_address};
            rate_hz = (argc >= 4) ? parseRateHz(argv[3], rate_hz) : rate_hz;
        } else {
            rate_hz = parseRateHz(argv[2], rate_hz);
        }
    }

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    drone::Mpu6050 imu;
    std::uint8_t active_address = 0;
    for (const std::uint8_t address : addresses) {
        if (imu.initialize(i2c_device, address)) {
            active_address = address;
            break;
        }
    }

    if (active_address == 0) {
        std::cerr << "Failed to initialize MPU6050 on " << i2c_device << " at address";
        for (const std::uint8_t address : addresses) {
            std::cerr << " 0x" << std::hex << static_cast<int>(address);
        }
        std::cerr << std::dec << "\n";
        return 1;
    }

    drone::LoopRate loop_rate(rate_hz);
    while (g_running) {
        const drone::ImuSample sample = imu.read();
        if (!sample.valid) {
            std::cerr << "Failed to read MPU6050 sample\n";
            return 1;
        }

        std::cout << std::fixed << std::setprecision(6) << sample.ax_g << ' ' << sample.ay_g << ' '
                  << sample.az_g << ' ' << sample.gx_dps << ' ' << sample.gy_dps << ' '
                  << sample.gz_dps << '\n';

        loop_rate.sleep();
    }

    return 0;
}

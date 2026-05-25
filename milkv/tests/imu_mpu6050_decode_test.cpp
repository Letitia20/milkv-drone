#include "imu_mpu6050.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>

namespace {

bool near(double actual, double expected, double tolerance) {
    return std::fabs(actual - expected) <= tolerance;
}

void put16(std::array<std::uint8_t, 14>& data, int offset, std::int16_t value) {
    const auto raw = static_cast<std::uint16_t>(value);
    data[static_cast<std::size_t>(offset)] = static_cast<std::uint8_t>((raw >> 8) & 0xff);
    data[static_cast<std::size_t>(offset + 1)] = static_cast<std::uint8_t>(raw & 0xff);
}

}  // namespace

int main() {
    std::array<std::uint8_t, 14> data {};
    put16(data, 0, 16384);
    put16(data, 2, -16384);
    put16(data, 4, 0);
    put16(data, 6, 0);
    put16(data, 8, 131);
    put16(data, 10, -262);
    put16(data, 12, 0);

    const drone::ImuSample sample = drone::decodeMpu6050Registers(data);

    if (!sample.valid || !near(sample.ax_g, 1.0, 0.0001) || !near(sample.ay_g, -1.0, 0.0001) ||
        !near(sample.az_g, 0.0, 0.0001) || !near(sample.temperature_c, 36.53, 0.001) ||
        !near(sample.gx_dps, 1.0, 0.0001) || !near(sample.gy_dps, -2.0, 0.0001) ||
        !near(sample.gz_dps, 0.0, 0.0001)) {
        std::cerr << "MPU6050 decode produced unexpected values\n";
        return 1;
    }

    return 0;
}

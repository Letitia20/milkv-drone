// 测试目标：验证互补滤波器对有效/无效 IMU 数据的 valid 状态处理。
#include "complementary_filter.hpp"

#include <iostream>

int main() {
    drone::ComplementaryFilter filter;

    drone::ImuSample invalid_sample;
    invalid_sample.valid = false;
    if (filter.update(invalid_sample, 0.01).valid) {
        std::cerr << "Invalid IMU sample should produce invalid attitude\n";
        return 1;
    }

    drone::ImuSample valid_sample;
    valid_sample.valid = true;
    valid_sample.az_g = 1.0;
    if (!filter.update(valid_sample, 0.01).valid) {
        std::cerr << "Valid IMU sample should produce valid attitude\n";
        return 1;
    }

    if (filter.update(valid_sample, 0.0).valid) {
        std::cerr << "Zero dt should produce invalid attitude\n";
        return 1;
    }

    return 0;
}

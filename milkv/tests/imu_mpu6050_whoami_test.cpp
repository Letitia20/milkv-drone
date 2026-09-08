// 测试目标：验证 MPU WHO_AM_I 返回值只接受实测 0x70。
#include "imu_mpu6050.hpp"

#include <iostream>

int main() {
    if (!drone::isSupportedMpu6050WhoAmI(0x70)) {
        std::cerr << "Expected WHO_AM_I support for observed 0x70\n";
        return 1;
    }

    if (drone::isSupportedMpu6050WhoAmI(0x68) || drone::isSupportedMpu6050WhoAmI(0x69) ||
        drone::isSupportedMpu6050WhoAmI(0x71)) {
        std::cerr << "Unexpected WHO_AM_I value accepted\n";
        return 1;
    }

    return 0;
}

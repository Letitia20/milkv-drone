// 测试目标：验证 MPU6050 允许地址列表，只接受 0x68/0x69。
#include "imu_mpu6050.hpp"

#include <iostream>

int main() {
    if (!drone::isAllowedMpu6050Address(0x68) || !drone::isAllowedMpu6050Address(0x69)) {
        std::cerr << "Expected MPU6050 I2C address allow-list to include 0x68 and 0x69\n";
        return 1;
    }

    if (drone::isAllowedMpu6050Address(0x70) || drone::isAllowedMpu6050Address(0x71)) {
        std::cerr << "Unexpected non-I2C-address accepted as MPU6050 address\n";
        return 1;
    }

    return 0;
}

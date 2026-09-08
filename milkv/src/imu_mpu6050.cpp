#include "imu_mpu6050.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>

#if defined(__linux__)
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/i2c-dev.h>
#endif

namespace drone {
namespace {

constexpr std::uint8_t kRegAccelXoutH = 0x3b;
constexpr std::uint8_t kRegPwrMgmt1 = 0x6b;
constexpr std::uint8_t kRegSmplrtDiv = 0x19;
constexpr std::uint8_t kRegConfig = 0x1a;
constexpr std::uint8_t kRegGyroConfig = 0x1b;
constexpr std::uint8_t kRegAccelConfig = 0x1c;
constexpr std::uint8_t kRegWhoAmI = 0x75;

std::int16_t readBe16(const std::array<std::uint8_t, 14>& data, std::size_t offset) {
    // MPU6050 寄存器是大端序：raw = high << 8 | low。
    const auto high = static_cast<std::uint16_t>(data[offset]);
    const auto low = static_cast<std::uint16_t>(data[offset + 1]);
    return static_cast<std::int16_t>((high << 8) | low);
}

}  // namespace

ImuSample decodeMpu6050Registers(const std::array<std::uint8_t, 14>& data) {
    ImuSample sample;
    sample.valid = true;
    // 原始值换算公式：
    // accel_g = raw / 16384.0（+-2g 量程）
    // gyro_dps = raw / 131.0（+-250 deg/s 量程）
    // temp_c = raw / 340.0 + 36.53
    sample.ax_g = static_cast<double>(readBe16(data, 0)) / 16384.0;
    sample.ay_g = static_cast<double>(readBe16(data, 2)) / 16384.0;
    sample.az_g = static_cast<double>(readBe16(data, 4)) / 16384.0;
    sample.temperature_c = static_cast<double>(readBe16(data, 6)) / 340.0 + 36.53;
    sample.gx_dps = static_cast<double>(readBe16(data, 8)) / 131.0;
    sample.gy_dps = static_cast<double>(readBe16(data, 10)) / 131.0;
    sample.gz_dps = static_cast<double>(readBe16(data, 12)) / 131.0;
    return sample;
}

bool isAllowedMpu6050Address(std::uint8_t address) {
    for (const std::uint8_t allowed_address : kMpu6050AllowedAddresses) {
        if (address == allowed_address) {
            return true;
        }
    }
    return false;
}

bool isSupportedMpu6050WhoAmI(std::uint8_t who_am_i) {
    return who_am_i == kMpu6050ExpectedWhoAmI;
}

Mpu6050::~Mpu6050() {
    closeDevice();
}

bool Mpu6050::initialize(const std::string& i2c_device, std::uint8_t address) {
    std::printf("[IMU] initialize: i2c=%s addr=0x%02X\n", i2c_device.c_str(), address);

    if (!isAllowedMpu6050Address(address)) {
        std::printf("[IMU] FAIL: address 0x%02X not in allowed list\n", address);
        return false;
    }
    std::printf("[IMU] address 0x%02X is allowed\n", address);

    closeDevice();
    i2c_device_ = i2c_device;
    address_ = address;
    initialized_ = false;

#if defined(__linux__)
    fd_ = ::open(i2c_device_.c_str(), O_RDWR);
    if (fd_ < 0) {
        std::printf("[IMU] FAIL: open(%s) failed, errno=%d (%s)\n",
                    i2c_device_.c_str(), errno, std::strerror(errno));
        return false;
    }
    std::printf("[IMU] open(%s) success, fd=%d\n", i2c_device_.c_str(), fd_);

    if (::ioctl(fd_, I2C_SLAVE, address_) < 0) {
        std::printf("[IMU] FAIL: ioctl(I2C_SLAVE, 0x%02X) failed, errno=%d (%s)\n",
                    address_, errno, std::strerror(errno));
        closeDevice();
        return false;
    }
    std::printf("[IMU] ioctl(I2C_SLAVE, 0x%02X) success\n", address_);

    {
        std::uint8_t who_am_i = 0;
        if (!readRegisters(kRegWhoAmI, &who_am_i, 1)) {
            std::printf("[IMU] FAIL: readRegisters(WHO_AM_I=0x%02X) failed, errno=%d (%s)\n",
                        kRegWhoAmI, errno, std::strerror(errno));
            closeDevice();
            return false;
        }

        std::printf("[IMU] WHO_AM_I = 0x%02X (expected 0x%02X)\n",
                    who_am_i, kMpu6050ExpectedWhoAmI);

        if (!isSupportedMpu6050WhoAmI(who_am_i)) {
            std::printf("[IMU] FAIL: WHO_AM_I 0x%02X not supported\n", who_am_i);
            closeDevice();
            return false;
        }
        std::printf("[IMU] WHO_AM_I 0x%02X accepted\n", who_am_i);
    }

    {
        bool regs_ok = true;

        if (!writeRegister(kRegPwrMgmt1, 0x00)) {
            std::printf("[IMU] FAIL: writeRegister(PWR_MGMT_1=0x%02X, 0x00) failed, errno=%d (%s)\n",
                        kRegPwrMgmt1, errno, std::strerror(errno));
            regs_ok = false;
        } else {
            std::printf("[IMU] writeRegister(PWR_MGMT_1=0x%02X, 0x00) success\n", kRegPwrMgmt1);
        }

        // 采样率配置：SampleRate = GyroOutputRate / (1 + SMPLRT_DIV)。
        // 这里写 0x07，配合 1kHz gyro 输出得到约 125Hz，满足作业 >=100Hz 要求。
        if (regs_ok && !writeRegister(kRegSmplrtDiv, 0x07)) {
            std::printf("[IMU] FAIL: writeRegister(SMPLRT_DIV=0x%02X, 0x07) failed, errno=%d (%s)\n",
                        kRegSmplrtDiv, errno, std::strerror(errno));
            regs_ok = false;
        } else if (regs_ok) {
            std::printf("[IMU] writeRegister(SMPLRT_DIV=0x%02X, 0x07) success\n", kRegSmplrtDiv);
        }

        if (regs_ok && !writeRegister(kRegConfig, 0x03)) {
            std::printf("[IMU] FAIL: writeRegister(CONFIG=0x%02X, 0x03) failed, errno=%d (%s)\n",
                        kRegConfig, errno, std::strerror(errno));
            regs_ok = false;
        } else if (regs_ok) {
            std::printf("[IMU] writeRegister(CONFIG=0x%02X, 0x03) success\n", kRegConfig);
        }

        if (regs_ok && !writeRegister(kRegGyroConfig, 0x00)) {
            std::printf("[IMU] FAIL: writeRegister(GYRO_CONFIG=0x%02X, 0x00) failed, errno=%d (%s)\n",
                        kRegGyroConfig, errno, std::strerror(errno));
            regs_ok = false;
        } else if (regs_ok) {
            std::printf("[IMU] writeRegister(GYRO_CONFIG=0x%02X, 0x00) success\n", kRegGyroConfig);
        }

        if (regs_ok && !writeRegister(kRegAccelConfig, 0x00)) {
            std::printf("[IMU] FAIL: writeRegister(ACCEL_CONFIG=0x%02X, 0x00) failed, errno=%d (%s)\n",
                        kRegAccelConfig, errno, std::strerror(errno));
            regs_ok = false;
        } else if (regs_ok) {
            std::printf("[IMU] writeRegister(ACCEL_CONFIG=0x%02X, 0x00) success\n", kRegAccelConfig);
        }

        if (!regs_ok) {
            closeDevice();
            return false;
        }
    }

    std::printf("[IMU] initialize SUCCESS\n");
    initialized_ = true;
    return true;
#else
    std::printf("[IMU] FAIL: not on Linux\n");
    return false;
#endif
}

bool Mpu6050::initializeAnyAddress(const std::string& i2c_device) {
    for (const std::uint8_t address : kMpu6050AllowedAddresses) {
        if (initialize(i2c_device, address)) {
            return true;
        }
    }
    return false;
}

ImuSample Mpu6050::read() {
    if (!initialized_ || !stub_valid_) {
        return {};
    }

#if defined(__linux__)
    std::array<std::uint8_t, 14> data {};
    if (!readRegisters(kRegAccelXoutH, data.data(), data.size())) {
        initialized_ = false;
        closeDevice();
        return {};
    }

    return decodeMpu6050Registers(data);
#else
    return {};
#endif
}

bool Mpu6050::isValid() const {
    return initialized_ && stub_valid_;
}

void Mpu6050::setStubValid(bool valid) {
    stub_valid_ = valid;
}

bool Mpu6050::writeRegister(std::uint8_t reg, std::uint8_t value) {
#if defined(__linux__)
    const std::uint8_t data[2] {reg, value};
    return ::write(fd_, data, sizeof(data)) == static_cast<ssize_t>(sizeof(data));
#else
    (void)reg;
    (void)value;
    return false;
#endif
}

bool Mpu6050::readRegisters(std::uint8_t start_reg, std::uint8_t* data, std::size_t length) {
#if defined(__linux__)
    if (::write(fd_, &start_reg, 1) != 1) {
        return false;
    }

    return ::read(fd_, data, length) == static_cast<ssize_t>(length);
#else
    (void)start_reg;
    (void)data;
    (void)length;
    return false;
#endif
}

void Mpu6050::closeDevice() {
#if defined(__linux__)
    if (fd_ >= 0) {
        ::close(fd_);
    }
#endif
    fd_ = -1;
}

}  // namespace drone

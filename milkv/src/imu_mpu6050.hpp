#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace drone {

constexpr const char* kMpu6050DefaultI2cDevice = "/dev/i2c-2";
constexpr std::uint8_t kMpu6050DefaultAddress = 0x68;
constexpr std::uint8_t kMpu6050AlternateAddress = 0x69;
constexpr std::uint8_t kMpu6050ExpectedWhoAmI = 0x68;
constexpr std::uint8_t kMpu6050ObservedWhoAmI = 0x70;
constexpr std::array<std::uint8_t, 2> kMpu6050AllowedAddresses {
    kMpu6050DefaultAddress,
    kMpu6050AlternateAddress,
};

struct ImuSample {
    bool valid {false};

    double ax_g {0.0};
    double ay_g {0.0};
    double az_g {1.0};

    double gx_dps {0.0};
    double gy_dps {0.0};
    double gz_dps {0.0};

    double temperature_c {25.0};
};

ImuSample decodeMpu6050Registers(const std::array<std::uint8_t, 14>& data);
bool isAllowedMpu6050Address(std::uint8_t address);
bool isSupportedMpu6050WhoAmI(std::uint8_t who_am_i);

class Mpu6050 {
public:
    Mpu6050() = default;
    ~Mpu6050();

    Mpu6050(const Mpu6050&) = delete;
    Mpu6050& operator=(const Mpu6050&) = delete;

    bool initialize(const std::string& i2c_device = kMpu6050DefaultI2cDevice,
                    std::uint8_t address = kMpu6050DefaultAddress);
    bool initializeAnyAddress(const std::string& i2c_device = kMpu6050DefaultI2cDevice);

    ImuSample read();
    bool isValid() const;

    void setStubValid(bool valid);

private:
    bool writeRegister(std::uint8_t reg, std::uint8_t value);
    bool readRegisters(std::uint8_t start_reg, std::uint8_t* data, std::size_t length);
    void closeDevice();

    std::string i2c_device_ {kMpu6050DefaultI2cDevice};
    std::uint8_t address_ {kMpu6050DefaultAddress};
    int fd_ {-1};
    bool initialized_ {false};
    bool stub_valid_ {true};
};

}  // namespace drone

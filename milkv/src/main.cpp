#include "complementary_filter.hpp"
#include "imu_mpu6050.hpp"
#include "loop_rate.hpp"
#include "motor_mixer.hpp"
#include "protocol.hpp"
#include "serial.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

namespace {

std::atomic<bool> g_running {true};

void handleSignal(int) {
    g_running = false;
}

constexpr double kMainLoopHz = 100.0;
constexpr int kLowVoltageMv = 9600;
constexpr int kThrottleLowUs = 1100;
constexpr int kArmSwitchHighUs = 1800;
constexpr int kArmSwitchLowUs = 1200;
constexpr auto kRcTimeout = std::chrono::milliseconds(500);
constexpr auto kHeartbeatPeriod = std::chrono::seconds(1);
constexpr auto kAttitudeLogPeriod = std::chrono::milliseconds(100);
constexpr int kGyroCalibrationSamples = 200;
constexpr auto kGyroCalibrationSamplePeriod = std::chrono::milliseconds(10);

struct GyroBias {
    double gx_dps {0.0};
    double gy_dps {0.0};
    double gz_dps {0.0};
};

bool rcChannelsSane(const drone::protocol::RcData& rc) {
    if (!rc.valid) {
        return false;
    }

    for (const int channel : rc.channels) {
        if (channel < 900 || channel > 2100) {
            return false;
        }
    }
    return true;
}

int parseBaudrate(const char* text, int fallback) {
    if (text == nullptr) {
        return fallback;
    }

    char* end = nullptr;
    const long value = std::strtol(text, &end, 10);
    if (end == text || *end != '\0' || value <= 0) {
        return fallback;
    }
    return static_cast<int>(value);
}

void sendSafeStop(drone::SerialPort& serial) {
    const std::array<int, 4> stop_motors {
        drone::protocol::kPwmMinUs,
        drone::protocol::kPwmMinUs,
        drone::protocol::kPwmMinUs,
        drone::protocol::kPwmMinUs,
    };

    serial.writeLine(drone::protocol::encodeDisarmLine());
    serial.writeLine(drone::protocol::encodeMotLine(stop_motors));
}

GyroBias calibrateGyroBias(drone::Mpu6050& imu) {
    GyroBias bias;
    int valid_samples = 0;

    std::cout << "Keep the board still, calibrating gyro...\n";

    for (int i = 0; i < kGyroCalibrationSamples && g_running; ++i) {
        const drone::ImuSample sample = imu.read();
        if (sample.valid) {
            bias.gx_dps += sample.gx_dps;
            bias.gy_dps += sample.gy_dps;
            bias.gz_dps += sample.gz_dps;
            ++valid_samples;
        }

        std::this_thread::sleep_for(kGyroCalibrationSamplePeriod);
    }

    if (valid_samples > 0) {
        bias.gx_dps /= valid_samples;
        bias.gy_dps /= valid_samples;
        bias.gz_dps /= valid_samples;
    }

    std::cout << std::fixed << std::setprecision(6)
              << "Gyro bias gx=" << bias.gx_dps
              << " gy=" << bias.gy_dps
              << " gz=" << bias.gz_dps
              << " samples=" << valid_samples << "/" << kGyroCalibrationSamples << "\n";

    return bias;
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <uart_device> [baudrate]\n"
                  << "Example: " << argv[0] << " /dev/ttyS1 115200\n";
        return 2;
    }

    const std::string uart_device = argv[1];
    const int baudrate = (argc >= 3) ? parseBaudrate(argv[2], 115200) : 115200;

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    drone::SerialPort serial;
    if (!serial.open(uart_device, baudrate)) {
        std::cerr << "Failed to open UART: " << serial.lastError() << "\n";
        return 1;
    }

    drone::Mpu6050 imu;
    bool imu_initialized = imu.initialize(drone::kMpu6050DefaultI2cDevice, drone::kMpu6050DefaultAddress);
    if (!imu_initialized) {
        imu_initialized = imu.initializeAnyAddress();
    }

    GyroBias gyro_bias;
    if (imu_initialized) {
        gyro_bias = calibrateGyroBias(imu);
    } else {
        std::cerr << "IMU initialization failed; staying disarmed\n";
    }

    drone::ComplementaryFilter filter;
    drone::LoopRate loop_rate(kMainLoopHz);

    drone::protocol::RcData rc;
    drone::protocol::BatteryData battery;
    auto last_rc_time = std::chrono::steady_clock::time_point::min();

    bool armed = false;
    bool low_voltage_latched = false;

    auto last_loop_time = std::chrono::steady_clock::now();
    auto next_heartbeat = last_loop_time + kHeartbeatPeriod;
    auto next_attitude_log = last_loop_time + kAttitudeLogPeriod;
    auto next_serial_warning = last_loop_time;

    std::cout << "Milk-V Duo 256 drone controller skeleton started at " << kMainLoopHz << " Hz\n"
              << "Safety notice: bench-test without propellers installed. Milk-V sends UART targets only;\n"
              << "STM32 must own ESC PWM generation, timeout failsafe, and emergency motor stop.\n";

    while (g_running) {
        const auto now = std::chrono::steady_clock::now();

        for (const auto& line : serial.readLines()) {
            drone::protocol::RcData parsed_rc;
            drone::protocol::BatteryData parsed_battery;

            if (drone::protocol::parseRcLine(line, parsed_rc)) {
                rc = parsed_rc;
                last_rc_time = now;
            } else if (drone::protocol::parseBatteryLine(line, parsed_battery)) {
                battery = parsed_battery;
            }
        }

        double dt_s = std::chrono::duration<double>(now - last_loop_time).count();
        if (dt_s <= 0.0) {
            dt_s = loop_rate.periodSeconds();
        }
        last_loop_time = now;

        drone::ImuSample imu_sample = imu.read();
        if (imu_sample.valid) {
            imu_sample.gx_dps -= gyro_bias.gx_dps;
            imu_sample.gy_dps -= gyro_bias.gy_dps;
            imu_sample.gz_dps -= gyro_bias.gz_dps;
        }
        const drone::Attitude attitude = filter.update(imu_sample, dt_s);
        const bool invalid_imu_safety = (!imu.isValid()) || (!imu_sample.valid) || (!attitude.valid);

        const bool have_rc_time = (last_rc_time != std::chrono::steady_clock::time_point::min());
        const auto rc_age = have_rc_time ? std::chrono::duration_cast<std::chrono::milliseconds>(
                                               now - last_rc_time)
                                         : std::chrono::hours(24);
        const bool rc_fresh = rcChannelsSane(rc) && rc_age <= kRcTimeout;
        const bool rc_failsafe = (!rc_fresh) || rc.failsafe;

        if (battery.valid && battery.voltage_mv < kLowVoltageMv) {
            low_voltage_latched = true;
        }

        const bool battery_ok = battery.valid && !low_voltage_latched && battery.voltage_mv >= kLowVoltageMv;
        const bool imu_valid = !invalid_imu_safety;
        const bool throttle_low = rc_fresh && rc.channels[2] <= kThrottleLowUs;

        // Channel 5 is treated as the arm switch for this skeleton.
        const bool arm_requested = rc_fresh && rc.channels[4] >= kArmSwitchHighUs;
        const bool disarm_switch = rc_fresh && rc.channels[4] <= kArmSwitchLowUs;

        if (rc_failsafe || !battery_ok || invalid_imu_safety || disarm_switch) {
            armed = false;
        } else if (!armed && arm_requested && throttle_low) {
            armed = true;
        }

        // Version 1 intentionally never maps throttle/PID output to motors.
        // This keeps the UART command path testable without spinning motors.
        const std::array<int, 4> motors {
            drone::protocol::kPwmMinUs,
            drone::protocol::kPwmMinUs,
            drone::protocol::kPwmMinUs,
            drone::protocol::kPwmMinUs,
        };

        bool write_ok = true;
        if (armed) {
            write_ok = serial.writeLine(drone::protocol::encodeArmLine()) && write_ok;
        } else {
            write_ok = serial.writeLine(drone::protocol::encodeDisarmLine()) && write_ok;
        }
        write_ok = serial.writeLine(drone::protocol::encodeMotLine(motors)) && write_ok;

        if (!write_ok && now >= next_serial_warning) {
            std::cerr << "UART write warning: " << serial.lastError() << "\n";
            next_serial_warning = now + kHeartbeatPeriod;
        }

        if (now >= next_attitude_log) {
            std::cout << std::fixed << std::setprecision(3)
                      << "attitude"
                      << " roll_deg=" << attitude.roll_deg
                      << " pitch_deg=" << attitude.pitch_deg
                      << " yaw_deg=" << attitude.yaw_deg
                      << " ax_g=" << imu_sample.ax_g
                      << " ay_g=" << imu_sample.ay_g
                      << " az_g=" << imu_sample.az_g
                      << " gx_dps=" << imu_sample.gx_dps
                      << " gy_dps=" << imu_sample.gy_dps
                      << " gz_dps=" << imu_sample.gz_dps
                      << " invalid_imu=" << (invalid_imu_safety ? 1 : 0) << "\n";
            next_attitude_log += kAttitudeLogPeriod;
        }

        if (now >= next_heartbeat) {
            std::cout << std::fixed << std::setprecision(2)
                      << "heartbeat"
                      << " armed=" << (armed ? 1 : 0)
                      << " rc_valid=" << (rc_fresh ? 1 : 0)
                      << " failsafe=" << (rc_failsafe ? 1 : 0)
                      << " bat_mv=" << battery.voltage_mv
                      << " low_voltage=" << (low_voltage_latched ? 1 : 0)
                      << " imu_valid=" << (imu_valid ? 1 : 0)
                      << " invalid_imu=" << (invalid_imu_safety ? 1 : 0)
                      << " roll=" << attitude.roll_deg
                      << " pitch=" << attitude.pitch_deg
                      << " yaw=" << attitude.yaw_deg
                      << " motors=[" << motors[0] << ',' << motors[1] << ',' << motors[2] << ','
                      << motors[3] << "]\n";
            next_heartbeat += kHeartbeatPeriod;
        }

        loop_rate.sleep();
    }

    for (int i = 0; i < 3; ++i) {
        sendSafeStop(serial);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    return 0;
}

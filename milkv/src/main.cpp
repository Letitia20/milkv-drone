#include "complementary_filter.hpp"
#include "esc_pwm_sysfs.hpp"
#include "ibus_receiver.hpp"
#include "imu_mpu6050.hpp"
#include "loop_rate.hpp"
#include "motor_output_logic.hpp"
#include "pid.hpp"
#include "protocol.hpp"

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
constexpr int kSwitchThresholdUs = 1500;
constexpr auto kRcTimeout = std::chrono::milliseconds(200);
constexpr auto kHeartbeatPeriod = std::chrono::seconds(1);
constexpr auto kAttitudeLogPeriod = std::chrono::milliseconds(100);  // 10 Hz
constexpr int kGyroCalibrationSamples = 200;
constexpr auto kGyroCalibrationSamplePeriod = std::chrono::milliseconds(10);
constexpr const char* kDefaultIbusDevice = "/dev/ttyS1";

// PID bench test gains (estimate for 250mm quad, tune before flight)
// These map attitude error (deg) → motor correction (us)
constexpr double kTestPidRollKp = 20.0;
constexpr double kTestPidRollKi = 8.0;
constexpr double kTestPidRollKd = 5.0;
constexpr double kTestPidPitchKp = 20.0;
constexpr double kTestPidPitchKi = 8.0;
constexpr double kTestPidPitchKd = 5.0;
constexpr double kTestPidYawKp = 15.0;   // yaw is rate-mode: deg/s → us
constexpr double kTestPidYawKi = 5.0;
constexpr double kTestPidYawKd = 3.0;
constexpr double kPidIntegralLimit = 200.0;
constexpr double kPidOutputLimit = 400.0;

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
        if (channel < drone::ibus::kMinChannelValue || channel > drone::ibus::kMaxChannelValue) {
            return false;
        }
    }
    return true;
}

float normStick(std::uint16_t value) {
    float x = (static_cast<float>(value) - 1500.0f) / 500.0f;
    if (x > 1.0f) {
        x = 1.0f;
    }
    if (x < -1.0f) {
        x = -1.0f;
    }
    return x;
}

float normThrottle(std::uint16_t value) {
    float x = (static_cast<float>(value) - 1000.0f) / 1000.0f;
    if (x > 1.0f) {
        x = 1.0f;
    }
    if (x < 0.0f) {
        x = 0.0f;
    }
    return x;
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

bool writeMotorsToEsc(drone::EscPwmSysfs& esc_pwm, const std::array<int, 4>& motors) {
    bool ok = true;
    ok = esc_pwm.setEscUs(1, motors[0]) && ok;
    ok = esc_pwm.setEscUs(2, motors[1]) && ok;
    ok = esc_pwm.setEscUs(3, motors[2]) && ok;
    ok = esc_pwm.setEscUs(4, motors[3]) && ok;
    return ok;
}

void sleepWhileRunning(std::chrono::milliseconds duration) {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (g_running && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

}  // namespace

int main(int argc, char* argv[]) {
    std::string ibus_device = kDefaultIbusDevice;
    std::string pwm_chip;
    bool test_pid = false;
    bool ibus_device_set = false;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--test-pid") {
            test_pid = true;
        } else if (std::string(argv[i]) == "--pwm-chip" && i + 1 < argc) {
            pwm_chip = argv[++i];
        } else if (!ibus_device_set) {
            ibus_device = argv[i];
            ibus_device_set = true;
        }
    }

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    drone::IBusReceiver ibus_receiver;
    if (!ibus_receiver.open(ibus_device.c_str())) {
        std::cerr << "Failed to open iBUS UART " << ibus_device << ": " << ibus_receiver.lastError() << "\n";
        return 1;
    }

    drone::EscPwmSysfs esc_pwm;
    if (!esc_pwm.initialize(pwm_chip)) {
        std::cerr << "Failed to initialize ESC sysfs PWM output: " << esc_pwm.lastError() << "\n";
        std::cerr << "Run as root; expected pwmchip4 for ESC1..ESC3 and pwmchip8 for ESC4 PWM9.\n";
        return 1;
    }
    std::cout << "ESC sysfs PWM output initialized at 50Hz; all ESCs held at "
              << drone::kEscPwmMinUs << "us\n";
    std::cout << "Holding ESCs at low throttle for 5 seconds to arm...\n";
    sleepWhileRunning(std::chrono::seconds(5));
    if (!g_running) {
        esc_pwm.stopAll();
        esc_pwm.disableAll();
        ibus_receiver.close();
        return 0;
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

    // PID controllers for bench testing
    drone::PID pid_roll(kTestPidRollKp, kTestPidRollKi, kTestPidRollKd);
    pid_roll.setIntegralLimit(kPidIntegralLimit);
    pid_roll.setOutputLimits(-kPidOutputLimit, kPidOutputLimit);
    drone::PID pid_pitch(kTestPidPitchKp, kTestPidPitchKi, kTestPidPitchKd);
    pid_pitch.setIntegralLimit(kPidIntegralLimit);
    pid_pitch.setOutputLimits(-kPidOutputLimit, kPidOutputLimit);
    drone::PID pid_yaw(kTestPidYawKp, kTestPidYawKi, kTestPidYawKd);
    pid_yaw.setIntegralLimit(kPidIntegralLimit);
    pid_yaw.setOutputLimits(-kPidOutputLimit, kPidOutputLimit);
    drone::protocol::RcData rc;
    drone::protocol::BatteryData battery;
    auto last_rc_time = std::chrono::steady_clock::time_point::min();

    bool armed = false;
    bool low_voltage_latched = false;

    auto last_loop_time = std::chrono::steady_clock::now();
    auto next_heartbeat = last_loop_time + kHeartbeatPeriod;
    auto next_attitude_log = last_loop_time + kAttitudeLogPeriod;

    std::cout << "Milk-V Duo 256 drone controller skeleton started at " << kMainLoopHz << " Hz\n"
              << "iBUS input: " << ibus_device << " (GP3/UART1_RX, 115200 8N1 raw)\n"
              << "Safety notice: bench-test without propellers installed. RC failsafe disarms within 200 ms.\n";

    if (test_pid) {
        std::cout << "=== PID TEST MODE ===\n"
                  << "Target: roll=0 deg, pitch=0 deg, yaw rate=0 dps\n"
                  << "Gains: roll kp=" << kTestPidRollKp << " ki=" << kTestPidRollKi
                  << " kd=" << kTestPidRollKd << "\n"
                  << "       pitch kp=" << kTestPidPitchKp << " ki=" << kTestPidPitchKi
                  << " kd=" << kTestPidPitchKd << "\n"
                  << "       yaw kp=" << kTestPidYawKp << " ki=" << kTestPidYawKi
                  << " kd=" << kTestPidYawKd << "\n"
                  << "Motors LOCKED at " << drone::protocol::kPwmMinUs << "us (safe).\n"
                  << "Tilt the board to see PID corrections.\n";
    }

    while (g_running) {
        const auto now = std::chrono::steady_clock::now();

        drone::IBusChannels ibus_channels;
        while (ibus_receiver.readFrame(ibus_channels)) {
            for (std::size_t i = 0; i < rc.channels.size(); ++i) {
                rc.channels[i] = static_cast<int>(ibus_channels.ch[i]);
            }
            rc.failsafe = false;
            rc.valid = true;
            last_rc_time = now;
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

        const bool battery_ok = !low_voltage_latched && (!battery.valid || battery.voltage_mv >= kLowVoltageMv);
        const bool imu_valid = !invalid_imu_safety;
        const bool throttle_low = rc_fresh && rc.channels[2] <= kThrottleLowUs;

        float roll_cmd = rc_fresh ? normStick(static_cast<std::uint16_t>(rc.channels[0])) : 0.0f;
        float pitch_cmd = rc_fresh ? normStick(static_cast<std::uint16_t>(rc.channels[1])) : 0.0f;
        float throttle_cmd = rc_fresh ? normThrottle(static_cast<std::uint16_t>(rc.channels[2])) : 0.0f;
        float yaw_cmd = rc_fresh ? normStick(static_cast<std::uint16_t>(rc.channels[3])) : 0.0f;
        const bool armed_switch = rc_fresh && rc.channels[4] > kSwitchThresholdUs;
        const bool mode_switch = rc_fresh && rc.channels[5] > kSwitchThresholdUs;

        if (rc_failsafe || !battery_ok || invalid_imu_safety || !armed_switch) {
            armed = false;
            throttle_cmd = 0.0f;
            roll_cmd = 0.0f;
            pitch_cmd = 0.0f;
            yaw_cmd = 0.0f;
        } else if (!armed && armed_switch && throttle_low) {
            armed = true;
        }

        // PID test mode: compute corrections from attitude error
        double pid_roll_out = 0.0;
        double pid_pitch_out = 0.0;
        double pid_yaw_out = 0.0;
        if (test_pid && imu_valid) {
            pid_roll_out = pid_roll.update(0.0, attitude.roll_deg, dt_s);
            pid_pitch_out = pid_pitch.update(0.0, attitude.pitch_deg, dt_s);
            pid_yaw_out = pid_yaw.update(0.0, imu_sample.gz_dps, dt_s);
        }

        drone::MotorOutputInput motor_input;
        motor_input.armed = armed;
        motor_input.rc_valid = rc_fresh;
        motor_input.failsafe = rc_failsafe;
        motor_input.mode = mode_switch;
        motor_input.throttle = throttle_cmd;
        motor_input.roll_correction_us = pid_roll_out;
        motor_input.pitch_correction_us = pid_pitch_out;
        motor_input.yaw_correction_us = pid_yaw_out;
        const drone::MotorOutputResult motor_output = drone::computeMotorOutput(motor_input);
        const std::array<int, 4> motors = motor_output.motors_after_clamp;

        if (!writeMotorsToEsc(esc_pwm, motors)) {
            std::cerr << "ESC PWM write failed: " << esc_pwm.lastError() << "\n";
            esc_pwm.stopAll();
            g_running = false;
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
                      << " invalid_imu=" << (invalid_imu_safety ? 1 : 0)
                      << " pid_r=" << pid_roll_out
                      << " pid_p=" << pid_pitch_out
                      << " pid_y=" << pid_yaw_out
                      << " rc_cmd=[" << roll_cmd << ',' << pitch_cmd << ','
                      << throttle_cmd << ',' << yaw_cmd << ']'
                      << " base_us=" << motor_output.base_us
                      << " mixer_before_clamp=[" << motor_output.mixer_before_clamp[0] << ','
                      << motor_output.mixer_before_clamp[1] << ','
                      << motor_output.mixer_before_clamp[2] << ','
                      << motor_output.mixer_before_clamp[3] << ']'
                      << " motors_after_clamp=[" << motor_output.motors_after_clamp[0] << ','
                      << motor_output.motors_after_clamp[1] << ','
                      << motor_output.motors_after_clamp[2] << ','
                      << motor_output.motors_after_clamp[3] << ']'
                      << " motor_output_enabled_reason="
                      << motor_output.motor_output_enabled_reason << "\n";
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
                      << " rc_ch=[" << rc.channels[0] << ',' << rc.channels[1] << ','
                      << rc.channels[2] << ',' << rc.channels[3] << ','
                      << rc.channels[4] << ',' << rc.channels[5] << ']'
                      << " cmd=[" << roll_cmd << ',' << pitch_cmd << ','
                      << throttle_cmd << ',' << yaw_cmd << ']'
                      << " mode=" << (mode_switch ? 1 : 0)
                      << " throttle=" << throttle_cmd
                      << " base_us=" << motor_output.base_us
                      << " mixer_before_clamp=[" << motor_output.mixer_before_clamp[0] << ','
                      << motor_output.mixer_before_clamp[1] << ','
                      << motor_output.mixer_before_clamp[2] << ','
                      << motor_output.mixer_before_clamp[3] << ']'
                      << " motors_after_clamp=[" << motor_output.motors_after_clamp[0] << ','
                      << motor_output.motors_after_clamp[1] << ','
                      << motor_output.motors_after_clamp[2] << ','
                      << motor_output.motors_after_clamp[3] << ']'
                      << " motor_output_enabled_reason="
                      << motor_output.motor_output_enabled_reason
                      << " roll=" << attitude.roll_deg
                      << " pitch=" << attitude.pitch_deg
                      << " yaw=" << attitude.yaw_deg
                      << " motors=[" << motors[0] << ',' << motors[1] << ',' << motors[2] << ','
                      << motors[3] << "]\n";
            next_heartbeat += kHeartbeatPeriod;
        }

        loop_rate.sleep();
    }

    esc_pwm.stopAll();
    esc_pwm.disableAll();
    ibus_receiver.close();

    return 0;
}

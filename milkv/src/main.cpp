#include "battery_monitor.hpp"
#include "bluetooth_telemetry.hpp"
#include "buzzer_gpio.hpp"
#include "complementary_filter.hpp"
#include "esc_pwm_sysfs.hpp"
#include "flight_control.hpp"
#include "ibus_receiver.hpp"
#include "imu_mpu6050.hpp"
#include "loop_rate.hpp"
#include "motor_output_logic.hpp"
#include "pid.hpp"
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

// 主飞控进程功能：
// 1. 100Hz 读取 iBUS、IMU、电池；
// 2. 互补滤波解算姿态；
// 3. PID 计算三轴修正；
// 4. 混控后输出四路 ESC PWM；
// 5. 低电压时 GP2 蜂鸣器报警，并限制到 1200us 受控降高；
// 6. 通过 HC-05 蓝牙发送 10Hz 遥测。
void handleSignal(int) {
    g_running = false;
}

constexpr double kMainLoopHz = 100.0;
constexpr int kLowVoltageMv = 9600;
constexpr int kThrottleLowUs = 1100;
constexpr int kSwitchThresholdUs = 1500;
constexpr int kSwaChannelIndex = 4;
constexpr int kArmSwitchChannelIndex = 5;
constexpr auto kRcTimeout = std::chrono::milliseconds(200);
constexpr auto kHeartbeatPeriod = std::chrono::seconds(1);
constexpr auto kAttitudeLogPeriod = std::chrono::milliseconds(100);  // 10 Hz
constexpr auto kBluetoothTelemetryPeriod = std::chrono::milliseconds(100);
constexpr auto kBatteryPollPeriod = std::chrono::milliseconds(500);
constexpr int kGyroCalibrationSamples = 200;
constexpr auto kGyroCalibrationSamplePeriod = std::chrono::milliseconds(10);
constexpr const char* kDefaultIbusDevice = "/dev/ttyS1";
constexpr const char* kDefaultBluetoothDevice = "/dev/ttyS0";
constexpr int kDefaultBluetoothBaudrate = 9600;

// PID bench test gains (estimate for 250mm quad, tune before flight).
// These map attitude error (deg) to motor correction (us).
constexpr double kTestPidRollKp = 20.0;
constexpr double kTestPidRollKi = 8.0;
constexpr double kTestPidRollKd = 5.0;
constexpr double kTestPidPitchKp = 20.0;
constexpr double kTestPidPitchKi = 8.0;
constexpr double kTestPidPitchKd = 5.0;
constexpr double kTestPidYawKp = 15.0;   // yaw is rate-mode: deg/s to us.
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

GyroBias calibrateGyroBias(drone::Mpu6050& imu) {
    GyroBias bias;
    int valid_samples = 0;

    std::cout << "Keep the board still, calibrating gyro...\n";

    // 陀螺零偏公式：bias = sum(gyro_sample) / N。
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
    // 解析启动参数：允许临时指定 iBUS、PWM、电池文件和蓝牙串口。
    std::string ibus_device = kDefaultIbusDevice;
    std::string pwm_chip;
    std::string battery_source;
    std::string bluetooth_device = kDefaultBluetoothDevice;
    int bluetooth_baudrate = kDefaultBluetoothBaudrate;
    bool bluetooth_enabled = true;
    bool test_pid = false;
    bool ibus_device_set = false;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--test-pid") {
            test_pid = true;
        } else if (std::string(argv[i]) == "--pwm-chip" && i + 1 < argc) {
            pwm_chip = argv[++i];
        } else if (std::string(argv[i]) == "--battery-file" && i + 1 < argc) {
            battery_source = argv[++i];
        } else if (std::string(argv[i]) == "--bluetooth-device" && i + 1 < argc) {
            bluetooth_device = argv[++i];
        } else if (std::string(argv[i]) == "--bluetooth-baud" && i + 1 < argc) {
            bluetooth_baudrate = std::atoi(argv[++i]);
        } else if (std::string(argv[i]) == "--no-bluetooth") {
            bluetooth_enabled = false;
        } else if (!ibus_device_set) {
            ibus_device = argv[i];
            ibus_device_set = true;
        }
    }

    // 注册退出信号，保证 Ctrl-C 或系统停止时能先停电机再退出。
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    // 打开遥控器 iBUS 输入；这是解锁和飞行指令的主要来源。
    drone::IBusReceiver ibus_receiver;
    if (!ibus_receiver.open(ibus_device.c_str())) {
        std::cerr << "Failed to open iBUS UART " << ibus_device << ": " << ibus_receiver.lastError() << "\n";
        return 1;
    }

    // 初始化四路 ESC PWM；失败时直接退出，避免进入不可控状态。
    drone::EscPwmSysfs esc_pwm;
    if (!esc_pwm.initialize(pwm_chip)) {
        std::cerr << "Failed to initialize ESC sysfs PWM output: " << esc_pwm.lastError() << "\n";
        std::cerr << "Run as root; expected pwmchip4 for ESC1..ESC3 and pwmchip8 for ESC4 PWM9.\n";
        return 1;
    }
    std::cout << "ESC sysfs PWM output initialized at 50Hz; all ESCs held at "
              << drone::kEscPwmMinUs << "us; motor_max_us=" << drone::kMotorMaxUs << "\n";

    // 低电压蜂鸣器不是飞控闭环必需项，初始化失败只报警不退出。
    drone::BuzzerGpio low_voltage_buzzer;
    if (!low_voltage_buzzer.initialize()) {
        std::cerr << "Low-voltage buzzer on GP2 unavailable: "
                  << low_voltage_buzzer.lastError() << "\n";
    } else {
        std::cout << "Low-voltage buzzer initialized on GP2\n";
    }

    // 蓝牙用于手机端查看遥测；不可用时不影响核心飞控。
    drone::SerialPort bluetooth;
    if (bluetooth_enabled) {
        if (!bluetooth.open(bluetooth_device, bluetooth_baudrate)) {
            std::cerr << "HC-05 bluetooth telemetry unavailable on "
                      << bluetooth_device << ": " << bluetooth.lastError() << "\n";
        } else {
            std::cout << "HC-05 bluetooth telemetry opened on " << bluetooth_device
                      << " at " << bluetooth_baudrate << " baud\n";
        }
    }

    // 上电后先给 ESC 保持最低油门，让电调完成低油门识别。
    std::cout << "Holding ESCs at low throttle for 5 seconds to arm...\n";
    sleepWhileRunning(std::chrono::seconds(5));
    if (!g_running) {
        low_voltage_buzzer.off();
        bluetooth.close();
        esc_pwm.stopAll();
        esc_pwm.disableAll();
        ibus_receiver.close();
        return 0;
    }

    // 初始化 MPU6050；默认地址失败后尝试备用地址。
    drone::Mpu6050 imu;
    bool imu_initialized = imu.initialize(drone::kMpu6050DefaultI2cDevice, drone::kMpu6050DefaultAddress);
    if (!imu_initialized) {
        imu_initialized = imu.initializeAnyAddress();
    }

    // IMU 可用时先校准陀螺仪零偏，后续读数都会减掉该偏移。
    GyroBias gyro_bias;
    if (imu_initialized) {
        gyro_bias = calibrateGyroBias(imu);
    } else {
        std::cerr << "IMU initialization failed; staying disarmed\n";
    }

    // 创建飞控核心对象：姿态滤波、定频循环、遥控映射和三轴 PID。
    drone::ComplementaryFilter filter;
    drone::LoopRate loop_rate(kMainLoopHz);
    drone::FlightControlConfig flight_config;

    // Conservative starter gains; tune with propellers removed before flight.
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
    drone::BatteryMonitor battery_monitor(battery_source);
    auto last_rc_time = std::chrono::steady_clock::time_point::min();

    // 主状态：armed 控制是否允许电机输出，低电压触发后会锁存。
    bool armed = false;
    bool arm_switch_ready = false;
    bool low_voltage_latched = false;
    auto low_voltage_latch_time = std::chrono::steady_clock::time_point::min();

    // 多个定时点分别控制日志、遥测和电池轮询频率。
    auto last_loop_time = std::chrono::steady_clock::now();
    auto next_heartbeat = last_loop_time + kHeartbeatPeriod;
    auto next_attitude_log = last_loop_time + kAttitudeLogPeriod;
    auto next_bluetooth_telemetry = last_loop_time + kBluetoothTelemetryPeriod;
    auto next_battery_poll = last_loop_time;

    std::cout << "Milk-V Duo 256 drone controller skeleton started at " << kMainLoopHz << " Hz\n"
              << "iBUS input: " << ibus_device << " (GP3/UART1_RX, 115200 8N1 raw)\n"
              << "Safety notice: bench-test without propellers installed. RC failsafe disarms within 200 ms.\n";
    if (battery_monitor.enabled()) {
        std::cout << "Battery monitor source: " << battery_source << "\n";
    }

    if (test_pid) {
        std::cout << "=== PID TEST MODE ===\n"
                  << "Target: RC roll/pitch attitude and yaw rate commands are active\n"
                  << "Gains: roll kp=" << kTestPidRollKp << " ki=" << kTestPidRollKi
                  << " kd=" << kTestPidRollKd << "\n"
                  << "       pitch kp=" << kTestPidPitchKp << " ki=" << kTestPidPitchKi
                  << " kd=" << kTestPidPitchKd << "\n"
                  << "       yaw kp=" << kTestPidYawKp << " ki=" << kTestPidYawKi
                  << " kd=" << kTestPidYawKd << "\n"
                  << "Keep propellers removed; tilt the board and move sticks to see PID corrections.\n";
    }

    // 主飞控循环：每轮完成输入读取、安全判断、姿态闭环和电机输出。
    while (g_running) {
        const auto now = std::chrono::steady_clock::now();

        // 读取所有已经到达的 iBUS 帧，只保留最新一帧通道值。
        drone::IBusChannels ibus_channels;
        while (ibus_receiver.readFrame(ibus_channels)) {
            for (std::size_t i = 0; i < rc.channels.size(); ++i) {
                rc.channels[i] = static_cast<int>(ibus_channels.ch[i]);
            }
            rc.failsafe = false;
            rc.valid = true;
            last_rc_time = now;
        }

        // 电池电压低频读取即可，避免每个 100Hz 循环都访问文件或外设。
        if (battery_monitor.enabled() && now >= next_battery_poll) {
            if (!battery_monitor.poll(battery)) {
                std::cerr << "Battery monitor read failed: "
                          << battery_monitor.lastError() << "\n";
            }
            next_battery_poll += kBatteryPollPeriod;
        }

        // dt 是滤波和 PID 的时间基准。
        double dt_s = std::chrono::duration<double>(now - last_loop_time).count();
        if (dt_s <= 0.0) {
            dt_s = loop_rate.periodSeconds();
        }
        last_loop_time = now;

        // 姿态闭环数据流：IMU 原始数据 -> 去零偏 -> 互补滤波 -> PID -> 电机混控。
        drone::ImuSample imu_sample = imu.read();
        if (imu_sample.valid) {
            imu_sample.gx_dps -= gyro_bias.gx_dps;
            imu_sample.gy_dps -= gyro_bias.gy_dps;
            imu_sample.gz_dps -= gyro_bias.gz_dps;
        }
        const drone::Attitude attitude = filter.update(imu_sample, dt_s);
        const bool invalid_imu_safety = (!imu.isValid()) || (!imu_sample.valid) || (!attitude.valid);

        // 遥控超过 200ms 没有新帧，或通道值异常，都视为 failsafe。
        const bool have_rc_time = (last_rc_time != std::chrono::steady_clock::time_point::min());
        const auto rc_age = have_rc_time ? std::chrono::duration_cast<std::chrono::milliseconds>(
                                               now - last_rc_time)
                                         : std::chrono::hours(24);
        const bool rc_fresh = rcChannelsSane(rc) && rc_age <= kRcTimeout;
        const bool rc_failsafe = (!rc_fresh) || rc.failsafe;
        const int swa_us = rc.channels[kSwaChannelIndex];
        const bool emergency_stop = drone::isEmergencyStopSwaUs(swa_us);

        // 低电压一旦触发就锁存，防止电压回弹后再次允许解锁。
        if (battery.valid && battery.voltage_mv < kLowVoltageMv) {
            if (!low_voltage_latched) {
                low_voltage_latch_time = now;
            }
            low_voltage_latched = true;
        }

        const bool imu_valid = !invalid_imu_safety;
        const bool throttle_low = rc_fresh && rc.channels[2] <= kThrottleLowUs;

        if (low_voltage_latched) {
            // 低电压蜂鸣器节奏由 elapsed_ms % 500 控制。
            const auto elapsed = low_voltage_latch_time == std::chrono::steady_clock::time_point::min()
                ? std::chrono::milliseconds(0)
                : std::chrono::duration_cast<std::chrono::milliseconds>(now - low_voltage_latch_time);
            low_voltage_buzzer.set(drone::lowVoltageBuzzerOn(true, elapsed));
        } else {
            low_voltage_buzzer.set(false);
        }

        // 把 1000-2000us 的遥控通道转换成归一化摇杆、目标姿态和油门。
        drone::PilotCommand pilot_command;
        if (rc_fresh) {
            pilot_command = drone::makePilotCommand(rc.channels[0],
                                                    rc.channels[1],
                                                    rc.channels[2],
                                                    rc.channels[3],
                                                    flight_config);
        }
        const bool arm_switch_high = rc_fresh && rc.channels[kArmSwitchChannelIndex] > kSwitchThresholdUs;
        const bool mode_switch = false;

        // 解锁开关必须先回到低位，再拨到高位，避免上电即解锁。
        if (!arm_switch_high) {
            arm_switch_ready = true;
        }
        if (emergency_stop) {
            arm_switch_ready = false;
        }

        // 任一关键安全条件失败都会立即上锁；解锁还要求低油门和未低电压。
        if (emergency_stop || rc_failsafe || invalid_imu_safety || !arm_switch_high) {
            armed = false;
            pilot_command = {};
        } else if (!armed && !low_voltage_latched && arm_switch_ready && arm_switch_high && throttle_low) {
            armed = true;
            arm_switch_ready = false;
        }

        double pid_roll_out = 0.0;
        double pid_pitch_out = 0.0;
        double pid_yaw_out = 0.0;
        if (armed && rc_fresh && imu_valid) {
            // 三轴闭环控制：roll/pitch 使用姿态角闭环，yaw 使用角速度闭环。
            pid_roll_out = pid_roll.update(pilot_command.roll_target_deg, attitude.roll_deg, dt_s);
            pid_pitch_out = pid_pitch.update(pilot_command.pitch_target_deg, attitude.pitch_deg, dt_s);
            pid_yaw_out = pid_yaw.update(pilot_command.yaw_rate_target_dps, imu_sample.gz_dps, dt_s);
        } else {
            pid_roll.reset();
            pid_pitch.reset();
            pid_yaw.reset();
        }

        // 汇总安全状态、油门和 PID 修正量，交给电机输出模块统一裁决。
        drone::MotorOutputInput motor_input;
        motor_input.armed = armed;
        motor_input.rc_valid = rc_fresh;
        motor_input.failsafe = rc_failsafe;
        motor_input.invalid_imu = invalid_imu_safety;
        motor_input.emergency_stop = emergency_stop;
        motor_input.low_voltage = low_voltage_latched;
        motor_input.mode = mode_switch;
        motor_input.throttle = pilot_command.throttle;
        motor_input.roll_cmd = pilot_command.roll_stick;
        motor_input.pitch_cmd = pilot_command.pitch_stick;
        motor_input.yaw_cmd = pilot_command.yaw_stick;
        motor_input.use_pid_corrections = armed && rc_fresh && imu_valid;
        motor_input.roll_correction_us = pid_roll_out;
        motor_input.pitch_correction_us = pid_pitch_out;
        motor_input.yaw_correction_us = pid_yaw_out;
        const drone::MotorOutputResult motor_output = drone::computeMotorOutput(motor_input);
        const std::array<int, 4> motors = motor_output.motors_after_clamp;

        // 最终 PWM 写入 ESC；写失败时立即停机并退出主循环。
        if (!writeMotorsToEsc(esc_pwm, motors)) {
            std::cerr << "ESC PWM write failed: " << esc_pwm.lastError() << "\n";
            esc_pwm.stopAll();
            g_running = false;
        }

        // 10Hz 详细姿态日志，主要用于台架调 PID 和混控方向。
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
                      << " emergency_stop=" << (emergency_stop ? 1 : 0)
                      << " swa_us=" << swa_us
                      << " pid_r=" << pid_roll_out
                      << " pid_p=" << pid_pitch_out
                      << " pid_y=" << pid_yaw_out
                      << " rc_cmd=[" << pilot_command.roll_stick << ',' << pilot_command.pitch_stick << ','
                      << pilot_command.throttle << ',' << pilot_command.yaw_stick << ']'
                      << " targets=[roll_deg=" << pilot_command.roll_target_deg
                      << ",pitch_deg=" << pilot_command.pitch_target_deg
                      << ",yaw_rate_dps=" << pilot_command.yaw_rate_target_dps << ']'
                      << " motor_max_us=" << motor_output.motor_max_us
                      << " base_us=" << motor_output.base_us
                      << " roll_mix=" << motor_output.roll_mix_us
                      << " pitch_mix=" << motor_output.pitch_mix_us
                      << " yaw_mix=" << motor_output.yaw_mix_us
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

        // 10Hz 蓝牙遥测，给手机串口助手显示姿态、电压和解锁状态。
        if (bluetooth.isOpen() && now >= next_bluetooth_telemetry) {
            drone::BluetoothTelemetrySnapshot snapshot;
            snapshot.roll_deg = attitude.roll_deg;
            snapshot.pitch_deg = attitude.pitch_deg;
            snapshot.yaw_deg = attitude.yaw_deg;
            snapshot.battery_mv = battery.voltage_mv;
            snapshot.low_voltage = low_voltage_latched;
            snapshot.armed = armed;
            snapshot.function_sensor_temp_c = imu_sample.temperature_c;
            if (!bluetooth.writeLine(drone::encodeBluetoothTelemetry(snapshot))) {
                std::cerr << "HC-05 bluetooth telemetry write failed: "
                          << bluetooth.lastError() << "\n";
                bluetooth.close();
            }
            next_bluetooth_telemetry += kBluetoothTelemetryPeriod;
        }

        // 1Hz 日志。
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
                      << " emergency_stop=" << (emergency_stop ? 1 : 0)
                      << " swa_us=" << swa_us
                      << " rc_ch=[" << rc.channels[0] << ',' << rc.channels[1] << ','
                      << rc.channels[2] << ',' << rc.channels[3] << ','
                      << rc.channels[4] << ',' << rc.channels[5] << ']'
                      << " cmd=[" << pilot_command.roll_stick << ',' << pilot_command.pitch_stick << ','
                      << pilot_command.throttle << ',' << pilot_command.yaw_stick << ']'
                      << " targets=[roll_deg=" << pilot_command.roll_target_deg
                      << ",pitch_deg=" << pilot_command.pitch_target_deg
                      << ",yaw_rate_dps=" << pilot_command.yaw_rate_target_dps << ']'
                      << " mode=" << (mode_switch ? 1 : 0)
                      << " throttle=" << pilot_command.throttle
                      << " motor_max_us=" << motor_output.motor_max_us
                      << " base_us=" << motor_output.base_us
                      << " roll_mix=" << motor_output.roll_mix_us
                      << " pitch_mix=" << motor_output.pitch_mix_us
                      << " yaw_mix=" << motor_output.yaw_mix_us
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

        // 补足本轮剩余时间，让主循环稳定在 100Hz 左右。
        loop_rate.sleep();
    }

    // 退出前的安全收尾：关蜂鸣器、关蓝牙、停 PWM、释放串口。
    low_voltage_buzzer.off();
    bluetooth.close();
    esc_pwm.stopAll();
    esc_pwm.disableAll();
    ibus_receiver.close();

    return 0;
}

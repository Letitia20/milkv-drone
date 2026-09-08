#pragma once

#include "imu_mpu6050.hpp"

namespace drone {

// 姿态角结果：roll/pitch/yaw 单位均为度。
// valid=false 表示本次 IMU 数据无效，飞控应保持安全状态。
struct Attitude {
    bool valid {false};
    double roll_deg {0.0};
    double pitch_deg {0.0};
    double yaw_deg {0.0};
};

// 互补滤波器功能：
// 1. 用加速度计估计 roll/pitch 的低频姿态；
// 2. 用陀螺仪积分补偿快速动态；
// 3. MPU6050 无磁力计，所以 yaw 暂时只由 gz 积分得到。
class ComplementaryFilter {
public:
    explicit ComplementaryFilter(double alpha = 0.98);

    Attitude update(const ImuSample& sample, double dt_s);
    void reset(const Attitude& attitude = {});

    Attitude attitude() const;

private:
    double alpha_;
    bool initialized_ {false};
    Attitude attitude_;
};

}  // namespace drone

#include "complementary_filter.hpp"

#include <algorithm>
#include <cmath>

namespace drone {
namespace {

constexpr double kRadToDeg = 57.29577951308232;

}  // namespace

ComplementaryFilter::ComplementaryFilter(double alpha)
    : alpha_(std::clamp(alpha, 0.0, 1.0)) {}

Attitude ComplementaryFilter::update(const ImuSample& sample, double dt_s) {
    if (!sample.valid || dt_s <= 0.0) {
        Attitude invalid_attitude = attitude_;
        invalid_attitude.valid = false;
        return invalid_attitude;
    }

    const double roll_acc = std::atan2(sample.ay_g, sample.az_g) * kRadToDeg;
    const double pitch_acc =
        std::atan2(-sample.ax_g, std::sqrt(sample.ay_g * sample.ay_g + sample.az_g * sample.az_g)) *
        kRadToDeg;

    if (!initialized_) {
        attitude_.valid = true;
        attitude_.roll_deg = roll_acc;
        attitude_.pitch_deg = pitch_acc;
        attitude_.yaw_deg = 0.0;
        initialized_ = true;
        return attitude_;
    }

    const double roll_gyro = attitude_.roll_deg + sample.gx_dps * dt_s;
    const double pitch_gyro = attitude_.pitch_deg + sample.gy_dps * dt_s;

    attitude_.roll_deg = alpha_ * roll_gyro + (1.0 - alpha_) * roll_acc;
    attitude_.pitch_deg = alpha_ * pitch_gyro + (1.0 - alpha_) * pitch_acc;
    attitude_.valid = true;

    // MPU6050 has no magnetometer, so yaw is gyro-only until a later heading source is added.
    attitude_.yaw_deg += sample.gz_dps * dt_s;

    return attitude_;
}

void ComplementaryFilter::reset(const Attitude& attitude) {
    attitude_ = attitude;
    initialized_ = false;
}

Attitude ComplementaryFilter::attitude() const {
    return attitude_;
}

}  // namespace drone

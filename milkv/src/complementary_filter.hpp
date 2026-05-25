#pragma once

#include "imu_mpu6050.hpp"

namespace drone {

struct Attitude {
    bool valid {false};
    double roll_deg {0.0};
    double pitch_deg {0.0};
    double yaw_deg {0.0};
};

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

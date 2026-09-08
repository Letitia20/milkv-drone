#include "pid.hpp"

#include <algorithm>
#include <cmath>

namespace drone {

PID::PID(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd) {}

void PID::setGains(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PID::setIntegralLimit(double limit_abs) {
    integral_limit_ = std::abs(limit_abs);
    if (integral_limit_ > 0.0) {
        integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);
    }
}

void PID::setOutputLimits(double min_output, double max_output) {
    if (min_output > max_output) {
        std::swap(min_output, max_output);
    }
    output_min_ = min_output;
    output_max_ = max_output;
    output_limited_ = true;
}

double PID::update(double setpoint, double measurement, double dt_s) {
    // 比例项 P：当前误差，误差越大输出越大。
    const double error = setpoint - measurement;

    // 微分项 D：误差变化率，用于抑制姿态快速越过目标。
    double derivative = 0.0;
    if (!first_update_ && dt_s > 0.0) {
        derivative = (error - previous_error_) / dt_s;
    }

    // 积分项 I：历史误差累计，用于消除长期偏差；限制积分防止积分饱和。
    if (dt_s > 0.0) {
        integral_ += error * dt_s;
        if (integral_limit_ > 0.0) {
            integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);
        }
    }

    // 本项目中 PID 输出单位映射为电机 PWM 修正量 us。
    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    if (output_limited_) {
        output = std::clamp(output, output_min_, output_max_);
    }

    previous_error_ = error;
    first_update_ = false;
    return output;
}

void PID::reset() {
    integral_ = 0.0;
    previous_error_ = 0.0;
    first_update_ = true;
}

}  // namespace drone

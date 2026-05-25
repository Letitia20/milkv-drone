#pragma once

namespace drone {

class PID {
public:
    PID(double kp = 0.0, double ki = 0.0, double kd = 0.0);

    void setGains(double kp, double ki, double kd);
    void setIntegralLimit(double limit_abs);
    void setOutputLimits(double min_output, double max_output);

    double update(double setpoint, double measurement, double dt_s);
    void reset();

private:
    double kp_ {0.0};
    double ki_ {0.0};
    double kd_ {0.0};

    double integral_ {0.0};
    double integral_limit_ {0.0};

    double output_min_ {0.0};
    double output_max_ {0.0};
    bool output_limited_ {false};

    double previous_error_ {0.0};
    bool first_update_ {true};
};

}  // namespace drone

#pragma once

#include <chrono>
#include <stdexcept>
#include <thread>

namespace drone {

class LoopRate {
public:
    explicit LoopRate(double hz)
        : period_(std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(1.0 / hz))),
          next_wakeup_(std::chrono::steady_clock::now() + period_) {
        if (hz <= 0.0) {
            throw std::invalid_argument("LoopRate frequency must be positive");
        }
    }

    void sleep() {
        const auto now = std::chrono::steady_clock::now();
        if (now < next_wakeup_) {
            std::this_thread::sleep_until(next_wakeup_);
        }

        const auto after_sleep = std::chrono::steady_clock::now();
        if (after_sleep > next_wakeup_ + period_) {
            next_wakeup_ = after_sleep + period_;
        } else {
            next_wakeup_ += period_;
        }
    }

    double periodSeconds() const {
        return std::chrono::duration<double>(period_).count();
    }

private:
    std::chrono::steady_clock::duration period_;
    std::chrono::steady_clock::time_point next_wakeup_;
};

}  // namespace drone

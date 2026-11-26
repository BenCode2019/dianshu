#pragma once
#include <algorithm>
#include <cmath>

namespace dianshu_controller {
inline double wrap(double a) {return std::atan2(std::sin(a), std::cos(a));}
class HeadingLatch {
public:
  void reset() {locked_ = false; previous_rate_ = 0.0;}
  double step(double error, double dt, double max_rate, double accel, double stop_tolerance) {
    if (!locked_) {
      direction_ = error >= 0.0 ? 1.0 : -1.0;
      remaining_ = previous_error_ = error;
      locked_ = true;
    } else {
      remaining_ += wrap(error - previous_error_);
      previous_error_ = error;
    }
    const double left = std::max(0.0, direction_ * remaining_ - stop_tolerance);
    const double target = std::min({max_rate, 0.9 * left, std::sqrt(2.0 * accel * left)});
    const double magnitude = std::clamp(target,
      std::max(0.0, previous_rate_ - accel * dt), previous_rate_ + accel * dt);
    // Once past the target, stop; never turn a small overshoot into a full circle.
    previous_rate_ = direction_ * remaining_ <= 0.0 ? 0.0 : magnitude;
    return direction_ * previous_rate_;
  }
  bool locked() const {return locked_;}
private:
  bool locked_{false};
  double direction_{1.0}, remaining_{0.0}, previous_error_{0.0}, previous_rate_{0.0};
};
}

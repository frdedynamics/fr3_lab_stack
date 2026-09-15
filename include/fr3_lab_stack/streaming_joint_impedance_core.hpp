#pragma once

#include <array>
#include <cmath>
#include <cstddef>

namespace fr3_lab_stack {

class StreamingJointImpedanceCore {
 public:
  static constexpr std::size_t kNumJoints = 7;
  using Vector7 = std::array<double, kNumJoints>;

  struct Output {
    Vector7 desired_q{};
    Vector7 tracking_error{};
    Vector7 commanded_effort{};
  };

  bool configure(const Vector7& k_gains, const Vector7& d_gains) noexcept {
    if (!finite(k_gains) || !finite(d_gains)) {
      return false;
    }
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      if (k_gains[i] < 0.0 || d_gains[i] < 0.0) {
        return false;
      }
    }
    k_gains_ = k_gains;
    d_gains_ = d_gains;
    configured_ = true;
    return true;
  }

  bool activate(const Vector7& measured_q) noexcept {
    if (!configured_ || !finite(measured_q)) {
      return false;
    }
    desired_q_ = measured_q;
    active_ = true;
    return true;
  }

  void deactivate() noexcept { active_ = false; }

  bool set_target(const Vector7& target_q) noexcept {
    if (!active_ || !finite(target_q)) {
      return false;
    }
    desired_q_ = target_q;
    return true;
  }

  bool compute(const Vector7& measured_q, const Vector7& measured_dq, Output* output) const noexcept {
    if (!active_ || output == nullptr || !finite(measured_q) || !finite(measured_dq)) {
      return false;
    }

    for (std::size_t i = 0; i < kNumJoints; ++i) {
      output->desired_q[i] = desired_q_[i];
      output->tracking_error[i] = desired_q_[i] - measured_q[i];
      output->commanded_effort[i] =
          k_gains_[i] * output->tracking_error[i] - d_gains_[i] * measured_dq[i];
      if (!std::isfinite(output->commanded_effort[i])) {
        return false;
      }
    }
    return true;
  }

  [[nodiscard]] bool configured() const noexcept { return configured_; }
  [[nodiscard]] bool active() const noexcept { return active_; }
  [[nodiscard]] const Vector7& desired_q() const noexcept { return desired_q_; }

 private:
  static bool finite(const Vector7& values) noexcept {
    for (const double value : values) {
      if (!std::isfinite(value)) {
        return false;
      }
    }
    return true;
  }

  Vector7 k_gains_{};
  Vector7 d_gains_{};
  Vector7 desired_q_{};
  bool configured_{false};
  bool active_{false};
};

}  // namespace fr3_lab_stack

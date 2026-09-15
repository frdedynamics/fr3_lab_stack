#pragma once

#include <array>
#include <cmath>
#include <cstddef>

#include <Eigen/Core>

namespace fr3_lab_stack {

class HybridImpedanceMath {
 public:
  static constexpr std::size_t kNumJoints = 7;
  static constexpr std::size_t kCartesianDof = 6;

  using Vector7 = std::array<double, kNumJoints>;
  using Vector6 = std::array<double, kCartesianDof>;
  using JacobianArray = std::array<double, kCartesianDof * kNumJoints>;

  // Computes the Polymetis-style Jacobian-dependent feedback term
  //
  //   tau_cart =
  //       J^T Kx J (q_des - q)
  //     - J^T Kxd J dq
  //
  // and the additional torque used by the controller:
  //
  //   tau_additional = tau_cart + coriolis.
  //
  // Gravity is deliberately not added here, matching Polymetis'
  // HybridJointImpedanceControl with ignore_gravity=true.
  static bool compute_additional(
      const Vector7& tracking_error,
      const Vector7& measured_dq,
      const Vector6& cartesian_k_gains,
      const Vector6& cartesian_d_gains,
      const JacobianArray& jacobian_column_major,
      const Vector7& coriolis,
      Vector7* cartesian_feedback,
      Vector7* additional_effort) noexcept {
    if (cartesian_feedback == nullptr || additional_effort == nullptr ||
        !finite(tracking_error) || !finite(measured_dq) ||
        !finite(cartesian_k_gains) || !finite(cartesian_d_gains) ||
        !finite(jacobian_column_major) || !finite(coriolis)) {
      return false;
    }

    for (std::size_t i = 0; i < kCartesianDof; ++i) {
      if (cartesian_k_gains[i] < 0.0 || cartesian_d_gains[i] < 0.0) {
        return false;
      }
    }

    Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(
        jacobian_column_major.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> error(
        tracking_error.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(
        measured_dq.data());
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> kx(
        cartesian_k_gains.data());
    Eigen::Map<const Eigen::Matrix<double, 6, 1>> kxd(
        cartesian_d_gains.data());

    const Eigen::Matrix<double, 6, 1> cartesian_error = jacobian * error;
    const Eigen::Matrix<double, 6, 1> cartesian_velocity = jacobian * dq;
    const Eigen::Matrix<double, 6, 1> feedback_wrench =
        kx.cwiseProduct(cartesian_error) -
        kxd.cwiseProduct(cartesian_velocity);
    const Eigen::Matrix<double, 7, 1> tau_cart =
        jacobian.transpose() * feedback_wrench;

    for (std::size_t i = 0; i < kNumJoints; ++i) {
      (*cartesian_feedback)[i] = tau_cart(static_cast<Eigen::Index>(i));
      (*additional_effort)[i] =
          (*cartesian_feedback)[i] + coriolis[i];
      if (!std::isfinite((*cartesian_feedback)[i]) ||
          !std::isfinite((*additional_effort)[i])) {
        return false;
      }
    }
    return true;
  }

 private:
  template <std::size_t N>
  static bool finite(const std::array<double, N>& values) noexcept {
    for (const double value : values) {
      if (!std::isfinite(value)) {
        return false;
      }
    }
    return true;
  }
};

}  // namespace fr3_lab_stack

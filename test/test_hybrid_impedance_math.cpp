#include <array>
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "fr3_lab_stack/hybrid_impedance_math.hpp"

namespace {

using Math = fr3_lab_stack::HybridImpedanceMath;

Math::JacobianArray identity_like_jacobian() {
  Math::JacobianArray values{};
  // Eigen::Map in the implementation interprets this array as column-major
  // 6x7. Set J(i, i) = 1 for joints 1..6; joint 7 is outside the 6D task.
  for (std::size_t i = 0; i < 6; ++i) {
    values[i + 6 * i] = 1.0;
  }
  return values;
}

TEST(HybridImpedanceMath, ZeroFeedbackPassesThroughCoriolis) {
  Math::Vector7 error{};
  Math::Vector7 dq{};
  Math::Vector6 kx{750.0, 750.0, 750.0, 15.0, 15.0, 15.0};
  Math::Vector6 kxd{37.0, 37.0, 37.0, 2.0, 2.0, 2.0};
  Math::JacobianArray jacobian{};
  Math::Vector7 coriolis{1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0};

  Math::Vector7 cartesian{};
  Math::Vector7 additional{};

  ASSERT_TRUE(Math::compute_additional(
      error, dq, kx, kxd, jacobian, coriolis,
      &cartesian, &additional));

  for (std::size_t i = 0; i < 7; ++i) {
    EXPECT_DOUBLE_EQ(cartesian[i], 0.0);
    EXPECT_DOUBLE_EQ(additional[i], coriolis[i]);
  }
}

TEST(HybridImpedanceMath, IdentityLikeJacobianAppliesCartesianStiffness) {
  Math::Vector7 error{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  Math::Vector7 dq{};
  Math::Vector6 kx{1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  Math::Vector6 kxd{};
  const auto jacobian = identity_like_jacobian();
  Math::Vector7 coriolis{};

  Math::Vector7 cartesian{};
  Math::Vector7 additional{};

  ASSERT_TRUE(Math::compute_additional(
      error, dq, kx, kxd, jacobian, coriolis,
      &cartesian, &additional));

  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_DOUBLE_EQ(cartesian[i], kx[i]);
    EXPECT_DOUBLE_EQ(additional[i], kx[i]);
  }
  EXPECT_DOUBLE_EQ(cartesian[6], 0.0);
  EXPECT_DOUBLE_EQ(additional[6], 0.0);
}

TEST(HybridImpedanceMath, IdentityLikeJacobianAppliesCartesianDamping) {
  Math::Vector7 error{};
  Math::Vector7 dq{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  Math::Vector6 kx{};
  Math::Vector6 kxd{1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  const auto jacobian = identity_like_jacobian();
  Math::Vector7 coriolis{};

  Math::Vector7 cartesian{};
  Math::Vector7 additional{};

  ASSERT_TRUE(Math::compute_additional(
      error, dq, kx, kxd, jacobian, coriolis,
      &cartesian, &additional));

  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_DOUBLE_EQ(cartesian[i], -kxd[i]);
    EXPECT_DOUBLE_EQ(additional[i], -kxd[i]);
  }
  EXPECT_DOUBLE_EQ(cartesian[6], 0.0);
  EXPECT_DOUBLE_EQ(additional[6], 0.0);
}

TEST(HybridImpedanceMath, RejectsNonFiniteInput) {
  Math::Vector7 error{};
  Math::Vector7 dq{};
  Math::Vector6 kx{};
  Math::Vector6 kxd{};
  Math::JacobianArray jacobian{};
  Math::Vector7 coriolis{};
  Math::Vector7 cartesian{};
  Math::Vector7 additional{};

  error[0] = std::numeric_limits<double>::quiet_NaN();

  EXPECT_FALSE(Math::compute_additional(
      error, dq, kx, kxd, jacobian, coriolis,
      &cartesian, &additional));
}

TEST(HybridImpedanceMath, RejectsNegativeGain) {
  Math::Vector7 error{};
  Math::Vector7 dq{};
  Math::Vector6 kx{};
  Math::Vector6 kxd{};
  Math::JacobianArray jacobian{};
  Math::Vector7 coriolis{};
  Math::Vector7 cartesian{};
  Math::Vector7 additional{};

  kx[0] = -1.0;

  EXPECT_FALSE(Math::compute_additional(
      error, dq, kx, kxd, jacobian, coriolis,
      &cartesian, &additional));
}

}  // namespace

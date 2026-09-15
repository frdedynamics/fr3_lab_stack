#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "fr3_lab_stack/streaming_joint_impedance_core.hpp"

namespace {

using Core = fr3_lab_stack::StreamingJointImpedanceCore;
using Vector7 = Core::Vector7;

const Vector7 kGains{40.0, 30.0, 50.0, 25.0, 35.0, 25.0, 10.0};
const Vector7 dGains{4.0, 6.0, 5.0, 5.0, 3.0, 2.0, 1.0};

TEST(StreamingJointImpedanceCore, ActivationHoldsMeasuredPoseWithoutAutonomousMotion) {
  Core core;
  ASSERT_TRUE(core.configure(kGains, dGains));

  const Vector7 q{0.1, -0.2, 0.3, -1.8, 0.5, 1.2, -0.7};
  const Vector7 dq{};
  ASSERT_TRUE(core.activate(q));

  Core::Output output;
  ASSERT_TRUE(core.compute(q, dq, &output));
  for (std::size_t i = 0; i < Core::kNumJoints; ++i) {
    EXPECT_DOUBLE_EQ(output.desired_q[i], q[i]);
    EXPECT_DOUBLE_EQ(output.tracking_error[i], 0.0);
    EXPECT_DOUBLE_EQ(output.commanded_effort[i], 0.0);
  }
}

TEST(StreamingJointImpedanceCore, HoldsLastTargetUntilReplaced) {
  Core core;
  ASSERT_TRUE(core.configure(kGains, dGains));
  const Vector7 q{};
  ASSERT_TRUE(core.activate(q));

  Vector7 target{};
  target[0] = 0.01;
  target[6] = -0.02;
  ASSERT_TRUE(core.set_target(target));

  Core::Output first;
  Core::Output second;
  ASSERT_TRUE(core.compute(q, Vector7{}, &first));
  ASSERT_TRUE(core.compute(q, Vector7{}, &second));

  EXPECT_EQ(first.desired_q, target);
  EXPECT_EQ(second.desired_q, target);
  EXPECT_DOUBLE_EQ(first.commanded_effort[0], 0.4);
  EXPECT_DOUBLE_EQ(first.commanded_effort[6], -0.2);
}

TEST(StreamingJointImpedanceCore, LatestTargetWins) {
  Core core;
  ASSERT_TRUE(core.configure(kGains, dGains));
  ASSERT_TRUE(core.activate(Vector7{}));

  Vector7 first_target{};
  first_target[1] = 0.01;
  Vector7 second_target{};
  second_target[1] = 0.02;

  ASSERT_TRUE(core.set_target(first_target));
  ASSERT_TRUE(core.set_target(second_target));

  Core::Output output;
  ASSERT_TRUE(core.compute(Vector7{}, Vector7{}, &output));
  EXPECT_EQ(output.desired_q, second_target);
  EXPECT_DOUBLE_EQ(output.commanded_effort[1], 0.6);
}

TEST(StreamingJointImpedanceCore, DampingOpposesMeasuredVelocity) {
  Core core;
  ASSERT_TRUE(core.configure(kGains, dGains));
  ASSERT_TRUE(core.activate(Vector7{}));

  Vector7 dq{};
  dq[2] = 0.2;
  Core::Output output;
  ASSERT_TRUE(core.compute(Vector7{}, dq, &output));
  EXPECT_DOUBLE_EQ(output.commanded_effort[2], -1.0);
}

TEST(StreamingJointImpedanceCore, RejectsNonFiniteTargetWithoutReplacingHold) {
  Core core;
  ASSERT_TRUE(core.configure(kGains, dGains));
  const Vector7 q{};
  ASSERT_TRUE(core.activate(q));

  Vector7 invalid{};
  invalid[3] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(core.set_target(invalid));
  EXPECT_EQ(core.desired_q(), q);
}

}  // namespace

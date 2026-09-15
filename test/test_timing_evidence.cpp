#include <gtest/gtest.h>
#include <realtime_tools/realtime_buffer.hpp>
#include "fr3_lab_stack/timing_evidence.hpp"
#include "fr3_lab_stack/streaming_joint_impedance_core.hpp"
namespace {
using namespace fr3_lab_stack;
struct Command : timing::TargetTiming { std::array<double, 7> q{}; };
TEST(Timing, BufferHoldSupersessionActivationOverflow) {
  StreamingJointImpedanceCore core;
  std::array<double, 7> gains{};
  gains.fill(1.0);
  ASSERT_TRUE(core.configure(gains, gains));
  ASSERT_TRUE(core.activate({}));
  realtime_tools::RealtimeBuffer<Command> buffer;
  Command command;
  command.activation = 11;
  command.sequence = 1;
  command.callback_ns = timing::monotonic_ns();
  command.stamp_ns = 456;
  command.q.fill(.2);
  buffer.writeFromNonRT(command);
  command.sequence = 2;
  command.q.fill(.3);
  buffer.writeFromNonRT(command);
  std::uint64_t last = 0;
  timing::Sample first;
  ASSERT_TRUE(timing::install(core, *buffer.readFromRT(), last, first));
  EXPECT_TRUE(first.applied);
  EXPECT_EQ(first.target.sequence, 2u);
  EXPECT_EQ(first.target.callback_ns, command.callback_ns);
  EXPECT_EQ(first.target.stamp_ns, 456);
  EXPECT_EQ(first.q_desired, command.q);
  EXPECT_GE(first.application_ns, command.callback_ns);
  const auto t4 = first.application_ns;
  for (int i = 0; i < 10; ++i) {
    timing::Sample hold;
    ASSERT_TRUE(timing::install(core, *buffer.readFromRT(), last, hold));
    EXPECT_FALSE(hold.applied);
  }
  EXPECT_EQ(first.application_ns, t4);
  timing::Ring<1> ring;
  ASSERT_TRUE(ring.push(first));
  core.deactivate();
  ASSERT_TRUE(core.activate({}));
  last = 0;
  command.activation = 22;
  command.sequence = 1;
  command.q.fill(.4);
  buffer.writeFromNonRT(command);
  timing::Sample second;
  ASSERT_TRUE(timing::install(core, *buffer.readFromRT(), last, second));
  EXPECT_FALSE(ring.push(second));
  EXPECT_EQ(ring.dropped(), 1u);
  EXPECT_EQ(ring.dropped_applications(), 1u);
  StreamingJointImpedanceCore::Output output;
  ASSERT_TRUE(core.compute({}, {}, &output));
  EXPECT_EQ(output.desired_q, command.q);
  EXPECT_EQ(last, 1u);
  timing::Sample read;
  ASSERT_TRUE(ring.pop(read));
  EXPECT_EQ(read.target.activation, 11);
  ASSERT_TRUE(ring.push(second));
  ASSERT_TRUE(ring.pop(read));
  EXPECT_EQ(read.target.activation, 22);
  EXPECT_EQ(read.target.sequence, 1u);
  EXPECT_FALSE(ring.pop(read));
  for (int i = 0; i < 10; ++i) {
    read.cycle = i;
    ASSERT_TRUE(ring.push(read));
    ASSERT_TRUE(ring.pop(read));
    EXPECT_EQ(read.cycle, static_cast<unsigned>(i));
  }
}
TEST(Timing, ExplicitPosixClock) {
  timespec before{}, after{};
  ASSERT_EQ(clock_gettime(CLOCK_MONOTONIC, &before), 0);
  const auto value = timing::monotonic_ns();
  ASSERT_EQ(clock_gettime(CLOCK_MONOTONIC, &after), 0);
  EXPECT_GE(value, before.tv_sec * 1000000000LL + before.tv_nsec);
  EXPECT_LE(value, after.tv_sec * 1000000000LL + after.tv_nsec);
}
}

#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <ctime>

namespace fr3_lab_stack::timing {
static_assert(std::atomic<std::int64_t>::is_always_lock_free);
inline std::int64_t monotonic_ns() noexcept {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<std::int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

struct TargetTiming {
  std::int64_t callback_ns{0};
  std::int64_t stamp_ns{0};
  std::uint64_t sequence{0};
  std::int64_t activation{0};
};
struct Sample {
  std::uint64_t cycle{0};
  std::int64_t activation{0};
  std::int64_t period_ns{0};
  bool applied{false};
  TargetTiming target{};
  std::int64_t application_ns{0};
  std::array<double, 7> q_desired{};
};

// Single RT producer, single non-RT consumer. Never reset while either is running.
// A full evidence ring drops the newest sample; it never delays command installation.
template <std::size_t Capacity>
class Ring {
 public:
  static_assert(Capacity > 0);
  static_assert(std::atomic<std::uint64_t>::is_always_lock_free);
  bool push(const Sample& sample) noexcept {
    const auto write = write_.load(std::memory_order_relaxed);
    if (write - read_.load(std::memory_order_acquire) == Capacity) {
      dropped_.fetch_add(1, std::memory_order_relaxed);
      if (sample.applied) dropped_applications_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    data_[write % Capacity] = sample;
    write_.store(write + 1, std::memory_order_release);
    return true;
  }
  bool pop(Sample& sample) noexcept {
    const auto read = read_.load(std::memory_order_relaxed);
    if (read == write_.load(std::memory_order_acquire)) return false;
    sample = data_[read % Capacity];
    read_.store(read + 1, std::memory_order_release);
    return true;
  }
  std::uint64_t dropped() const noexcept { return dropped_.load(); }
  std::uint64_t dropped_applications() const noexcept { return dropped_applications_.load(); }
 private:
  std::array<Sample, Capacity> data_{};
  std::atomic<std::uint64_t> write_{0}, read_{0}, dropped_{0}, dropped_applications_{0};
};

// Called only after successful core.set_target(), under the controller's existing
// new-sequence guard. Copying the command preserves callback time across buffering.
inline void latch(Sample& sample, const TargetTiming& target,
                  const std::array<double, 7>& q, std::int64_t t4) noexcept {
  sample.applied = true;
  sample.target = target;
  sample.application_ns = t4;
  sample.q_desired = q;
}
// Shared installation gate: evidence capacity cannot affect command application.
template <typename Core, typename Command>
bool install(Core& core, const Command& target, std::uint64_t& last_sequence,
             Sample& sample) noexcept {
  if (target.sequence <= last_sequence) return true;
  if (!core.set_target(target.q)) return false;
  latch(sample, target, target.q, monotonic_ns());
  last_sequence = target.sequence;
  return true;
}
}  // namespace fr3_lab_stack::timing

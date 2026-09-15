#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <franka_semantic_components/franka_robot_model.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "fr3_lab_stack/streaming_joint_impedance_core.hpp"

namespace fr3_lab_stack {

class StreamingJointImpedanceController : public controller_interface::ControllerInterface {
 public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using Vector7 = StreamingJointImpedanceCore::Vector7;
  using Vector6 = std::array<double, 6>;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

 private:
  struct TargetCommand {
    Vector7 q{};
    std::int64_t stamp_ns{0};
    std::uint64_t sequence{0};
    bool external{false};
  };

  static constexpr std::size_t kNumJoints = StreamingJointImpedanceCore::kNumJoints;
  static constexpr std::size_t kTelemetryWidth = 39;

  bool read_joint_state(Vector7* q, Vector7* dq) const noexcept;
  bool validate_joint_names(const std::vector<std::string>& names) const noexcept;
  void target_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void publish_telemetry(const rclcpp::Time& time,
                         const Vector7& q,
                         const Vector7& dq,
                         const StreamingJointImpedanceCore::Output& output);

  std::string robot_type_{"fr3"};
  std::string arm_prefix_;
  std::string arm_id_{"fr3"};
  std::vector<std::string> joint_names_;

  Vector7 k_gains_{};
  Vector7 d_gains_{};
  Vector6 cartesian_k_gains_{};
  Vector6 cartesian_d_gains_{};
  bool hybrid_impedance_enabled_{true};

  double target_max_age_s_{0.25};
  double target_future_tolerance_s_{0.05};
  double telemetry_rate_hz_{100.0};

  StreamingJointImpedanceCore core_;
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;

  realtime_tools::RealtimeBuffer<TargetCommand> target_buffer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_subscriber_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>
      state_publisher_;

  std::atomic<bool> accept_commands_{false};
  std::atomic<std::uint64_t> next_sequence_{0};
  std::uint64_t last_applied_sequence_{0};
  std::int64_t last_external_target_stamp_ns_{0};
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace fr3_lab_stack

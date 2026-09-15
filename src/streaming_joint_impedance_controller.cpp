#include "fr3_lab_stack/streaming_joint_impedance_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <unistd.h>

#include <franka/model.h>
#include <pluginlib/class_list_macros.hpp>

#include "fr3_lab_stack/hybrid_impedance_math.hpp"

namespace fr3_lab_stack {
namespace {
// Non-RT JSON string encoding, including arbitrary source frame IDs.
std::string json_string(const std::string& value) {
  std::ostringstream out;
  out << '"';
  for (const unsigned char ch : value) {
    if (ch == '"' || ch == '\\') out << '\\' << ch;
    else if (ch < 0x20) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << unsigned(ch);
    else out << ch;
  }
  out << '"';
  return out.str();
}
}  // namespace


controller_interface::InterfaceConfiguration
StreamingJointImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joint_names_) {
    config.names.push_back(joint + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
StreamingJointImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto& joint : joint_names_) {
    config.names.push_back(joint + "/position");
    config.names.push_back(joint + "/velocity");
  }

  if (hybrid_impedance_enabled_ && franka_robot_model_) {
    for (const auto& name : franka_robot_model_->get_state_interface_names()) {
      config.names.push_back(name);
    }
  }
  return config;
}

StreamingJointImpedanceController::CallbackReturn
StreamingJointImpedanceController::on_init() {
  try {
    auto_declare<std::string>("robot_type", "fr3");
    auto_declare<std::string>("arm_prefix", "");

    auto_declare<std::vector<double>>(
        "k_gains",
        std::vector<double>{40.0, 30.0, 50.0, 25.0, 35.0, 25.0, 10.0});
    auto_declare<std::vector<double>>(
        "d_gains",
        std::vector<double>{4.0, 6.0, 5.0, 5.0, 3.0, 2.0, 1.0});

    auto_declare<bool>("hybrid_impedance_enabled", true);
    auto_declare<std::vector<double>>(
        "cartesian_k_gains",
        std::vector<double>{750.0, 750.0, 750.0, 15.0, 15.0, 15.0});
    auto_declare<std::vector<double>>(
        "cartesian_d_gains",
        std::vector<double>{37.0, 37.0, 37.0, 2.0, 2.0, 2.0});

    auto_declare<double>("target_max_age_s", 0.25);
    auto_declare<double>("target_future_tolerance_s", 0.05);
    auto_declare<double>("telemetry_rate_hz", 100.0);
  } catch (const std::exception& exc) {
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "Parameter declaration failed: %s",
        exc.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

StreamingJointImpedanceController::CallbackReturn
StreamingJointImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  robot_type_ = get_node()->get_parameter("robot_type").as_string();
  arm_prefix_ = get_node()->get_parameter("arm_prefix").as_string();

  const auto k = get_node()->get_parameter("k_gains").as_double_array();
  const auto d = get_node()->get_parameter("d_gains").as_double_array();

  hybrid_impedance_enabled_ =
      get_node()->get_parameter("hybrid_impedance_enabled").as_bool();
  const auto cartesian_k =
      get_node()->get_parameter("cartesian_k_gains").as_double_array();
  const auto cartesian_d =
      get_node()->get_parameter("cartesian_d_gains").as_double_array();

  target_max_age_s_ =
      get_node()->get_parameter("target_max_age_s").as_double();
  target_future_tolerance_s_ =
      get_node()->get_parameter("target_future_tolerance_s").as_double();
  telemetry_rate_hz_ =
      get_node()->get_parameter("telemetry_rate_hz").as_double();

  if (robot_type_.empty() ||
      k.size() != kNumJoints ||
      d.size() != kNumJoints ||
      cartesian_k.size() != cartesian_k_gains_.size() ||
      cartesian_d.size() != cartesian_d_gains_.size() ||
      !std::isfinite(target_max_age_s_) ||
      target_max_age_s_ <= 0.0 ||
      !std::isfinite(target_future_tolerance_s_) ||
      target_future_tolerance_s_ < 0.0 ||
      !std::isfinite(telemetry_rate_hz_) ||
      telemetry_rate_hz_ <= 0.0 ||
      telemetry_rate_hz_ > 1000.0) {
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "Invalid streaming impedance parameters");
    return CallbackReturn::ERROR;
  }

  std::copy(k.begin(), k.end(), k_gains_.begin());
  std::copy(d.begin(), d.end(), d_gains_.begin());
  std::copy(
      cartesian_k.begin(),
      cartesian_k.end(),
      cartesian_k_gains_.begin());
  std::copy(
      cartesian_d.begin(),
      cartesian_d.end(),
      cartesian_d_gains_.begin());

  for (const double value : cartesian_k_gains_) {
    if (!std::isfinite(value) || value < 0.0) {
      RCLCPP_ERROR(
          get_node()->get_logger(),
          "cartesian_k_gains must be finite and non-negative");
      return CallbackReturn::ERROR;
    }
  }
  for (const double value : cartesian_d_gains_) {
    if (!std::isfinite(value) || value < 0.0) {
      RCLCPP_ERROR(
          get_node()->get_logger(),
          "cartesian_d_gains must be finite and non-negative");
      return CallbackReturn::ERROR;
    }
  }

  if (!core_.configure(k_gains_, d_gains_)) {
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "Joint impedance gains must be finite and non-negative");
    return CallbackReturn::ERROR;
  }

  const std::string prefix =
      arm_prefix_.empty() ? "" : arm_prefix_ + "_";
  arm_id_ = prefix + robot_type_;

  joint_names_.clear();
  joint_names_.reserve(kNumJoints);
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    joint_names_.push_back(
        arm_id_ + "_joint" + std::to_string(i + 1));
  }

  franka_robot_model_ =
      std::make_unique<franka_semantic_components::FrankaRobotModel>(
          arm_id_ + "/robot_model",
          arm_id_ + "/robot_state");

  target_subscriber_ =
      get_node()->create_subscription<sensor_msgs::msg::JointState>(
          "~/target_joint",
          rclcpp::QoS(1).reliable(),
          [this](sensor_msgs::msg::JointState::SharedPtr msg) {
            target_callback(std::move(msg));
          });

  auto publisher =
      get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
          "~/state",
          rclcpp::QoS(1).best_effort());
  state_publisher_ =
      std::make_shared<
          realtime_tools::RealtimePublisher<
              std_msgs::msg::Float64MultiArray>>(publisher);
  state_publisher_->msg_.data.assign(kTelemetryWidth, 0.0);

  char hostname[256]{};
  gethostname(hostname, sizeof(hostname) - 1);
  std::string boot;
  std::ifstream("/proc/sys/kernel/random/boot_id") >> boot;
  char time_namespace[256]{};
  const auto length = readlink("/proc/self/ns/time", time_namespace, sizeof(time_namespace) - 1);
  if (length > 0) time_namespace[length] = '\0';
  std::ostringstream provenance;
  provenance << "\"clock\":\"CLOCK_MONOTONIC\",\"hostname\":" << json_string(hostname)
             << ",\"boot_id\":" << json_string(boot)
             << ",\"time_namespace\":" << json_string(time_namespace)
             << ",\"instance\":\"" << getpid() << "-" << timing::monotonic_ns() << "\"";
  provenance_ = provenance.str();
  timing_publisher_ = get_node()->create_publisher<std_msgs::msg::String>(
      "~/timing", rclcpp::QoS(100).reliable());
  timing_timer_ = get_node()->create_wall_timer(
      std::chrono::milliseconds(20), [this]() { drain_evidence(); });

  accept_commands_.store(false);
  next_sequence_.store(0);
  last_applied_sequence_ = 0;
  last_external_target_stamp_ns_ = 0;
  last_publish_time_ =
      rclcpp::Time(0, 0, RCL_ROS_TIME);

  RCLCPP_INFO(
      get_node()->get_logger(),
      "Configured streaming joint impedance controller "
      "(hybrid=%s). Target topic: %s/target_joint",
      hybrid_impedance_enabled_ ? "true" : "false",
      get_node()->get_fully_qualified_name());

  return CallbackReturn::SUCCESS;
}

StreamingJointImpedanceController::CallbackReturn
StreamingJointImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (hybrid_impedance_enabled_) {
    if (!franka_robot_model_) {
      RCLCPP_ERROR(
          get_node()->get_logger(),
          "Cannot activate: Franka robot model is unavailable");
      return CallbackReturn::ERROR;
    }
    franka_robot_model_->assign_loaned_state_interfaces(
        state_interfaces_);
  }

  Vector7 q{};
  Vector7 dq{};
  if (!read_joint_state(&q, &dq)) {
    if (hybrid_impedance_enabled_ && franka_robot_model_) {
      franka_robot_model_->release_interfaces();
    }
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "Cannot activate: invalid FR3 joint state");
    return CallbackReturn::ERROR;
  }

  if (!core_.activate(q)) {
    if (hybrid_impedance_enabled_ && franka_robot_model_) {
      franka_robot_model_->release_interfaces();
    }
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "Cannot activate streaming impedance core");
    return CallbackReturn::ERROR;
  }

  if (hybrid_impedance_enabled_) {
    try {
      const auto jacobian =
          franka_robot_model_->getZeroJacobian(
              franka::Frame::kEndEffector);
      const auto coriolis =
          franka_robot_model_->getCoriolisForceVector();

      for (const double value : jacobian) {
        if (!std::isfinite(value)) {
          throw std::runtime_error(
              "non-finite Jacobian at activation");
        }
      }
      for (const double value : coriolis) {
        if (!std::isfinite(value)) {
          throw std::runtime_error(
              "non-finite Coriolis vector at activation");
        }
      }
    } catch (const std::exception& exc) {
      core_.deactivate();
      franka_robot_model_->release_interfaces();
      RCLCPP_ERROR(
          get_node()->get_logger(),
          "Cannot activate hybrid impedance: %s",
          exc.what());
      return CallbackReturn::ERROR;
    }
  }

  activation_.store(timing::monotonic_ns());
  TargetCommand initial_hold;
  initial_hold.q = q;
  initial_hold.stamp_ns =
      get_node()->now().nanoseconds();
  initial_hold.sequence = 0;
  initial_hold.external = false;
  target_buffer_.writeFromNonRT(initial_hold);

  next_sequence_.store(0);
  last_applied_sequence_ = 0;
  last_external_target_stamp_ns_ = 0;
  last_publish_time_ =
      rclcpp::Time(0, 0, RCL_ROS_TIME);

  for (auto& command_interface : command_interfaces_) {
    if (!command_interface.set_value(0.0)) {
      RCLCPP_ERROR(
          get_node()->get_logger(),
          "Failed to initialize effort command interface");
      core_.deactivate();
      if (hybrid_impedance_enabled_ && franka_robot_model_) {
        franka_robot_model_->release_interfaces();
      }
      return CallbackReturn::ERROR;
    }
  }

  accept_commands_.store(true);
  RCLCPP_INFO(
      get_node()->get_logger(),
      "Activated with q_desired equal to the measured activation "
      "pose; no autonomous target is generated");
  return CallbackReturn::SUCCESS;
}

StreamingJointImpedanceController::CallbackReturn
StreamingJointImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  accept_commands_.store(false);
  core_.deactivate();

  if (hybrid_impedance_enabled_ && franka_robot_model_) {
    franka_robot_model_->release_interfaces();
  }

  RCLCPP_INFO(
      get_node()->get_logger(),
      "Deactivated streaming joint impedance controller");
  return CallbackReturn::SUCCESS;
}

StreamingJointImpedanceController::CallbackReturn
StreamingJointImpedanceController::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  accept_commands_.store(false);
  core_.deactivate();
  timing_timer_.reset();
  drain_evidence();
  timing_publisher_.reset();
  target_subscriber_.reset();
  state_publisher_.reset();
  franka_robot_model_.reset();
  return CallbackReturn::SUCCESS;
}

bool StreamingJointImpedanceController::read_joint_state(
    Vector7* q,
    Vector7* dq) const noexcept {
  if (q == nullptr ||
      dq == nullptr ||
      state_interfaces_.size() < 2 * kNumJoints) {
    return false;
  }

  for (std::size_t i = 0; i < kNumJoints; ++i) {
    const auto q_value =
        state_interfaces_.at(2 * i).get_optional();
    const auto dq_value =
        state_interfaces_.at(2 * i + 1).get_optional();

    if (!q_value.has_value() ||
        !dq_value.has_value() ||
        !std::isfinite(*q_value) ||
        !std::isfinite(*dq_value)) {
      return false;
    }

    (*q)[i] = *q_value;
    (*dq)[i] = *dq_value;
  }
  return true;
}

bool StreamingJointImpedanceController::validate_joint_names(
    const std::vector<std::string>& names) const noexcept {
  return names == joint_names_;
}

void StreamingJointImpedanceController::target_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  const auto t3 = timing::monotonic_ns();
  const auto activation = activation_.load();
  const auto receipt_ros_ns = get_node()->now().nanoseconds();
  const auto emit = [&](const char* outcome, std::uint64_t sequence = 0) {
    std::ostringstream fields;
    fields << "\"event\":\"callback\",\"activation\":" << activation
           << ",\"t3_ns\":" << t3 << ",\"receipt_ros_ns\":" << receipt_ros_ns
           << ",\"source_stamp_ns\":"
           << (msg ? static_cast<std::int64_t>(msg->header.stamp.sec) * 1000000000LL + msg->header.stamp.nanosec : 0)
           << ",\"source_frame_id\":" << json_string(msg ? msg->header.frame_id : std::string{})
           << ",\"sequence\":" << sequence << ",\"outcome\":" << json_string(outcome);
    publish_evidence(fields.str());
  };
  if (!accept_commands_.load()) {
    RCLCPP_WARN(
        get_node()->get_logger(),
        "Ignoring joint target while controller is inactive");
    emit("inactive");
    return;
  }

  if (!msg ||
      !validate_joint_names(msg->name) ||
      msg->position.size() != kNumJoints) {
    RCLCPP_WARN(
        get_node()->get_logger(),
        "Rejected joint target: expected exactly seven ordered "
        "names and seven positions");
    emit("joint_shape");
    return;
  }

  TargetCommand command;
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (!std::isfinite(msg->position[i])) {
      RCLCPP_WARN(
          get_node()->get_logger(),
          "Rejected joint target: non-finite position");
      emit("nonfinite_position");
      return;
    }
    command.q[i] = msg->position[i];
  }

  const rclcpp::Time stamp(
      msg->header.stamp,
      get_node()->get_clock()->get_clock_type());
  if (stamp.nanoseconds() <= 0) {
    RCLCPP_WARN(
        get_node()->get_logger(),
        "Rejected joint target: header stamp is required");
    emit("invalid_stamp");
    return;
  }

  const double age_s =
      (get_node()->now() - stamp).seconds();
  if (!std::isfinite(age_s) ||
      age_s > target_max_age_s_ ||
      age_s < -target_future_tolerance_s_) {
    RCLCPP_WARN(
        get_node()->get_logger(),
        "Rejected joint target: stamp age %.6f s outside "
        "[-%.6f, %.6f] s",
        age_s,
        target_future_tolerance_s_,
        target_max_age_s_);
    emit("stamp_age");
    return;
  }

  command.callback_ns = t3;
  command.activation = activation;
  command.stamp_ns = stamp.nanoseconds();
  command.sequence =
      next_sequence_.fetch_add(1) + 1;
  command.external = true;
  target_buffer_.writeFromNonRT(command);
  emit("accepted", command.sequence);
}

controller_interface::return_type
StreamingJointImpedanceController::update(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) {
  timing::Sample evidence;
  evidence.cycle = ++cycle_;
  evidence.activation = activation_.load();
  evidence.period_ns = period.nanoseconds();
  // Enqueue on every exit, including update errors. No heap work or blocking.
  struct Capture {
    timing::Ring<8192>& ring;
    timing::Sample& sample;
    ~Capture() { ring.push(sample); }
  } capture{evidence_, evidence};
  Vector7 q{};
  Vector7 dq{};
  if (!read_joint_state(&q, &dq)) {
    return controller_interface::return_type::ERROR;
  }

  const TargetCommand* target =
      target_buffer_.readFromRT();
  if (target != nullptr &&
      target->sequence > last_applied_sequence_) {
    if (!timing::install(core_, *target, last_applied_sequence_, evidence)) {
      return controller_interface::return_type::ERROR;
    }

    if (target->external) {
      last_external_target_stamp_ns_ =
          target->stamp_ns;
    }
  }

  StreamingJointImpedanceCore::Output output;
  if (!core_.compute(q, dq, &output)) {
    return controller_interface::return_type::ERROR;
  }

  if (hybrid_impedance_enabled_) {
    if (!franka_robot_model_) {
      return controller_interface::return_type::ERROR;
    }

    try {
      const auto jacobian_array =
          franka_robot_model_->getZeroJacobian(
              franka::Frame::kEndEffector);
      const auto coriolis_array =
          franka_robot_model_->getCoriolisForceVector();

      HybridImpedanceMath::Vector7 cartesian_feedback{};
      HybridImpedanceMath::Vector7 additional_effort{};

      if (!HybridImpedanceMath::compute_additional(
              output.tracking_error,
              dq,
              cartesian_k_gains_,
              cartesian_d_gains_,
              jacobian_array,
              coriolis_array,
              &cartesian_feedback,
              &additional_effort)) {
        return controller_interface::return_type::ERROR;
      }

      for (std::size_t i = 0; i < kNumJoints; ++i) {
        output.commanded_effort[i] +=
            additional_effort[i];
        if (!std::isfinite(output.commanded_effort[i])) {
          return controller_interface::return_type::ERROR;
        }
      }
    } catch (const std::exception&) {
      return controller_interface::return_type::ERROR;
    }
  }

  if (command_interfaces_.size() != kNumJoints) {
    return controller_interface::return_type::ERROR;
  }

  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (!command_interfaces_[i].set_value(
            output.commanded_effort[i])) {
      return controller_interface::return_type::ERROR;
    }
  }

  publish_telemetry(time, q, dq, output);
  return controller_interface::return_type::OK;
}

void StreamingJointImpedanceController::publish_evidence(const std::string& fields) {
  if (!timing_publisher_) return;
  std_msgs::msg::String msg;
  msg.data = "{\"schema\":1," + provenance_ + ",\"evidence_id\":" +
      std::to_string(++evidence_id_) + ",\"publication_errors\":" +
      std::to_string(timing_publication_errors_) + "," + fields + "}";
  try {
    timing_publisher_->publish(msg);
  } catch (const std::exception&) {
    // Evidence publication failure must not escape into the command callback.
    // The next successful record exposes both the ID gap and cumulative count.
    ++timing_publication_errors_;
  }
}

void StreamingJointImpedanceController::drain_evidence() {
  std::ostringstream fields;
  fields << std::setprecision(17)
         << "\"event\":\"samples\",\"activation\":" << activation_.load()
         << ",\"samples\":[";
  timing::Sample sample;
  bool first = true;
  // Bound non-RT work too; arrivals during draining cannot extend it indefinitely.
  for (std::size_t i = 0; i < 8192 && evidence_.pop(sample); ++i) {
    if (!first) fields << ',';
    first = false;
    fields << "{\"cycle\":" << sample.cycle
           << ",\"activation\":" << sample.activation
           << ",\"period_ns\":" << sample.period_ns;
    if (sample.applied) {
      fields << ",\"application\":{\"activation\":" << sample.activation
             << ",\"callback_activation\":" << sample.target.activation
             << ",\"sequence\":" << sample.target.sequence
             << ",\"source_stamp_ns\":" << sample.target.stamp_ns
             << ",\"t3_ns\":" << sample.target.callback_ns
             << ",\"t4_ns\":" << sample.application_ns << ",\"q_desired\":[";
      for (std::size_t j = 0; j < 7; ++j) {
        if (j) fields << ',';
        fields << sample.q_desired[j];
      }
      fields << "]}";
    }
    fields << '}';
  }
  fields << "],\"dropped_period_samples\":" << evidence_.dropped()
         << ",\"dropped_application_records\":" << evidence_.dropped_applications();
  publish_evidence(fields.str());
}

void StreamingJointImpedanceController::publish_telemetry(
    const rclcpp::Time& time,
    const Vector7& q,
    const Vector7& dq,
    const StreamingJointImpedanceCore::Output& output) {
  if (!state_publisher_) {
    return;
  }

  const double publish_period_s =
      1.0 / telemetry_rate_hz_;

  if (last_publish_time_.nanoseconds() != 0 &&
      (time - last_publish_time_).seconds() <
          publish_period_s) {
    return;
  }

  if (!state_publisher_->trylock()) {
    return;
  }

  auto& data = state_publisher_->msg_.data;

  // Fixed telemetry schema:
  // [0:7)   measured q
  // [7:14)  measured dq
  // [14:21) desired q
  // [21:28) tracking error (desired - measured)
  // [28:35) final commanded effort
  // [35]    controller time [s]
  // [36]    age of last external target stamp [s],
  //         or -1 before any external target
  // [37]    last applied external target sequence
  //         (0 means activation hold only)
  // [38]    command acceptance enabled (1/0)
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    data[i] = q[i];
    data[7 + i] = dq[i];
    data[14 + i] = output.desired_q[i];
    data[21 + i] = output.tracking_error[i];
    data[28 + i] = output.commanded_effort[i];
  }

  data[35] = time.seconds();
  data[36] =
      last_external_target_stamp_ns_ > 0
          ? static_cast<double>(
                time.nanoseconds() -
                last_external_target_stamp_ns_) *
                1e-9
          : -1.0;
  data[37] =
      static_cast<double>(last_applied_sequence_);
  data[38] =
      accept_commands_.load() ? 1.0 : 0.0;

  state_publisher_->unlockAndPublish();
  last_publish_time_ = time;
}

}  // namespace fr3_lab_stack

PLUGINLIB_EXPORT_CLASS(
    fr3_lab_stack::StreamingJointImpedanceController,
    controller_interface::ControllerInterface)

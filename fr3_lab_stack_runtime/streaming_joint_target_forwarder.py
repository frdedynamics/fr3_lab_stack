# Copyright 2026 frdedynamics
# SPDX-License-Identifier: Apache-2.0
"""C1-A1: immediate, validated JointState forwarding with expiring active evidence."""

import json
import math
import time
from collections import Counter

import rclpy
from controller_manager_msgs.srv import ListControllers
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

CONTROLLER = 'fr3_streaming_joint_impedance_controller'
JOINTS = [f'fr3_joint{i}' for i in range(1, 8)]
DEFAULTS = dict(target_max_age_s=0.25, target_future_tolerance_s=0.05,
                controller_poll_interval_s=0.10, controller_evidence_expiry_s=0.30)


def endpoint_qos():
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)


def rejection_reason(msg, receipt_ns, settings):
    if list(msg.name) != JOINTS:
        return 'joint_names'
    if len(msg.position) != 7:
        return 'position_count'
    if not all(math.isfinite(p) for p in msg.position):
        return 'nonfinite_position'
    stamp = msg.header.stamp
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        return 'invalid_timestamp'
    source_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
    if source_ns <= 0:
        return 'invalid_timestamp'
    age_ns = receipt_ns - source_ns
    if age_ns > round(settings['target_max_age_s'] * 1e9):
        return 'stale_timestamp'
    if age_ns < -round(settings['target_future_tolerance_s'] * 1e9):
        return 'future_timestamp'
    return None


class StreamingJointTargetForwarder(Node):
    """Run with the single-threaded executor used by main to serialize callbacks."""

    def __init__(self):
        super().__init__('fr3_streaming_joint_target_forwarder')
        self.settings = {}
        for name, default in DEFAULTS.items():
            value = self.declare_parameter(
                name, default, ParameterDescriptor(read_only=True)).value
            if not math.isfinite(value) or value < 0 or (
                    value == 0 and name != 'target_future_tolerance_s'):
                raise ValueError(f'{name} must be finite and positive (future tolerance may be zero)')
            self.settings[name] = value
        self.counts = dict(received=0, accepted=0, rejected=0, forwarded=0,
                           publication_errors=0)
        self.rejections = Counter()
        self.evidence_ns = None
        self.pending = None
        self.controller_status = 'unknown'
        self.publisher = self.create_publisher(
            JointState, f'/{CONTROLLER}/target_joint', endpoint_qos())
        self.telemetry = self.create_publisher(
            String, '~/telemetry', endpoint_qos())
        self.subscription = self.create_subscription(
            JointState, '/fr3_streaming_joint_target', self.on_target, endpoint_qos())
        self.client = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.poll_timer = self.create_timer(
            self.settings['controller_poll_interval_s'], self.poll,
            clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(json.dumps({'event': 'configuration', **self.settings}))

    def poll(self):
        if self.pending is not None:
            return
        if not self.client.service_is_ready():
            self.evidence_ns = None
            self.controller_status = 'service_unavailable'
            return
        sent_ns = time.monotonic_ns()
        try:
            self.pending = self.client.call_async(ListControllers.Request())
        except Exception as exc:
            self.evidence_ns = None
            self.controller_status = 'request_error'
            self.get_logger().warning(f'ListControllers request failed: {exc}')
            return
        self.pending.add_done_callback(lambda future: self.on_controllers(future, sent_ns))

    def on_controllers(self, future, sent_ns):
        self.pending = None
        self.evidence_ns = None
        try:
            matches = [c for c in future.result().controller if c.name == CONTROLLER]
            active = len(matches) == 1 and matches[0].state == 'active'
            self.controller_status = 'active' if active else 'inactive_or_missing'
            if active:
                self.evidence_ns = sent_ns
        except Exception as exc:
            self.controller_status = 'response_error'
            self.get_logger().warning(f'ListControllers response failed: {exc}')

    def on_target(self, msg):
        receipt_mono_ns = time.monotonic_ns()
        receipt_ros_ns = self.get_clock().now().nanoseconds
        self.counts['received'] += 1
        record = dict(source_stamp_sec=msg.header.stamp.sec,
                      source_stamp_nanosec=msg.header.stamp.nanosec,
                      server_receipt_ros_ns=receipt_ros_ns,
                      server_receipt_monotonic_ns=receipt_mono_ns,
                      publication_call_start_monotonic_ns=None,
                      publication_call_end_monotonic_ns=None,
                      publication_call_duration_ns=None)
        reason = rejection_reason(msg, receipt_ros_ns, self.settings)
        evidence_age_ns = (None if self.evidence_ns is None else
                           receipt_mono_ns - self.evidence_ns)
        if reason is None:
            if evidence_age_ns is None:
                reason = 'controller_active_unconfirmed'
            elif not 0 <= evidence_age_ns <= round(
                    self.settings['controller_evidence_expiry_s'] * 1e9):
                reason = 'controller_evidence_expired'
        if reason:
            self.counts['rejected'] += 1
            self.rejections[reason] += 1
            record['outcome'] = 'rejected'
        else:
            self.counts['accepted'] += 1
            start_ns = time.monotonic_ns()
            try:
                self.publisher.publish(msg)
            except Exception as exc:
                end_ns = time.monotonic_ns()
                self.counts['publication_errors'] += 1
                record.update(outcome='publication_error', publication_error=str(exc))
            else:
                end_ns = time.monotonic_ns()
                self.counts['forwarded'] += 1
                record['outcome'] = 'forwarded'
            record.update(publication_call_start_monotonic_ns=start_ns,
                          publication_call_end_monotonic_ns=end_ns,
                          publication_call_duration_ns=end_ns - start_ns)
        record.update(rejection_reason=reason, counts=dict(self.counts),
                      rejection_counts=dict(self.rejections), parameters=self.settings,
                      controller_status=self.controller_status,
                      controller_evidence_age_ns=evidence_age_ns,
                      server_processing_duration_ns=time.monotonic_ns() - receipt_mono_ns)
        # Telemetry failure is distinct from failure to publish the target.
        payload = json.dumps(record, allow_nan=False)
        self.get_logger().info(payload)
        try:
            self.telemetry.publish(String(data=payload))
        except Exception as exc:
            self.get_logger().error(f'Telemetry publication failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = StreamingJointTargetForwarder()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()

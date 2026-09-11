# Copyright 2026 frdedynamics
# SPDX-License-Identifier: Apache-2.0
"""Persistent one-goal-at-a-time FR3 joint-target action server."""

import json
import math
from threading import Lock
from types import SimpleNamespace

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from fr3_lab_stack_interfaces.action import ExecuteJointTarget

from .joint_target import (
    RosTransport,
    health_evidence,
    health_gate,
    joint_state,
    new_evidence,
    plain,
    require,
    validate_args,
    vector,
)


ACTION_NAME = '/fr3_joint_target'


class BusyGate:
    """Reserve one server slot; concurrent goals are rejected rather than queued."""

    def __init__(self):
        self._lock = Lock()
        self._busy = False

    def reserve(self):
        with self._lock:
            if self._busy:
                return False
            self._busy = True
            return True

    def release(self):
        with self._lock:
            self._busy = False

    @property
    def busy(self):
        with self._lock:
            return self._busy


def stamp_age_seconds(stamp, now_ns):
    sec = int(stamp.sec)
    nanosec = int(stamp.nanosec)
    require(sec >= 0 and 0 <= nanosec < 1_000_000_000,
            'Invalid reference timestamp fields')
    stamp_ns = sec * 1_000_000_000 + nanosec
    require(stamp_ns > 0, 'reference_stamp must be nonzero')
    age = (int(now_ns) - stamp_ns) / 1e9
    require(age >= 0, f'Future reference_stamp: age={age:.6f}s')
    return age


def validate_goal_request(request, now_ns, freshness):
    require(isinstance(request.request_id, str) and request.request_id.strip(),
            'request_id must be nonempty')
    reference_q = vector(request.reference_q, 'reference_q')
    target_q = vector(request.target_q, 'target_q')
    require(math.isfinite(freshness) and freshness > 0,
            'freshness must be finite and positive')
    age = stamp_age_seconds(request.reference_stamp, now_ns)
    require(age <= freshness,
            f'Stale reference_stamp: age={age:.6f}s > {freshness:.6f}s')
    return dict(reference_q=reference_q, target_q=target_q, reference_age_s=age)


def build_runtime_args(request, settings):
    args = SimpleNamespace(
        target=vector(request.target_q, 'target_q'),
        delta=None,
        execute=bool(settings['execute']),
        output=None,
        velocity_scale=float(settings['velocity_scale']),
        acceleration_scale=float(settings['acceleration_scale']),
        goal_tolerance=float(settings['goal_tolerance']),
        planning_time=float(settings['planning_time']),
        stationary_threshold=float(settings['stationary_threshold']),
        freshness=float(settings['freshness']),
        timeout=float(settings['timeout']),
        execution_timeout=float(settings['execution_timeout']),
        reference_q=vector(request.reference_q, 'reference_q'),
        reference_tolerance=float(settings['reference_tolerance']),
        node_name='fr3_joint_target_worker',
    )
    validate_args(args)
    return args


def evidence_code(value):
    if isinstance(value, dict):
        return int(value.get('val', 0) or 0)
    return 0


def fill_result(result, evidence, final_q, final_dq):
    result.success = evidence.get('outcome') != 'failed'
    result.outcome = str(evidence.get('outcome', 'failed'))
    result.final_q = vector(final_q, 'final_q')
    result.final_dq = vector(final_dq, 'final_dq')
    result.planning_error_code = evidence_code(evidence.get('planning_error'))
    result.execution_attempted = bool(evidence.get('execution_attempted', False))
    result.execution_action_status = int(evidence.get('execution_action_status') or 0)
    result.execution_error_code = evidence_code(evidence.get('execution_error'))
    result.evidence_json = json.dumps(plain(evidence), allow_nan=False, sort_keys=True)
    return result


class JointTargetServer(Node):
    def __init__(self):
        super().__init__('fr3_joint_target_server')
        self.declare_parameter('execute', False)
        self.declare_parameter('velocity_scale', 0.1)
        self.declare_parameter('acceleration_scale', 0.1)
        self.declare_parameter('goal_tolerance', 1e-4)
        self.declare_parameter('planning_time', 2.0)
        self.declare_parameter('stationary_threshold', 0.01)
        self.declare_parameter('freshness', 0.25)
        self.declare_parameter('timeout', 5.0)
        self.declare_parameter('execution_timeout', 5.0)
        self.declare_parameter('reference_tolerance', 0.002)

        self._gate = BusyGate()
        self._callback_group = ReentrantCallbackGroup()
        self._server = ActionServer(
            self,
            ExecuteJointTarget,
            ACTION_NAME,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            'FR3 joint-target server ready; execute=%s' %
            self.get_parameter('execute').value
        )

    def settings(self):
        names = (
            'execute',
            'velocity_scale',
            'acceleration_scale',
            'goal_tolerance',
            'planning_time',
            'stationary_threshold',
            'freshness',
            'timeout',
            'execution_timeout',
            'reference_tolerance',
        )
        return {name: self.get_parameter(name).value for name in names}

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def goal_callback(self, goal_request):
        try:
            validate_goal_request(
                goal_request,
                self.now_ns(),
                float(self.get_parameter('freshness').value),
            )
        except ValueError as exc:
            self.get_logger().warning(f'Rejecting invalid joint-target goal: {exc}')
            return GoalResponse.REJECT

        if not self._gate.reserve():
            self.get_logger().warning('Rejecting joint-target goal: server is busy')
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        # First version deliberately has no partial-motion cancellation contract.
        # Runtime exceptions/timeouts can still cancel the underlying MoveIt goal.
        return CancelResponse.REJECT

    def publish_final_feedback(self, goal_handle, stage, q, dq):
        feedback = ExecuteJointTarget.Feedback()
        feedback.stage = str(stage)
        feedback.measured_q = vector(q, 'feedback_q')
        feedback.measured_dq = vector(dq, 'feedback_dq')
        goal_handle.publish_feedback(feedback)

    def execute_callback(self, goal_handle):
        request = goal_handle.request
        evidence = new_evidence()
        evidence['request_id'] = request.request_id
        evidence['reference_stamp'] = plain(request.reference_stamp)
        evidence['server_action_name'] = ACTION_NAME

        worker = None
        final_q = [0.0] * 7
        final_dq = [0.0] * 7

        try:
            settings = self.settings()
            validated = validate_goal_request(
                request, self.now_ns(), float(settings['freshness']))
            evidence['reference_age_at_execution_s'] = validated['reference_age_s']
            evidence['server_settings'] = dict(settings)

            args = build_runtime_args(request, settings)
            worker = RosTransport(args, evidence)
            worker.run()

            # Plan-only mode does not enter the runtime execution-monitoring block,
            # so obtain one fresh measured state for the action result as well.
            if 'measured' in evidence and 'final_q' in evidence['measured']:
                final_q = evidence['measured']['final_q']
                final_dq = evidence['measured']['final_dq']
            else:
                js, fs = worker.fresh()
                final_q, final_dq = joint_state(js)
                health_gate(fs)
                evidence['server_final_state'] = dict(
                    q=final_q,
                    dq=final_dq,
                    franka=health_evidence(fs),
                )

            self.publish_final_feedback(
                goal_handle, evidence.get('outcome', 'completed'), final_q, final_dq)
            goal_handle.succeed()

        except (Exception, KeyboardInterrupt) as exc:
            evidence['outcome'] = 'failed'
            evidence['error'] = str(exc) or type(exc).__name__
            if worker is not None:
                try:
                    js, fs = worker.fresh()
                    final_q, final_dq = joint_state(js)
                    evidence['server_failure_state'] = dict(
                        q=final_q,
                        dq=final_dq,
                        franka=health_evidence(fs),
                    )
                except Exception as state_exc:
                    evidence['server_failure_state_error'] = str(state_exc)
            goal_handle.abort()
            self.get_logger().error(
                f'Joint-target request {request.request_id!r} failed: {evidence["error"]}')

        finally:
            if worker is not None:
                worker.node.destroy_node()
            self._gate.release()

        result = ExecuteJointTarget.Result()
        return fill_result(result, evidence, final_q, final_dq)

    def destroy_node(self):
        self._server.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JointTargetServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

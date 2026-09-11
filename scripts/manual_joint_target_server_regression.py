#!/usr/bin/python3
# Copyright 2026 frdedynamics
# SPDX-License-Identifier: Apache-2.0
"""Manual regression client for the validated FR3 joint-target action server.

This is a commissioning helper, not the production SAPS client. It always uses
the known +0.01 rad J1 target constructed from a fresh /joint_states sample.

The helper queries the server's ``execute`` parameter first. If physical
execution is enabled, ``--allow-execution`` is required before any goal is sent.
"""

import argparse
import json
from pathlib import Path
import time

import rclpy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from fr3_lab_stack_interfaces.action import ExecuteJointTarget


JOINTS = [f"fr3_joint{i}" for i in range(1, 8)]
ACTION_NAME = "/fr3_joint_target"
SERVER_PARAMETER_SERVICE = "/fr3_joint_target_server/get_parameters"


def parser():
    p = argparse.ArgumentParser(
        description=(
            "Send the known +0.01 rad J1 commissioning request to the "
            "FR3 joint-target action server."
        )
    )
    p.add_argument(
        "--allow-execution",
        action="store_true",
        help=(
            "Permit sending the goal when the server reports execute=true. "
            "Without this flag, physical execution is refused."
        ),
    )
    p.add_argument(
        "--output",
        default="/tmp/fr3_joint_target_server_regression.json",
        help="Evidence JSON output path.",
    )
    p.add_argument("--server-timeout", type=float, default=5.0)
    p.add_argument("--state-timeout", type=float, default=5.0)
    p.add_argument("--result-timeout", type=float, default=15.0)
    return p


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def spin_future(node, future, timeout, label):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    require(future.done(), f"Timed out waiting for {label}")
    return future.result()


def server_execute_enabled(node, timeout):
    client = node.create_client(GetParameters, SERVER_PARAMETER_SERVICE)
    require(
        client.wait_for_service(timeout_sec=timeout),
        f"Parameter service unavailable: {SERVER_PARAMETER_SERVICE}",
    )
    request = GetParameters.Request()
    request.names = ["execute"]
    response = spin_future(
        node, client.call_async(request), timeout, "server execute parameter"
    )
    require(len(response.values) == 1, "Unexpected execute-parameter response")
    value = response.values[0]
    require(
        value.type == ParameterType.PARAMETER_BOOL,
        "Server execute parameter is not boolean",
    )
    return bool(value.bool_value)


def fresh_joint_state(node, timeout):
    latest = {"msg": None}

    def callback(msg):
        latest["msg"] = msg

    sub = node.create_subscription(
        JointState, "/joint_states", callback, qos_profile_sensor_data
    )
    deadline = time.monotonic() + timeout
    while (
        rclpy.ok()
        and latest["msg"] is None
        and time.monotonic() < deadline
    ):
        rclpy.spin_once(node, timeout_sec=0.05)

    node.destroy_subscription(sub)
    require(latest["msg"] is not None, "No /joint_states received")
    return latest["msg"]


def arm_q(msg):
    names = list(msg.name)
    require(set(JOINTS).issubset(names), f"Missing FR3 joints: {names}")
    require(len(msg.position) == len(names), "Incomplete joint positions")
    return [float(msg.position[names.index(j)]) for j in JOINTS]


def max_abs(values):
    return max(abs(float(v)) for v in values) if values else 0.0


def validate_result(result, evidence, execute_enabled):
    require(result.success, f"Server returned failure: {result.outcome}")
    require(result.planning_error_code == 1, "MoveIt planning did not succeed")

    planned = evidence.get("planned") or {}
    endpoint_error = planned.get("endpoint_error") or []
    require(
        endpoint_error and max_abs(endpoint_error) <= 1e-4,
        "Planned endpoint error exceeds 1e-4 rad",
    )

    reference_error = evidence.get("reference_error") or []
    require(
        reference_error and max_abs(reference_error) <= 0.002,
        "Reference-state mismatch exceeds 0.002 rad",
    )

    if execute_enabled:
        require(result.outcome == "execution_succeeded", "Unexpected execution outcome")
        require(result.execution_attempted, "Execution was not attempted")
        require(
            evidence.get("execution_attempts") == 1,
            "Expected exactly one execution attempt",
        )
        require(result.execution_action_status == 4, "ExecuteTrajectory did not succeed")
        require(result.execution_error_code == 1, "MoveIt execution did not succeed")

        measured = evidence.get("measured") or {}
        final_error = measured.get("final_tracking_error") or []
        final_dq = measured.get("final_dq") or []
        require(
            final_error and max_abs(final_error) <= 0.01,
            "Final tracking error exceeds 0.01 rad",
        )
        require(
            final_dq and max_abs(final_dq) <= 0.02,
            "Final measured velocity exceeds 0.02 rad/s",
        )
        require(
            not measured.get("telemetry_errors"),
            f"Telemetry errors recorded: {measured.get('telemetry_errors')}",
        )
    else:
        require(result.outcome == "plan_only_validated", "Unexpected plan-only outcome")
        require(not result.execution_attempted, "Plan-only server attempted execution")
        require(
            evidence.get("execution_attempts") == 0,
            "Plan-only server recorded an execution attempt",
        )


def main(argv=None):
    args = parser().parse_args(argv)

    rclpy.init(args=[])
    node = rclpy.create_node(
        "fr3_joint_target_manual_regression_client",
        use_global_arguments=False,
    )

    try:
        action = ActionClient(node, ExecuteJointTarget, ACTION_NAME)
        require(
            action.wait_for_server(timeout_sec=args.server_timeout),
            f"Action server unavailable: {ACTION_NAME}",
        )

        execute_enabled = server_execute_enabled(node, args.server_timeout)
        print(f"server execute={execute_enabled}")

        if execute_enabled and not args.allow_execution:
            raise RuntimeError(
                "Server has execute=true; refusing physical motion without "
                "--allow-execution"
            )

        msg = fresh_joint_state(node, args.state_timeout)
        reference_q = arm_q(msg)
        target_q = reference_q.copy()
        target_q[0] += 0.01

        goal = ExecuteJointTarget.Goal()
        goal.request_id = f"manual-regression-{time.time_ns()}"
        goal.reference_q = reference_q
        goal.target_q = target_q
        goal.reference_stamp = msg.header.stamp

        print("request_id:", goal.request_id)
        print("reference_q:", reference_q)
        print("target_q:", target_q)
        print("J1 delta:", target_q[0] - reference_q[0])

        def feedback_cb(feedback_msg):
            print("feedback stage:", feedback_msg.feedback.stage)

        handle = spin_future(
            node,
            action.send_goal_async(goal, feedback_callback=feedback_cb),
            args.server_timeout,
            "goal response",
        )
        require(handle.accepted, "Goal was rejected")
        print("goal accepted")

        wrapped = spin_future(
            node,
            handle.get_result_async(),
            args.result_timeout,
            "action result",
        )
        result = wrapped.result
        evidence = json.loads(result.evidence_json)

        validate_result(result, evidence, execute_enabled)

        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(evidence, indent=2) + "\n")

        print()
        print("=== RESULT ===")
        print("action status:", wrapped.status)
        print("success:", result.success)
        print("outcome:", result.outcome)
        print("planning_error_code:", result.planning_error_code)
        print("execution_attempted:", result.execution_attempted)
        print("execution_action_status:", result.execution_action_status)
        print("execution_error_code:", result.execution_error_code)
        print("final_q:", list(result.final_q))
        print("final_dq:", list(result.final_dq))
        print("evidence:", output)
        print()
        print("REGRESSION PASSED")

        return 0
    except (RuntimeError, ValueError, json.JSONDecodeError) as exc:
        print(f"ERROR: {exc}")
        return 2
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

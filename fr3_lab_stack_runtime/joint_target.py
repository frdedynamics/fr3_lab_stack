# Copyright 2026 frdedynamics
# SPDX-License-Identifier: Apache-2.0
"""Plan an absolute FR3 joint target; motion requires --execute.

Uses GetMotionPlan and, optionally, one ExecuteTrajectory goal. Runtime limits
come from /move_group robot_description and robot_description_planning.joint_limits.
Graph checks are conservative snapshots, not an exclusive command lock: the
operator must keep other command clients stopped for the entire invocation.

Health policy: IDLE/MOVE only, no current Franka errors and no active
collision indicators. Contact, last-motion errors, and command-success rate are
recorded as evidence but are not independent rejection conditions. State stamps
must use the same ROS clock as this client. A fresh sample means a callback
received after the request plus a bounded source/receipt age; its source stamp
does not have to be later than the client's request timestamp. Only optional
FR3 finger joints are ignored when normalizing /joint_states. Plans are checked
at every supplied waypoint;
reported derivative peaks are waypoint peaks, not controller interpolation or
measured motion. Start agreement is 1e-6 rad; execution drift is <= 0.002 rad.
"""

import argparse
from collections.abc import Mapping
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

JOINTS = tuple(f'fr3_joint{i}' for i in range(1, 8))
CONTROLLER = 'fr3_arm_controller'


def require(condition, message):
    if not condition:
        raise ValueError(message)


def vector(values, label='target'):
    result = [float(v) for v in values]
    require(len(result) == 7, f'{label}: expected seven values')
    require(all(math.isfinite(v) for v in result), f'{label}: nonfinite value')
    return result


def joint_state(msg):
    names = list(msg.name)
    require(len(names) == len(set(names)), 'Duplicate joint names')
    # /joint_states can also contain the two gripper joints. No other joints allowed.
    require(set(JOINTS) <= set(names), 'Missing FR3 arm joints')
    require(set(names) <= set(JOINTS) | {'fr3_finger_joint1', 'fr3_finger_joint2'},
            'Unexpected joints in joint state')
    require(len(msg.position) == len(names) == len(msg.velocity),
            'Incomplete joint positions/velocities')
    require(all(math.isfinite(v) for v in list(msg.position) + list(msg.velocity)),
            'Nonfinite joint telemetry')
    return (vector([msg.position[names.index(j)] for j in JOINTS], 'q'),
            vector([msg.velocity[names.index(j)] for j in JOINTS], 'dq'))


def stationary(dq, threshold):
    require(max(abs(v) for v in vector(dq, 'dq')) <= threshold,
            'Robot is not stationary')


def drift_check(start, current, tolerance=0.002):
    drift = [b - a for a, b in zip(vector(start), vector(current))]
    require(max(map(abs, drift)) <= tolerance, 'Start drift exceeds 0.002 rad')
    return drift


def controller_gate(controllers):
    arm = [c for c in controllers if c.name == CONTROLLER]
    require(len(arm) == 1 and arm[0].state == 'active', 'Arm controller is not active')
    require(arm[0].type == 'joint_trajectory_controller/JointTrajectoryController',
            'Unexpected arm controller type')
    require(set(arm[0].claimed_interfaces) == {f'{j}/effort' for j in JOINTS},
            'Arm controller must own exactly seven effort interfaces')
    for c in controllers:
        if c.state != 'active' or c.name == CONTROLLER:
            continue
        claims = list(c.claimed_interfaces) + list(c.required_command_interfaces)
        require(c.name != 'fr3_policy_position_controller' and
                all(x.split('/')[0] in ('fr3_finger_joint1', 'fr3_finger_joint2')
                    for x in claims), f'Competing active controller: {c.name}')


def graph_gate(direct_publishers, nodes, published_topics):
    require(not direct_publishers, 'Direct arm joint_trajectory publisher detected')
    tokens = ('servo', 'spacemouse', 'space_mouse', 'spacenav')
    require(not any(t in n.lower() for n in nodes for t in tokens),
            'Servo/SpaceMouse node detected; stop competing command paths')
    for name, types in published_topics:
        require(not any(t in name.lower() for t in tokens) and
                'control_msgs/msg/JointJog' not in types,
                f'Competing command topic: {name}')


def plain(value):
    """ROS messages and arrays to strict JSON data; reject nonfinite evidence."""
    if hasattr(value, 'get_fields_and_field_types'):
        return {k: plain(getattr(value, k)) for k in value.get_fields_and_field_types()}
    if isinstance(value, Mapping):
        return {str(k): plain(v) for k, v in value.items()}
    if isinstance(value, (str, bool, int)) or value is None:
        return value
    if isinstance(value, float):
        require(math.isfinite(value), 'Nonfinite value in state/evidence')
        return value
    return [plain(v) for v in value]


def health_evidence(msg):
    data = plain(msg)  # Check every numeric field, including nested state.
    return {k: data[k] for k in ('robot_mode', 'current_errors', 'last_motion_errors',
                                'collision_indicators', 'control_command_success_rate')}


def health_gate(msg):
    data = health_evidence(msg)
    require(msg.robot_mode in (msg.ROBOT_MODE_IDLE, msg.ROBOT_MODE_MOVE),
            'Franka robot mode is not commandable (IDLE/MOVE)')
    require(not any(data['current_errors'].values()), 'Franka current_errors present')
    collision = msg.collision_indicators
    collision_values = list(collision.is_joint_collision)
    for field in (
        collision.is_cartesian_linear_collision,
        collision.is_cartesian_angular_collision,
    ):
        collision_values.extend((field.x, field.y, field.z))
    require(
        all(math.isfinite(v) for v in collision_values)
        and not any(collision_values),
        'Franka collision indicator active or invalid',
    )
    require(math.isfinite(msg.control_command_success_rate),
            'Invalid Franka control_command_success_rate')
    joint_state(msg.measured_joint_state)
    return data


def limits_from_parameters(urdf, params):
    """Use URDF position/velocity bounds plus MoveIt acceleration limits.

    This matches the physically commissioned validation path: the FR3 URDF is
    authoritative for joint position/velocity bounds, while the live MoveIt
    planning parameters provide the per-joint acceleration limits.
    """
    root = ET.fromstring(urdf)
    limits = []
    for joint in JOINTS:
        elem = root.find(f"joint[@name='{joint}']")
        require(elem is not None and elem.attrib.get('type') == 'revolute',
                f'Missing bounded revolute joint {joint}')
        limit = elem.find('limit')
        require(limit is not None, f'Missing URDF limits: {joint}')
        lower, upper, velocity = [
            float(limit.attrib[k]) for k in ('lower', 'upper', 'velocity')
        ]
        safety = elem.find('safety_controller')
        if safety is not None:
            lower = max(lower, float(safety.attrib.get('soft_lower_limit', lower)))
            upper = min(upper, float(safety.attrib.get('soft_upper_limit', upper)))

        p = params[joint]
        require(p.get('has_acceleration_limits') is True,
                f'Missing enabled runtime acceleration limit: {joint}')
        acceleration = float(p['max_acceleration'])

        require(
            all(math.isfinite(v) for v in (lower, upper, velocity, acceleration))
            and lower < upper
            and velocity > 0
            and acceleration > 0,
            f'Invalid limits: {joint}',
        )
        limits.append(
            dict(
                lower=lower,
                upper=upper,
                velocity=velocity,
                acceleration=acceleration,
            )
        )
    return limits


def position_limits(q, limits):
    for joint, value, limit in zip(JOINTS, vector(q), limits):
        require(limit['lower'] <= value <= limit['upper'], f'Position limit violation: {joint}')


def validate_plan(response, start, target, limits, velocity_scale, acceleration_scale, tolerance):
    require(response.error_code.val == 1, f'MoveIt planning failed: {response.error_code.val}')
    require(math.isfinite(response.planning_time) and response.planning_time >= 0,
            'Invalid planning time')
    trajectory = response.trajectory
    require(not trajectory.multi_dof_joint_trajectory.joint_names and
            not trajectory.multi_dof_joint_trajectory.points, 'Unexpected multi-DOF trajectory')
    jt = trajectory.joint_trajectory
    require(list(jt.joint_names) == list(JOINTS), 'Trajectory joint ordering mismatch')
    require(len(jt.points) >= 2, 'Trajectory must have at least two points')
    peak_v, peak_a, previous = [0.0] * 7, [0.0] * 7, -1
    for i, point in enumerate(jt.points):
        q = vector(point.positions, 'planned q')
        dq = vector(point.velocities, 'planned dq')
        ddq = vector(point.accelerations, 'planned ddq')
        require(not point.effort, 'Unexpected effort commands in trajectory')
        sec, ns = point.time_from_start.sec, point.time_from_start.nanosec
        require(sec >= 0 and 0 <= ns < 1000000000, 'Invalid trajectory duration')
        stamp = sec * 1000000000 + ns
        require((i == 0 and stamp == 0) or (i > 0 and stamp > previous),
                'Trajectory timestamps must start at zero and strictly increase')
        previous = stamp
        if i == 0:
            require(max(abs(a - b) for a, b in zip(q, start)) <= 1e-6,
                    'Trajectory start-state mismatch')
        position_limits(q, limits)
        for j, limit in enumerate(limits):
            require(abs(dq[j]) <= limit['velocity'] * velocity_scale + 1e-9,
                    f'Velocity limit violation: {JOINTS[j]}')
            require(abs(ddq[j]) <= limit['acceleration'] * acceleration_scale + 1e-9,
                    f'Acceleration limit violation: {JOINTS[j]}')
            peak_v[j], peak_a[j] = max(peak_v[j], abs(dq[j])), max(peak_a[j], abs(ddq[j]))
    endpoint = [a - b for a, b in zip(jt.points[-1].positions, target)]
    require(max(map(abs, endpoint)) <= tolerance, 'Planned endpoint outside goal tolerance')
    return dict(point_count=len(jt.points), duration_s=previous / 1e9,
                peak_abs_dq=peak_v, peak_abs_ddq=peak_a, endpoint_error=endpoint,
                planning_time_s=response.planning_time)


class SingleExecution:
    """Consume the one permitted attempt before invoking the transport."""
    def __init__(self, evidence):
        self.evidence = evidence

    def send(self, client, goal):
        require(self.evidence['execution_attempts'] == 0, 'Execution retry forbidden')
        self.evidence.update(execution_attempted=True, execution_attempts=1)
        return client.send_goal_async(goal)


class RosTransport:
    def __init__(self, args, evidence):
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState
        from franka_msgs.msg import FrankaRobotState
        from controller_manager_msgs.srv import ListControllers
        from moveit_msgs.srv import GetMotionPlan
        from moveit_msgs.action import ExecuteTrajectory
        from rcl_interfaces.srv import GetParameters
        self.rclpy, self.args, self.evidence = rclpy, args, evidence
        self.node = rclpy.create_node('fr3_joint_target')
        self.samples, self.counts = {}, {'joint': 0, 'franka': 0}
        self.monitoring = False
        self.node.create_subscription(JointState, '/joint_states',
                                      lambda m: self.receive('joint', m), qos_profile_sensor_data)
        self.node.create_subscription(FrankaRobotState, '/franka_robot_state_broadcaster/robot_state',
                                      lambda m: self.receive('franka', m), qos_profile_sensor_data)
        self.controllers = self.node.create_client(ListControllers, '/controller_manager/list_controllers')
        self.planner = self.node.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.parameters = self.node.create_client(GetParameters, '/move_group/get_parameters')
        self.action = ActionClient(self.node, ExecuteTrajectory, '/execute_trajectory')

    def receive(self, kind, msg):
        self.counts[kind] += 1
        self.samples[kind] = (msg, time.monotonic())
        if self.monitoring and kind == 'joint':
            try:
                self.check_stamp(msg)
                _, dq = joint_state(msg)
                measured = self.evidence['measured']
                measured['peak_abs_dq'] = [max(a, abs(b)) for a, b in
                                           zip(measured['peak_abs_dq'], dq)]
                measured['sample_count'] += 1
            except ValueError as exc:
                self.evidence['measured']['telemetry_errors'].append(str(exc))

    def check_stamp(self, msg):
        stamp = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        now = self.node.get_clock().now().nanoseconds
        require(stamp > 0, 'Unstamped telemetry')
        age = (now - stamp) / 1e9
        require(age >= 0, f'Future telemetry: age={age:.6f}s')
        require(age <= self.args.freshness,
                f'Stale telemetry: age={age:.6f}s > {self.args.freshness:.6f}s')

    def wait(self, future, timeout=None):
        end = time.monotonic() + (self.args.timeout if timeout is None else timeout)
        while self.rclpy.ok() and not future.done() and time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=min(0.05, max(0, end - time.monotonic())))
        require(future.done(), 'ROS operation timed out; remote outcome may be unknown')
        return future.result()

    def call(self, client, request):
        require(client.wait_for_service(timeout_sec=self.args.timeout),
                f'Service unavailable: {client.srv_name}')
        if client is self.planner:
            # Discovery must not make the supplied planning start state stale.
            for msg, received in self.samples.values():
                self.check_stamp(msg)
                require(time.monotonic() - received <= self.args.freshness,
                        'Planning start telemetry expired during service discovery')
        return self.wait(client.call_async(request))

    def fresh(self):
        counts = self.counts.copy()
        end = time.monotonic() + self.args.timeout
        while self.rclpy.ok() and time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            if all(self.counts[k] > counts[k] for k in counts):
                try:
                    for msg, received in self.samples.values():
                        self.check_stamp(msg)
                        receipt_age = time.monotonic() - received
                        require(
                            0 <= receipt_age <= self.args.freshness,
                            f'Stale telemetry receipt: age={receipt_age:.6f}s '
                            f'> {self.args.freshness:.6f}s',
                        )
                    return self.samples['joint'][0], self.samples['franka'][0]
                except ValueError:
                    pass
        raise ValueError('Timed out waiting for fresh joint and Franka state')

    def ownership(self):
        from controller_manager_msgs.srv import ListControllers
        response = self.call(self.controllers, ListControllers.Request())
        controller_gate(response.controller)
        nodes = [namespace.rstrip('/') + '/' + name for name, namespace in
                 self.node.get_node_names_and_namespaces()]
        published = [(name, types) for name, types in self.node.get_topic_names_and_types()
                     if self.node.count_publishers(name)]
        graph_gate(self.node.get_publishers_info_by_topic('/fr3_arm_controller/joint_trajectory'),
                   nodes, published)

    def preflight(self, label):
        self.ownership()
        js, fs = self.fresh()
        q, dq = joint_state(js)
        self.evidence[f'franka_{label}'] = health_evidence(fs)
        health_gate(fs)
        stationary(dq, self.args.stationary_threshold)
        return q, dq

    def limits(self):
        from rcl_interfaces.srv import GetParameters
        from rclpy.parameter import parameter_value_to_python
        suffixes = ('has_acceleration_limits', 'max_acceleration')
        names = ['robot_description'] + [f'robot_description_planning.joint_limits.{j}.{s}'
                                          for j in JOINTS for s in suffixes]
        response = self.call(self.parameters, GetParameters.Request(names=names))
        require(len(response.values) == len(names), 'Incomplete MoveIt parameters')
        values = dict(zip(names, map(parameter_value_to_python, response.values)))
        require(isinstance(values['robot_description'], str), 'Runtime robot_description unavailable')
        params = {j: {s: values[f'robot_description_planning.joint_limits.{j}.{s}']
                      for s in suffixes} for j in JOINTS}
        return limits_from_parameters(values['robot_description'], params)

    def run(self):
        from moveit_msgs.srv import GetMotionPlan
        from moveit_msgs.msg import Constraints, JointConstraint
        from moveit_msgs.action import ExecuteTrajectory
        args, out = self.args, self.evidence
        limits = self.limits()
        out['limits'] = limits

        require(
            self.planner.wait_for_service(timeout_sec=args.timeout),
            'Service unavailable: /plan_kinematic_path',
        )

        q, _ = self.preflight('before_planning')
        target = vector(args.target if args.target is not None else
                        [a + b for a, b in zip(q, args.delta)])
        out.update(requested_absolute_target=target, planning_start_q=q)
        position_limits(q, limits)
        position_limits(target, limits)
        request = GetMotionPlan.Request()
        req = request.motion_plan_request
        req.group_name = 'fr3_arm'
        req.num_planning_attempts = 1
        req.allowed_planning_time = args.planning_time
        req.max_velocity_scaling_factor = args.velocity_scale
        req.max_acceleration_scaling_factor = args.acceleration_scale
        req.start_state.is_diff = True
        req.start_state.joint_state.name = list(JOINTS)
        req.start_state.joint_state.position = q
        req.start_state.joint_state.velocity = [0.0] * 7
        req.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=j, position=v, tolerance_above=args.goal_tolerance,
                            tolerance_below=args.goal_tolerance, weight=1.0)
            for j, v in zip(JOINTS, target)])]
        response = self.call(self.planner, request).motion_plan_response
        out['planning_error'] = plain(response.error_code)
        out['planned'] = validate_plan(response, q, target, limits, args.velocity_scale,
                                       args.acceleration_scale, args.goal_tolerance)
        if not args.execute:
            out['outcome'] = 'plan_only_validated'
            return
        require(self.action.wait_for_server(timeout_sec=args.timeout), 'ExecuteTrajectory unavailable')
        current, _ = self.preflight('before_execution')
        out['pre_execution_q'] = current
        out['start_drift'] = [b - a for a, b in zip(q, current)]
        drift_check(q, current)
        goal = ExecuteTrajectory.Goal(trajectory=response.trajectory, controller_names=[CONTROLLER])
        out['measured'] = dict(peak_abs_dq=[0.0] * 7, sample_count=0, telemetry_errors=[])
        self.monitoring = True
        handle = None
        try:
            handle = self.wait(SingleExecution(out).send(self.action, goal))
            out['execution_goal_accepted'] = handle.accepted
            require(handle.accepted, 'ExecuteTrajectory goal rejected')
            result = self.wait(handle.get_result_async(), args.execution_timeout)
            out['execution_action_status'] = result.status
            out['execution_error'] = plain(result.result.error_code)
            require(result.status == 4 and result.result.error_code.val == 1,
                    'ExecuteTrajectory failed or cancelled')
        except (Exception, KeyboardInterrupt):
            if handle is not None and handle.accepted and out.get('execution_action_status') is None:
                # Cancellation is not another trajectory goal. Never send a hold or retry.
                out['cancellation_requested'] = True
                try:
                    cancel = self.wait(handle.cancel_goal_async())
                    out['cancellation_return_code'] = cancel.return_code
                except Exception as exc:
                    out['cancellation_error'] = str(exc)
            raise
        finally:
            self.monitoring = False
            try:
                js, fs = self.fresh()
                final_q, final_dq = joint_state(js)
                out['measured'].update(final_q=final_q, final_dq=final_dq,
                                       final_tracking_error=[a - b for a, b in zip(final_q, target)])
                out['franka_after'] = health_evidence(fs)
                health_gate(fs)
            except Exception as exc:
                out['post_execution_error'] = str(exc)
        require('post_execution_error' not in out, 'Post-execution state/health check failed')
        require(out['measured']['sample_count'] > 0 and not out['measured']['telemetry_errors'],
                'Incomplete or invalid measured execution telemetry')
        out['outcome'] = 'execution_succeeded'


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    target = p.add_mutually_exclusive_group(required=True)
    target.add_argument('--target', type=float, nargs=7, metavar='Q')
    target.add_argument('--delta', type=float, nargs=7, metavar='DQ')
    p.add_argument('--execute', action='store_true', help='Send exactly one ExecuteTrajectory goal')
    p.add_argument('--output', type=Path, help='Write strict JSON evidence (including failures)')
    p.add_argument('--velocity-scale', type=float, default=0.1)
    p.add_argument('--acceleration-scale', type=float, default=0.1)
    p.add_argument('--goal-tolerance', type=float, default=1e-4)
    p.add_argument('--planning-time', type=float, default=2.0)
    p.add_argument('--stationary-threshold', type=float, default=0.01)
    p.add_argument('--freshness', type=float, default=0.25, help='Maximum telemetry age in seconds')
    p.add_argument('--timeout', type=float, default=5.0, help='Service/state timeout in seconds')
    p.add_argument('--execution-timeout', type=float, default=5.0)
    return p


def main(argv=None):
    p = parser()
    args = p.parse_args(argv)
    out = dict(schema_version=1, joint_names=list(JOINTS), execution_attempted=False,
               execution_attempts=0, execution_action_status=None, execution_error=None,
               execution_goal_accepted=None, outcome='failed')
    ros = None
    initialized = False
    output = None
    try:
        # Verify evidence destination before any possible command.
        if args.output:
            output = args.output.open('w', encoding='utf-8')
        vector(args.target if args.target is not None else args.delta)
        for key in ('velocity_scale', 'acceleration_scale', 'goal_tolerance', 'planning_time',
                    'stationary_threshold', 'freshness', 'timeout', 'execution_timeout'):
            value = getattr(args, key)
            require(math.isfinite(value) and value > 0, f'{key} must be finite and positive')
        require(args.velocity_scale <= 1 and args.acceleration_scale <= 1, 'Scaling must be <= 1')
        out['settings'] = {k: v for k, v in vars(args).items() if k != 'output'}
        import rclpy
        rclpy.init(args=[])
        initialized = True
        ros = RosTransport(args, out)
        ros.run()
    except (Exception, KeyboardInterrupt) as exc:
        out.update(outcome='failed', error=str(exc) or type(exc).__name__)
    finally:
        if ros is not None:
            ros.node.destroy_node()
        if initialized:
            rclpy.try_shutdown()
    payload = json.dumps(plain(out), indent=2, allow_nan=False)
    print(('PLAN ONLY' if not args.execute else 'SINGLE EXECUTION') + ': ' + out['outcome'])
    if 'planned' in out:
        planned = out['planned']
        print(f"Plan: {planned['point_count']} points, {planned['duration_s']:.6f} s; "
              'planned derivative peaks below are waypoint values.')
    print(f"Execution attempts: {out['execution_attempts']} (maximum 1)")
    if out.get('error'):
        print('Failure: ' + out['error'])
    print(payload)
    if output is not None:
        with output:
            output.write(payload + '\n')
    return 0 if out['outcome'] != 'failed' else 1


if __name__ == '__main__':
    raise SystemExit(main())

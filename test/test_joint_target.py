"""Hardware-independent validation and mocked transport contract tests."""
import copy
import json
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

import fr3_lab_stack_runtime.joint_target as m


def state():
    return NS(name=list(m.JOINTS), position=[0.] * 7, velocity=[0.] * 7)


def controllers():
    return [NS(name=m.CONTROLLER, state='active',
               type='joint_trajectory_controller/JointTrajectoryController',
               claimed_interfaces=[f'{j}/effort' for j in m.JOINTS],
               required_command_interfaces=[])]


def plan():
    points = [NS(positions=[0.] * 7, velocities=[0.] * 7, accelerations=[0.] * 7,
                 effort=[], time_from_start=NS(sec=i, nanosec=0)) for i in range(2)]
    return NS(error_code=NS(val=1), planning_time=0.1,
              trajectory=NS(joint_trajectory=NS(joint_names=list(m.JOINTS), points=points),
                            multi_dof_joint_trajectory=NS(joint_names=[], points=[])))


def limits():
    return [dict(lower=-3., upper=3., velocity=1., acceleration=2.) for _ in m.JOINTS]


def validate(p):
    return m.validate_plan(p, [0.] * 7, [0.] * 7, limits(), .1, .1, 1e-4)


def test_normalization_and_fingers():
    s = state()
    s.name.reverse()
    s.position = list(range(7))
    assert m.joint_state(s)[0] == list(reversed(range(7)))
    s.name.append('fr3_finger_joint1')
    s.position.append(.02)
    s.velocity.append(0.)
    assert len(m.joint_state(s)[0]) == 7


@pytest.mark.parametrize('change', [
    lambda s: s.name.pop(), lambda s: s.name.__setitem__(0, s.name[1]),
    lambda s: s.name.__setitem__(0, 'wrong_joint'),
    lambda s: s.position.__setitem__(0, float('nan')),
    lambda s: s.velocity.__setitem__(0, float('inf')),
    lambda s: s.velocity.clear(),
])
def test_bad_joint_state(change):
    s = state()
    change(s)
    with pytest.raises(ValueError):
        m.joint_state(s)


@pytest.mark.parametrize('q', [[0.] * 6, [0.] * 8, [float('nan')] * 7, [float('inf')] * 7])
def test_bad_target(q):
    with pytest.raises(ValueError):
        m.vector(q)


def test_stationary_and_drift():
    m.stationary([.01] * 7, .01)
    m.drift_check([0.] * 7, [.002] * 7)
    with pytest.raises(ValueError):
        m.stationary([.01001] * 7, .01)
    with pytest.raises(ValueError):
        m.drift_check([0.] * 7, [.00201] * 7)


def test_reference_check():
    assert m.reference_check([0.] * 7, [.002] * 7) == [.002] * 7
    with pytest.raises(ValueError, match='Reference state mismatch'):
        m.reference_check([0.] * 7, [.00201] * 7)
    with pytest.raises(ValueError, match='reference_tolerance'):
        m.reference_check([0.] * 7, [0.] * 7, 0.0)


def test_valid_plan_metrics():
    result = validate(plan())
    assert result['duration_s'] == 1 and result['point_count'] == 2
    assert result['peak_abs_dq'] == [0.] * 7


@pytest.mark.parametrize('change', [
    lambda p: p.trajectory.joint_trajectory.joint_names.reverse(),
    lambda p: p.trajectory.joint_trajectory.points.pop(),
    lambda p: setattr(p.trajectory.joint_trajectory.points[0].time_from_start, 'nanosec', 1),
    lambda p: setattr(p.trajectory.joint_trajectory.points[1].time_from_start, 'sec', 0),
    lambda p: p.trajectory.joint_trajectory.points[0].positions.__setitem__(0, .001),
    lambda p: p.trajectory.joint_trajectory.points[-1].positions.__setitem__(0, .000101),
    lambda p: p.trajectory.joint_trajectory.points[-1].positions.__setitem__(0, 4.),
    lambda p: p.trajectory.joint_trajectory.points[-1].velocities.__setitem__(0, .10001),
    lambda p: p.trajectory.joint_trajectory.points[-1].accelerations.__setitem__(0, .20001),
    lambda p: p.trajectory.joint_trajectory.points[-1].velocities.__setitem__(0, float('nan')),
    lambda p: p.trajectory.joint_trajectory.points[-1].accelerations.clear(),
    lambda p: p.trajectory.multi_dof_joint_trajectory.points.append('unexpected'),
])
def test_plan_rejection(change):
    p = plan()
    change(p)
    with pytest.raises(ValueError):
        validate(p)


def test_controller_gate():
    c = controllers()
    m.controller_gate(c)
    for key, value in [('state', 'inactive'), ('claimed_interfaces', []), ('type', 'other')]:
        bad = copy.deepcopy(c)
        setattr(bad[0], key, value)
        with pytest.raises(ValueError):
            m.controller_gate(bad)
    for name, claims in [('fr3_policy_position_controller', []), ('other', ['fr3_joint1/position'])]:
        with pytest.raises(ValueError):
            m.controller_gate(c + [NS(name=name, state='active', claimed_interfaces=claims,
                                      required_command_interfaces=[])])


def test_graph_gate():
    m.graph_gate([], ['/move_group', '/moveit_simple_controller_manager'],
                 [('/execute_trajectory/_action/status', ['action_msgs/msg/GoalStatusArray'])])
    for args in [([object()], [], []), ([], ['/servo_node'], []),
                 ([], ['/spacenav'], []), ([], [], [('/jog', ['control_msgs/msg/JointJog'])])]:
        with pytest.raises(ValueError):
            m.graph_gate(*args)


def test_no_retry_even_on_send_exception():
    evidence = dict(execution_attempted=False, execution_attempts=0)
    once = m.SingleExecution(evidence)
    client = Mock()
    client.send_goal_async.side_effect = RuntimeError('transport failed')
    with pytest.raises(RuntimeError):
        once.send(client, object())
    with pytest.raises(ValueError):
        once.send(client, object())
    assert client.send_goal_async.call_count == 1
    assert evidence == dict(execution_attempted=True, execution_attempts=1)


def test_json_safe():
    assert json.loads(json.dumps(m.plain({'q': (1., 2.), 'missing': None}), allow_nan=False)) == {
        'q': [1., 2.], 'missing': None}
    with pytest.raises(ValueError):
        m.plain({'bad': float('nan')})


def test_runtime_limits():
    urdf = '<robot>' + ''.join(
        f'<joint name="{j}" type="revolute">'
        '<limit lower="-2" upper="2" velocity="2"/>'
        '<safety_controller soft_lower_limit="-1" soft_upper_limit="1"/>'
        '</joint>'
        for j in m.JOINTS
    ) + '</robot>'

    params = {
        j: dict(has_acceleration_limits=True, max_acceleration=3.)
        for j in m.JOINTS
    }

    result = m.limits_from_parameters(urdf, params)
    assert result[0] == dict(
        lower=-1.,
        upper=1.,
        velocity=2.,
        acceleration=3.,
    )

    params[m.JOINTS[0]]['has_acceleration_limits'] = None
    with pytest.raises(ValueError, match='acceleration'):
        m.limits_from_parameters(urdf, params)


def test_ros_health_constants_and_finite_fields():
    from franka_msgs.msg import FrankaRobotState

    fs = FrankaRobotState()
    fs.robot_mode = fs.ROBOT_MODE_IDLE
    fs.control_command_success_rate = 1.
    fs.measured_joint_state.name = list(m.JOINTS)
    fs.measured_joint_state.position = [0.] * 7
    fs.measured_joint_state.velocity = [0.] * 7

    m.health_gate(fs)
    fs.robot_mode = fs.ROBOT_MODE_MOVE
    m.health_gate(fs)

    # Current robot errors, active collision indicators, invalid modes, and
    # nonfinite state remain fail-closed.
    rejecting_mutations = [
        lambda f: setattr(f, 'robot_mode', f.ROBOT_MODE_GUIDING),
        lambda f: setattr(f.current_errors, 'joint_position_limits_violation', True),
        lambda f: setattr(f.collision_indicators.is_cartesian_linear_collision, 'x', 1.),
        lambda f: setattr(f.o_t_ee.pose.position, 'x', float('nan')),
        lambda f: setattr(f, 'control_command_success_rate', float('nan')),
    ]
    for mutate in rejecting_mutations:
        bad = copy.deepcopy(fs)
        mutate(bad)
        with pytest.raises(ValueError):
            m.health_gate(bad)

    # Contact and historical errors are retained in evidence but do not, by
    # themselves, reject an otherwise currently healthy robot.
    contact = copy.deepcopy(fs)
    contact.collision_indicators.is_cartesian_linear_contact.x = 1.
    contact.last_motion_errors.cartesian_reflex = True
    contact.control_command_success_rate = .98
    evidence = m.health_gate(contact)

    assert evidence['collision_indicators']['is_cartesian_linear_contact']['x'] == 1.
    assert evidence['last_motion_errors']['cartesian_reflex'] is True
    assert evidence['control_command_success_rate'] == pytest.approx(.98)


def test_stamp_gate():
    transport = object.__new__(m.RosTransport)
    transport.args = NS(freshness=.25)
    transport.node = Mock()
    transport.node.get_clock.return_value.now.return_value.nanoseconds = 2000000000
    for sec, ns, valid in [(2, 0, True), (1, 900000000, True), (1, 0, False),
                           (0, 0, False), (2, 1, False)]:
        msg = NS(header=NS(stamp=NS(sec=sec, nanosec=ns)))
        if valid:
            transport.check_stamp(msg)
        else:
            with pytest.raises(ValueError):
                transport.check_stamp(msg)


@pytest.mark.parametrize('execute,status,code,accepted', [
    (False, 4, 1, True), (True, 4, 1, True), (True, 6, -4, True),
    (True, 5, -7, True), (True, 4, 1, False),
    (True, 'result_timeout', 1, True), (True, 'goal_timeout', 1, True),
    (True, 'drift', 1, True), (True, 'health', 1, True),
])
def test_mocked_transport_contract(execute, status, code, accepted):
    from moveit_msgs.msg import MotionPlanResponse
    from trajectory_msgs.msg import JointTrajectoryPoint
    from franka_msgs.msg import FrankaRobotState
    r = object.__new__(m.RosTransport)
    r.args = m.parser().parse_args(['--target'] + ['0'] * 7 + (['--execute'] if execute else []))
    r.evidence = dict(execution_attempted=False, execution_attempts=0)
    r.limits = lambda: limits()
    r.preflight = Mock(return_value=([0.] * 7, [0.] * 7))
    if status == 'drift':
        r.preflight.side_effect = [([0.] * 7, [0.] * 7), ([.003] * 7, [0.] * 7)]
    if status == 'health':
        r.preflight.side_effect = [([0.] * 7, [0.] * 7), ValueError('unhealthy')]
    response = MotionPlanResponse()
    response.error_code.val = 1
    response.trajectory.joint_trajectory.joint_names = list(m.JOINTS)
    for i in range(2):
        point = JointTrajectoryPoint(positions=[0.] * 7, velocities=[0.] * 7, accelerations=[0.] * 7)
        point.time_from_start.sec = i
        response.trajectory.joint_trajectory.points.append(point)
    r.planner = Mock()
    r.planner.wait_for_service.return_value = True
    r.call = Mock(return_value=NS(motion_plan_response=response))
    r.action = Mock()
    handle = Mock(accepted=accepted)
    result = NS(status=status, result=NS(error_code=response.error_code))
    result.result.error_code.val = code
    # Planning and execution errors must be separate objects.
    response.error_code.val = 1
    from moveit_msgs.msg import MoveItErrorCodes
    result.result.error_code = MoveItErrorCodes(val=code)
    def wait(future, timeout=None):
        r.evidence['measured']['sample_count'] += 1
        if future is handle.cancel_goal_async.return_value:
            return NS(return_code=0)
        if status == 'goal_timeout' or (status == 'result_timeout' and
                                        future is handle.get_result_async.return_value):
            raise ValueError('ROS operation timed out')
        return handle if future is r.action.send_goal_async.return_value else result
    r.wait = wait
    fs = FrankaRobotState(robot_mode=FrankaRobotState.ROBOT_MODE_IDLE,
                          control_command_success_rate=1.)
    fs.measured_joint_state.name = list(m.JOINTS)
    fs.measured_joint_state.position = [0.] * 7
    fs.measured_joint_state.velocity = [0.] * 7
    r.fresh = lambda: (state(), fs)
    if execute and (status != 4 or code != 1 or not accepted):
        with pytest.raises(ValueError):
            r.run()
    else:
        r.run()
    attempted = execute and status not in ('drift', 'health')
    assert r.action.send_goal_async.call_count == int(attempted)
    assert r.evidence['execution_attempts'] == int(attempted)
    req = r.call.call_args.args[1].motion_plan_request
    assert req.group_name == 'fr3_arm' and req.num_planning_attempts == 1
    assert list(req.start_state.joint_state.velocity) == [0.] * 7
    assert len(req.goal_constraints[0].joint_constraints) == 7
    if attempted:
        goal = r.action.send_goal_async.call_args.args[0]
        assert goal.controller_names == [m.CONTROLLER]
        assert goal.trajectory == response.trajectory
        assert 'final_q' in r.evidence['measured']
    if status == 'result_timeout':
        handle.cancel_goal_async.assert_called_once()
        assert r.evidence['cancellation_return_code'] == 0
    if not execute:
        r.preflight.assert_called_once_with('before_planning')



def test_validate_args_with_reference():
    args = m.parser().parse_args(['--target'] + ['0'] * 7)
    args.reference_q = [0.] * 7
    args.reference_tolerance = .002
    m.validate_args(args)
    args.reference_tolerance = 0.
    with pytest.raises(ValueError, match='reference_tolerance'):
        m.validate_args(args)


def test_invalid_cli_json(tmp_path):
    output = tmp_path / 'result.json'
    assert m.main(['--target'] + ['nan'] * 7 + ['--output', str(output)]) == 1
    assert json.loads(output.read_text())['execution_attempts'] == 0
    # Invalid arguments fail before ROS initialization or any execution attempt.

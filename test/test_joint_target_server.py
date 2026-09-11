"""Unit tests for the persistent joint-target server contract."""

import pytest

from fr3_lab_stack_interfaces.action import ExecuteJointTarget
import fr3_lab_stack_runtime.joint_target_server as s


def request():
    req = ExecuteJointTarget.Goal()
    req.request_id = 'req-1'
    req.reference_q = [0.0] * 7
    req.target_q = [0.01] + [0.0] * 6
    req.reference_stamp.sec = 10
    req.reference_stamp.nanosec = 0
    return req


def settings(execute=False):
    return dict(
        execute=execute,
        velocity_scale=.1,
        acceleration_scale=.1,
        goal_tolerance=1e-4,
        planning_time=2.0,
        stationary_threshold=.01,
        freshness=.25,
        timeout=5.0,
        execution_timeout=5.0,
        reference_tolerance=.002,
    )


def test_reference_stamp_age():
    req = request()
    assert s.stamp_age_seconds(req.reference_stamp, 10_100_000_000) == pytest.approx(.1)

    req.reference_stamp.sec = 0
    with pytest.raises(ValueError, match='nonzero'):
        s.stamp_age_seconds(req.reference_stamp, 10_100_000_000)

    req.reference_stamp.sec = 11
    with pytest.raises(ValueError, match='Future'):
        s.stamp_age_seconds(req.reference_stamp, 10_100_000_000)


def test_goal_validation():
    req = request()
    result = s.validate_goal_request(req, 10_100_000_000, .25)
    assert result['reference_q'] == [0.] * 7
    assert result['target_q'][0] == .01
    assert result['reference_age_s'] == pytest.approx(.1)

    req.request_id = ''
    with pytest.raises(ValueError, match='request_id'):
        s.validate_goal_request(req, 10_100_000_000, .25)


def test_goal_rejects_stale_and_nonfinite():
    req = request()
    with pytest.raises(ValueError, match='Stale'):
        s.validate_goal_request(req, 10_300_000_000, .25)

    req = request()
    req.target_q[0] = float('nan')
    with pytest.raises(ValueError, match='nonfinite'):
        s.validate_goal_request(req, 10_100_000_000, .25)


def test_busy_gate_rejects_queueing():
    gate = s.BusyGate()
    assert gate.reserve() is True
    assert gate.busy is True
    assert gate.reserve() is False
    gate.release()
    assert gate.busy is False
    assert gate.reserve() is True


def test_build_runtime_args_preserves_absolute_target_and_reference():
    req = request()
    args = s.build_runtime_args(req, settings(execute=False))
    assert args.target == list(req.target_q)
    assert args.reference_q == list(req.reference_q)
    assert args.delta is None
    assert args.execute is False
    assert args.reference_tolerance == pytest.approx(.002)
    assert args.node_name == 'fr3_joint_target_worker'


def test_build_runtime_args_rejects_bad_scale():
    req = request()
    bad = settings()
    bad['velocity_scale'] = 1.01
    with pytest.raises(ValueError, match='Scaling'):
        s.build_runtime_args(req, bad)


@pytest.mark.parametrize(
    'evidence,planning,attempted,status,execution',
    [
        (
            dict(outcome='plan_only_validated',
                 planning_error={'val': 1},
                 execution_attempted=False,
                 execution_action_status=None,
                 execution_error=None),
            1, False, 0, 0,
        ),
        (
            dict(outcome='execution_succeeded',
                 planning_error={'val': 1},
                 execution_attempted=True,
                 execution_action_status=4,
                 execution_error={'val': 1}),
            1, True, 4, 1,
        ),
    ],
)
def test_fill_result(evidence, planning, attempted, status, execution):
    result = ExecuteJointTarget.Result()
    s.fill_result(result, evidence, [0.] * 7, [0.] * 7)
    assert result.success is True
    assert result.planning_error_code == planning
    assert result.execution_attempted is attempted
    assert result.execution_action_status == status
    assert result.execution_error_code == execution
    assert '"outcome"' in result.evidence_json


def test_fill_result_failure():
    result = ExecuteJointTarget.Result()
    s.fill_result(
        result,
        dict(outcome='failed', error='bad request'),
        [0.] * 7,
        [0.] * 7,
    )
    assert result.success is False
    assert result.outcome == 'failed'

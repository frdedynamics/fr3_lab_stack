"""C1-A1 validation, fail-closed polling, and immediate publication contract."""
import json
from collections import Counter
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from sensor_msgs.msg import JointState

from fr3_lab_stack_runtime import streaming_joint_target_forwarder as s


def target():
    msg = JointState()
    msg.name = s.JOINTS
    msg.position = [0.1 * i for i in range(7)]
    msg.header.stamp.sec = 10
    return msg


@pytest.mark.parametrize('age,reason', [
    (250_000_000, None), (250_000_001, 'stale_timestamp'),
    (-50_000_000, None), (-50_000_001, 'future_timestamp')])
def test_timestamp_boundaries(age, reason):
    assert s.rejection_reason(target(), 10_000_000_000 + age, s.DEFAULTS) == reason


@pytest.mark.parametrize('change,reason', [
    (lambda m: setattr(m, 'name', list(reversed(s.JOINTS))), 'joint_names'),
    (lambda m: setattr(m, 'name', s.JOINTS[:-1] + ['fr3_joint6']), 'joint_names'),
    (lambda m: setattr(m, 'position', [0.] * 6), 'position_count'),
    (lambda m: setattr(m, 'position', [0.] * 8), 'position_count'),
    (lambda m: setattr(m, 'position', [float('nan')] * 7), 'nonfinite_position'),
    (lambda m: setattr(m, 'position', [float('inf')] * 7), 'nonfinite_position'),
    (lambda m: setattr(m.header.stamp, 'sec', 0), 'invalid_timestamp'),
    (lambda m: setattr(m.header.stamp, 'sec', -1), 'invalid_timestamp'),
    (lambda m: setattr(m.header.stamp, 'nanosec', 1_000_000_000), 'invalid_timestamp'),
])
def test_invalid_targets(change, reason):
    msg = target()
    change(msg)
    assert s.rejection_reason(msg, 10_000_000_000, s.DEFAULTS) == reason


@pytest.fixture
def node(monkeypatch):
    # Exercise the actual callbacks without a ROS graph or robot.
    n = SimpleNamespace(settings=dict(s.DEFAULTS),
                        counts=dict(received=0, accepted=0, rejected=0, forwarded=0,
                                    publication_errors=0),
                        rejections=Counter(), evidence_ns=900_000_000,
                        pending=None, controller_status='active',
                        publisher=Mock(), telemetry=Mock(), client=Mock())
    n.get_logger = Mock(return_value=Mock())
    n.get_clock = Mock(return_value=SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=10_000_000_000)))
    n.on_controllers = lambda f, sent: s.StreamingJointTargetForwarder.on_controllers(n, f, sent)
    monkeypatch.setattr(s.time, 'monotonic_ns', lambda: 1_000_000_000)
    return n


def record(node):
    return json.loads(node.telemetry.publish.call_args.args[0].data)


def test_immediate_unmodified_forwarding_and_timing(node, monkeypatch):
    times = iter([1_000_000_000, 1_000_000_010, 1_000_000_030, 1_000_000_040])
    monkeypatch.setattr(s.time, 'monotonic_ns', lambda: next(times))
    msg = target()
    s.StreamingJointTargetForwarder.on_target(node, msg)
    assert node.publisher.publish.call_args.args[0] is msg
    r = record(node)
    assert r['counts'] == dict(received=1, accepted=1, rejected=0, forwarded=1,
                               publication_errors=0)
    assert r['source_stamp_sec'] == 10
    assert r['server_receipt_ros_ns'] == 10_000_000_000
    assert r['publication_call_duration_ns'] == 20
    assert r['server_processing_duration_ns'] == 40


@pytest.mark.parametrize('evidence,reason', [
    (None, 'controller_active_unconfirmed'), (699_999_999, 'controller_evidence_expired')])
def test_controller_gate(node, evidence, reason):
    node.evidence_ns = evidence
    s.StreamingJointTargetForwarder.on_target(node, target())
    node.publisher.publish.assert_not_called()
    assert record(node)['rejection_counts'] == {reason: 1}
    assert record(node)['counts']['rejected'] == 1
    assert record(node)['publication_call_duration_ns'] is None


def test_publish_failure_is_accepted_but_not_forwarded(node):
    node.publisher.publish.side_effect = RuntimeError('publish failed')
    s.StreamingJointTargetForwarder.on_target(node, target())
    assert record(node)['counts'] == dict(received=1, accepted=1, rejected=0,
                                          forwarded=0, publication_errors=1)
    assert record(node)['outcome'] == 'publication_error'
    node.publisher.publish.assert_called_once()


def test_one_outstanding_request_and_delayed_reply(node):
    s.StreamingJointTargetForwarder.poll(node)
    future = node.pending
    for _ in range(10):
        s.StreamingJointTargetForwarder.poll(node)
    node.client.call_async.assert_called_once()
    future.result.return_value = SimpleNamespace(controller=[
        SimpleNamespace(name=s.CONTROLLER, state='active')])
    node.on_controllers(future, 600_000_000)
    assert node.pending is None
    s.StreamingJointTargetForwarder.on_target(node, target())
    assert record(node)['rejection_reason'] == 'controller_evidence_expired'


@pytest.mark.parametrize('state', ['inactive', 'missing', 'error', 'active'])
def test_controller_response(node, state):
    future = Mock()
    future.result.return_value = SimpleNamespace(controller=[] if state == 'missing' else [
        SimpleNamespace(name=s.CONTROLLER, state=state)])
    if state == 'error':
        future.result.side_effect = RuntimeError('service failure')
    node.on_controllers(future, 900_000_000)
    assert node.evidence_ns == (900_000_000 if state == 'active' else None)


def test_unavailable_service_revokes_evidence(node):
    node.client.service_is_ready.return_value = False
    s.StreamingJointTargetForwarder.poll(node)
    assert node.evidence_ns is None
    node.client.call_async.assert_not_called()


def test_endpoint_qos():
    qos = s.endpoint_qos()
    assert qos.history == HistoryPolicy.KEEP_LAST
    assert qos.depth == 1
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.VOLATILE


def test_request_error_revokes_evidence(node):
    node.client.call_async.side_effect = RuntimeError('request failed')
    s.StreamingJointTargetForwarder.poll(node)
    assert node.pending is None
    assert node.evidence_ns is None
    assert node.controller_status == 'request_error'


def test_rejection_counts_accumulate_without_publishing(node):
    msg = target()
    msg.name = []
    for _ in range(2):
        s.StreamingJointTargetForwarder.on_target(node, msg)
    node.publisher.publish.assert_not_called()
    assert record(node)['counts']['received'] == 2
    assert record(node)['rejection_counts'] == {'joint_names': 2}


def test_ros_graph_forwarding(monkeypatch):
    """Real local ROS endpoints and mock controller manager; no hardware."""
    import time
    import rclpy
    from controller_manager_msgs.msg import ControllerState
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node

    monkeypatch.setenv('ROS_DOMAIN_ID', '173')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init()
    forwarder = s.StreamingJointTargetForwarder()
    peer = Node('forwarder_test_peer')
    executor = SingleThreadedExecutor()
    executor.add_node(forwarder)
    executor.add_node(peer)
    state = ['active']
    received = []

    def controllers(request, response):
        response.controller = [ControllerState(name=s.CONTROLLER, state=state[0])]
        return response

    peer.create_service(s.ListControllers, '/controller_manager/list_controllers', controllers)
    pub = peer.create_publisher(JointState, '/fr3_streaming_joint_target', s.endpoint_qos())
    peer.create_subscription(JointState, f'/{s.CONTROLLER}/target_joint',
                             received.append, s.endpoint_qos())

    def until(predicate):
        deadline = time.monotonic() + 5.0
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert predicate()

    try:
        until(lambda: forwarder.evidence_ns is not None and
              pub.get_subscription_count() == 1 and
              forwarder.publisher.get_subscription_count() == 1)
        for endpoint in (forwarder.publisher, forwarder.subscription):
            assert endpoint.qos_profile == s.endpoint_qos()
        msg = target()
        msg.header.stamp = peer.get_clock().now().to_msg()
        msg.header.frame_id = 'preserve_me'
        msg.velocity = [0.2] * 7
        pub.publish(msg)
        until(lambda: len(received) == 1)
        assert received[0] == msg
        assert forwarder.counts['forwarded'] == 1
        state[0] = 'inactive'
        until(lambda: forwarder.controller_status == 'inactive_or_missing')
        msg.header.stamp = peer.get_clock().now().to_msg()
        pub.publish(msg)
        until(lambda: forwarder.counts['rejected'] == 1)
        assert forwarder.counts['forwarded'] == 1
        assert len(received) == 1
    finally:
        executor.shutdown()
        peer.destroy_node()
        forwarder.destroy_node()
        rclpy.shutdown()

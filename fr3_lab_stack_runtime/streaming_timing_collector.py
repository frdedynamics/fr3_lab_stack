"""Record a short measured-q-only run; analyze raw JSONL offline as well."""
import argparse
import json
import math
import time
import uuid
from pathlib import Path

from .timing_evidence import analyze, clock_provenance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='timing-run.jsonl')
    parser.add_argument('--analyze', metavar='JSONL')
    parser.add_argument('--count', type=int, default=20)
    parser.add_argument('--interval', type=float, default=0.1)
    args = parser.parse_args()
    if args.analyze:
        records = [json.loads(line) for line in Path(args.analyze).read_text().splitlines()]
        print(json.dumps(analyze(records), indent=2))
        return
    if args.count <= 0 or not math.isfinite(args.interval) or args.interval < .02:
        parser.error('count must be positive; interval must be finite and >= 0.02 seconds')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray, String
    from .streaming_joint_target_forwarder import CONTROLLER, JOINTS, endpoint_qos

    provenance = clock_provenance()
    run = str(uuid.uuid4())
    records = []
    rclpy.init()
    node = rclpy.create_node('fr3_streaming_timing_collector')
    # Exclusive creation prevents accidentally overwriting prior measured evidence.
    output = open(args.output, 'x', encoding='utf-8')

    def save(kind, **fields):
        record = dict(kind=kind, run=run, **fields)
        output.write(json.dumps(record, allow_nan=False) + '\n')
        output.flush()
        records.append(record)

    state = [None, 0]
    def on_state(msg):
        if len(msg.data) == 39 and msg.data[38] == 1 and all(math.isfinite(q) for q in msg.data[:7]):
            state[:] = [list(msg.data[:7]), time.monotonic_ns()]

    node.create_subscription(Float64MultiArray, f'/{CONTROLLER}/state', on_state, qos_profile_sensor_data)
    node.create_subscription(String, '/fr3_streaming_joint_target_forwarder/telemetry',
                             lambda m: save('forwarder', data=json.loads(m.data)), 100)
    node.create_subscription(String, f'/{CONTROLLER}/timing',
                             lambda m: save('controller', data=json.loads(m.data)), 100)
    publisher = node.create_publisher(JointState, '/fr3_streaming_joint_target', endpoint_qos())
    stamps = set()
    def spin_for(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=min(.02, max(0., end - time.monotonic())))

    try:
        save('metadata', **provenance, count=args.count, interval_s=args.interval,
             target_mode='fresh measured q from 39-field state', period_nominal_ns=1_000_000,
             long_period_thresholds_ns=[1_500_000, 2_000_000])
        spin_for(2.)
        if publisher.get_subscription_count() != 1:
            raise RuntimeError('Expected exactly one forwarder subscriber')
        for _ in range(args.count):
            if state[0] is None or time.monotonic_ns() - state[1] > 100_000_000:
                raise RuntimeError('No fresh active-controller measured q within 100 ms')
            msg = JointState()
            msg.name = JOINTS
            msg.position = state[0]
            stamp = node.get_clock().now().nanoseconds
            if stamp <= 0 or stamp in stamps:
                raise RuntimeError('Source ROS stamp is zero or repeats within this run')
            stamps.add(stamp)
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
            msg.header.frame_id = run
            outcome = 'returned'
            t0 = time.monotonic_ns()
            try:
                publisher.publish(msg)
            except Exception:
                outcome = 'publication_error'
                raise
            finally:
                end = time.monotonic_ns()
                save('sent', **provenance, source_stamp_ns=stamp, q=list(msg.position),
                     t0_ns=t0, publication_call_end_ns=end, publication_call_outcome=outcome)
            spin_for(args.interval)
        spin_for(2.)
    finally:
        save('summary', data=analyze(records))
        output.close()
        node.destroy_node()
        rclpy.shutdown()

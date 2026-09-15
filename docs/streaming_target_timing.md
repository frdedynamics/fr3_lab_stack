# C1-A2: target timing and controller-period evidence

**C1-A2 passed: measured-q timing characterization.**

The operator's 2026-09-15 nominal 10 Hz run applied 20/20 targets with exact target
matching and no observed evidence loss. It recorded 6000 consecutive controller
periods, none above 1.5 ms. The [dated validation report](validation/2026-09-15_c1-a2_measured-q_143031/README.md)
contains the verified timing table, original evidence, configuration, provenance,
and exact commands. **C1-B is next; moving 15 Hz execution and policy inference
remain unvalidated.** Finalization did not execute physical C1-B actions.

Based on clean commit `67a99139de545f95038625d7dbe2be68d2630bec`.
This instrumentation leaves joint/hybrid impedance mathematics, latest-target
replacement, activation measured-pose hold, command validation, and the 39-field
`/fr3_streaming_joint_impedance_controller/state` message unchanged. Its configured
rate remains 100 Hz. Timing evidence is a separate stream.

## Capture contract and clocks

| Event | Capture | JSON field |
| --- | --- | --- |
| T0 | Client immediately before target `publish()` | sent `t0_ns` |
| T1 | First statement of forwarder target callback | `server_receipt_monotonic_ns` |
| T2 | Immediately before forwarder target `publish()` | `publication_call_start_monotonic_ns` |
| T3 | First statement of controller target callback | `t3_ns` |
| T4 | Immediately after the first successful `core.set_target()` for this sequence | application `t4_ns` |

T4 is **software equilibrium installation**, not actuator response, torque delivery,
or physical convergence. It remains an application event even if subsequent work
in that update returns an error. Holding a target produces no repeated event.
An intermediate callback can be accepted and overwritten in the latest-value
buffer before any update installs it.

T2 uses publish-call **start**. The controller can receive the target before the
forwarder publish call returns. Forwarder call end/duration remain separate;
the client separately records `publication_call_end_ns`. No interval uses a
publication-call end in place of T2.

C++ explicitly calls `clock_gettime(CLOCK_MONOTONIC)`. Python uses
`time.monotonic_ns()` and verifies at startup that its value lies between calls to
`time.clock_gettime_ns(time.CLOCK_MONOTONIC)`. Software tests verify both bindings;
Python also records `get_clock_info('monotonic').implementation`. Every process
records clock name, hostname, Linux boot ID, and `/proc/self/ns/time` identity.
The analyzer computes intervals only when all four provenance fields are nonempty
and equal. Thus cross-host, cross-boot, or different time-namespace monotonic values
are never subtracted. This assumes the Linux host/namespace provenance accurately
identifies the processes; the collector should run on the controller host.
ROS/source stamps remain available for freshness and correlation, and are never
subtracted from monotonic timestamps. Source and node ROS clocks must share the
existing freshness clock domain.

## Schemas (version 1)

All timestamp, duration, sequence, identity counter, cycle, and overflow values
are JSON **integers**, with times/durations in nanoseconds. Parse them as 64-bit
integers (or Python integers), not IEEE-754 doubles. Positions are seven doubles
in ordered `fr3_joint1` … `fr3_joint7` radians. Statistical means/medians may be
fractional nanoseconds.

### Controller: `~/timing`, `std_msgs/msg/String`

Reliable, volatile, KEEP_LAST depth 100. A 20 ms non-RT wall timer drains evidence
and publishes a batch, including empty heartbeat batches. Callback records publish
from the non-RT subscription callback. Both use the node's mutually exclusive
default callback group. The RT producer is the controller's single update thread.

Common fields:

- `schema: 1`, `clock`, `hostname`, `boot_id`, `time_namespace`.
- `instance`: process ID plus configure-time monotonic timestamp. New configuration
  gets a new identity; `evidence_id` increases for each publication attempt.
- `publication_errors`: cumulative evidence publisher exceptions. Failed attempts
  consume an ID and are not retried; the next successful record exposes the count.
- `event`: `callback` or `samples`.
- `activation`: activation-time monotonic timestamp, unchanged until next activation.

`callback` fields:

- `t3_ns`, `receipt_ros_ns`, `source_stamp_ns`, `source_frame_id`.
- `sequence`: accepted controller sequence; zero for rejected callbacks.
- `outcome`: `accepted`, `inactive`, `joint_shape`, `nonfinite_position`,
  `invalid_stamp`, or `stamp_age`. Existing validation order/semantics are retained.

`samples` fields:

- `samples`: array of `{cycle, activation, period_ns, application?}`.
- `cycle`: lifetime cumulative update index, including updates returning errors.
- `period_ns`: **the supplied `update(..., period)` value**, every cycle, without
  reconstructing it from telemetry or assuming it is 1 ms.
- Optional `application`: `{activation, callback_activation, sequence, source_stamp_ns, t3_ns, t4_ns,
  q_desired}`. Emitted only when that update first installs a new sequence.
  `activation` identifies the installing update; `callback_activation` preserves
  callback identity. A mismatch cannot join to a target in the collector.
- `dropped_period_samples`, `dropped_application_records`: cumulative RT overflow
  counts for the controller object lifetime. One dropped sample can increment both.

The batch's top-level activation is its publication-time activation. Each sample
and application retains its own activation; old queued samples must not be relabeled
when activation changes. The ring and cycle/overflow counters are never reset while
producer or consumer can run. Activation resets the existing command sequence only.

### Forwarder: existing `~/telemetry`

Existing fields, counters, logging, and depth-1 telemetry QoS remain. Added:
`schema`, `forwarder_instance` (process UUID), `source_frame_id`, `clock`, `hostname`,
`boot_id`, `time_namespace`, `python_clock_implementation`, `clock_verified`.
Existing source sec/nanosec, ROS receipt, T1, T2, publish-call end and duration,
acceptance/rejection/publication-error outcomes remain intact.

### Collector JSONL

Every row has `kind` and `run` (UUID). Kinds:

- `metadata`: clock provenance, requested count/interval, measured-q target mode,
  nominal period and long-period thresholds.
- `sent`: clock provenance, `source_stamp_ns`, seven-position `q`, `t0_ns`,
  `publication_call_end_ns`, `publication_call_outcome` (`returned` or `publication_error`).
- `forwarder` / `controller`: original decoded evidence under `data`.
- `summary`: offline analysis under `data`.

The client places the run UUID in `JointState.header.frame_id`, which the forwarder
preserves. It requires positive, unique source stamps in a run and aborts on a
repeat (including paused ROS time). The analyzer rejects mixed runs and duplicate
sent stamps. Joins first require run UUID and source stamp, then controller
`(instance, activation, sequence)`; application T3 must agree with callback T3.
An application missing callback/run evidence remains an unmatched application.

## Storage, statistics, and loss interpretation

`timing_evidence.hpp` provides an 8192-entry preallocated single-producer,
single-consumer ring. Release/acquire atomics transfer fixed-size copies. Full
storage drops the newest evidence and increments lock-free counters. It never
blocks, retries, or postpones command installation, and is never read to choose
commands. Added RT instrumentation performs no allocation, logging, ROS publication,
file I/O, or locking. JSON serialization, publication, collection, sorting, and
statistics happen outside RT. Instrumentation cost still requires hardware validation.

The collector reports period statistics per controller instance and activation over
**received samples during the capture**, including discovery/drain windows. It retains
raw integer samples for reanalysis. It reports count, mean, median, p95, p99, maximum,
mean signed deviation from nominal 1,000,000 ns, and maximum absolute deviation.
Median averages the middle pair for even N. Percentiles use **nearest rank**:
sort N samples ascending, select one-based index `ceil(p*N)`. Empty statistics are
null. Threshold counts use strict `>` with explicitly recorded thresholds
1,500,000 ns and 2,000,000 ns. These are **long periods**, not proven missed cycles.
No missed-cycle estimate is produced.

Per-target status is `applied`, `superseded`, `rejected`, `publication_error`,
`unmatched`, `ambiguous`, or `accepted_application_unresolved`. Supersession requires
a later applied sequence in the same instance/activation, earlier capture coverage,
and no observed controller evidence gap or application overflow. Otherwise absence
of an application is unresolved; an observed loss gives `unresolved_reason:
lost_evidence`. Raw rejection reasons remain in the evidence.

Summary explicitly reports unmatched forwarder/callback/application counts,
interior evidence-ID gaps per process, controller evidence publication errors, and
cumulative RT overflow counts. IDs and
forwarder received counters detect middle-of-capture transport losses. Loss before
the first or after the last received ID is **unknown**. Cumulative overflow may
predate the run and conservatively prevents a superseded conclusion. Statistics
with evidence gaps/overflow describe the received subset; they do not certify the
whole controller run. Reliable DDS is not a guarantee of complete recording.

## Measured-q collection procedure

Use the already commissioned hybrid-controller bringup and activate
`fr3_streaming_joint_impedance_controller` with the existing configuration. Run all
three processes on the controller host in the same ROS domain, boot, and time
namespace. The commands below assume the controller is already active; they do not
switch controllers or start policy playback.

In each terminal:

```bash
cd /home/hvl-robotics2404/franka_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Confirm the active controller:

```bash
ros2 control list_controllers
```

Start the forwarder in terminal 1 (use the existing process if already running):

```bash
ros2 run fr3_lab_stack fr3_streaming_joint_target_forwarder --ros-args \
  --params-file /home/hvl-robotics2404/franka_ros2_ws/src/fr3_lab_stack/config/streaming_joint_target_forwarder.yaml
```

In terminal 2, record 20 fresh measured-q targets, 100 ms apart:

```bash
ros2 run fr3_lab_stack fr3_streaming_timing_collector \
  --count 20 --interval 0.1 --output /tmp/c1-a2-measured-q.jsonl
ros2 run fr3_lab_stack fr3_streaming_timing_collector \
  --analyze /tmp/c1-a2-measured-q.jsonl > /tmp/c1-a2-measured-q-summary.json
```

The client exclusively creates the output file, waits two seconds for discovery,
requires exactly one forwarder subscriber, and uses fresh active-controller state
(no older than 100 ms) for every target. Each target is the just-observed measured q;
there are no prerecorded actions or offsets. It drains two seconds after the last
send and exits without changing controller activation. Inspect every target status,
all loss counters, provenance, applied versus sent q, T0–T4 intervals, and period
statistics before drawing hardware timing conclusions. Timestamp precision alone
does not establish measurement accuracy.

## Software validation and added files

- C++ helper: `include/fr3_lab_stack/timing_evidence.hpp`.
- Python clock/statistics/analysis helper: `fr3_lab_stack_runtime/timing_evidence.py`.
- Client/collector: `fr3_lab_stack_runtime/streaming_timing_collector.py` and
  `scripts/fr3_streaming_timing_collector.py` (installed as `fr3_streaming_timing_collector`).
- Tests: `test/test_timing_evidence.cpp`, `test/test_timing_evidence.py`.

The tests exercise the same installation helper used by `update()`, a real
RealtimeBuffer, the impedance core, holding/supersession, activation reset,
overflow without changed desired q, ring wraparound, both clock bindings, synthetic
statistics, start-versus-end timing, cross-host refusal, run identity, missing
callbacks, rejected/unmatched records, and loss-sensitive analysis. They do not
establish hardware timing or physical response.

From the workspace root:

```bash
PYTHONNOUSERSITE=1 colcon build --packages-select fr3_lab_stack \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
PYTHONNOUSERSITE=1 colcon test --packages-select fr3_lab_stack
colcon test-result --test-result-base build/fr3_lab_stack --verbose
```

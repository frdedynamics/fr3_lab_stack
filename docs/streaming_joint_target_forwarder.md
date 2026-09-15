# C1-A1: persistent streaming joint-target forwarder

Implemented on `27fd6ad` in `fr3_lab_stack`. Start after building and sourcing the workspace:

```bash
ros2 run fr3_lab_stack fr3_streaming_joint_target_forwarder --ros-args \
  --params-file "$(ros2 pkg prefix fr3_lab_stack)/share/fr3_lab_stack/config/streaming_joint_target_forwarder.yaml"
```

The persistent node subscribes to `/fr3_streaming_joint_target` and publishes the
same `sensor_msgs/msg/JointState` object to
`/fr3_streaming_joint_impedance_controller/target_joint` directly in the input
callback. Both endpoints use reliable, volatile, KEEP_LAST depth-1 QoS. Positions,
source timestamp, and other message fields are preserved.

## Admission and controller evidence

Names must be exactly `fr3_joint1` through `fr3_joint7`, in order. Positions must
contain exactly seven finite numbers. The timestamp must have nonnegative seconds,
normalized nanoseconds, and a strictly positive total value. Age uses the node's
ROS clock at callback entry; source and server must share that clock domain.

Read-only startup parameters (also recorded in startup logs and every target record):

| Parameter | Default | Meaning |
| --- | --- | --- |
| `target_max_age_s` | 0.25 s | Maximum source age, matching the controller |
| `target_future_tolerance_s` | 0.05 s | Maximum future offset, matching the controller |
| `controller_poll_interval_s` | 0.10 s | Steady-clock polling timer interval |
| `controller_evidence_expiry_s` | 0.30 s | Maximum monotonic age of active evidence |

Age and evidence limits include their boundaries. If overriding timestamp limits,
keep them aligned with the controller configuration.

The node asynchronously calls `/controller_manager/list_controllers`, allowing
at most one outstanding request. It admits targets only after a successful reply
identifies exactly one `fr3_streaming_joint_impedance_controller` as `active`.
Evidence age starts at request dispatch, conservatively including service latency.
A delayed reply cannot renew old evidence. Inactive/missing controllers, service
unavailability, or service errors revoke evidence. A hung request stays outstanding;
evidence expires and targets are rejected until it completes. Polling continues
independently of ROS time pauses. No controller switching occurs.

Evidence is a recent observation, not an atomic guarantee of controller state at
publication or delivery; the controller retains its own validation.

## Observability and timing

Every input emits a JSON log record and a `std_msgs/msg/String` JSON record on
`/fr3_streaming_joint_target_forwarder/telemetry` (reliable, volatile, depth 1).
Logs provide per-input records; a slow telemetry subscriber may miss records.
Counters are cumulative for the process lifetime:

- `received`: callbacks entered.
- `accepted`: messages passing validation and the active-evidence gate.
- `rejected`: messages failing admission; `rejection_counts` groups the first
  failing reason: `joint_names`, `position_count`, `nonfinite_position`,
  `invalid_timestamp`, `stale_timestamp`, `future_timestamp`,
  `controller_active_unconfirmed`, or `controller_evidence_expired`.
- `forwarded`: target `publish()` calls returning normally.
- `publication_errors`: target `publish()` calls raising an exception; these
  remain accepted, are not counted as forwarded, and are not retried.

Thus `received = accepted + rejected` and
`accepted = forwarded + publication_errors` after each callback's decision.
Telemetry publication failures are separately logged and do not alter target counts.

Each record includes source seconds/nanoseconds, server receipt ROS nanoseconds,
server receipt monotonic nanoseconds, controller status and evidence age, outcome,
rejection reason, parameters, and cumulative counters.

`publication_call_start_monotonic_ns` is sampled immediately before the target
publisher's Python `publish(msg)` invocation. `publication_call_end_monotonic_ns`
is sampled immediately after it returns or raises. Their difference is
`publication_call_duration_ns`. These fields are null for rejected targets.
This measures local API-call wall duration, including local blocking; it is **not
middleware delivery confirmation**, controller acceptance, or control-loop
application latency. `server_processing_duration_ns` measures callback entry
through decision and record preparation, before JSON serialization, logging, and
telemetry publication. All processing durations use `time.monotonic_ns()` and
remain independent of ROS clock jumps.

There are no application command queues, target retries, mappings, scheduling,
inference holds, interpolation, rescaling, or controller modifications.

## Software validation

Run these as separate commands from the workspace root after sourcing ROS and
`install/setup.bash`:

```bash
PYTHONNOUSERSITE=1 colcon test --packages-select fr3_lab_stack
colcon test-result --test-result-base build/fr3_lab_stack --verbose
```

The result directory limits the report to this package. Plain
`colcon test-result --verbose` includes saved results from every workspace package,
including failures from earlier runs of unrelated packages.

`test/test_streaming_joint_target_forwarder.py` exercises timestamp boundaries,
malformed targets, direct unchanged publication, timing and count semantics,
publication failures, expired/missing active evidence, delayed asynchronous replies,
one outstanding request, service failures, and QoS. These tests require ROS Python
packages but no robot. Hardware forwarding and delivery latency are not validated
by these tests.

Validation on 2026-09-15: package build succeeded; all six package test suites
passed, including 27 forwarder tests. The ROS graph test uses localhost domain
173 and a mock ListControllers service to verify unchanged message delivery and
rejection after an inactive response. No hardware commands were sent.

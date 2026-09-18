# Asynchronous Franka Hand Move/Grasp client

`fr3_lab_stack_runtime.franka_hand.RosFrankaHand` is a reusable,
policy-independent library attached to a caller-owned ROS node/executor. It
accepts total gripper width in metres, explicit speed in m/s, and a positive
lifecycle timeout. The wrapper contains no DROID threshold, policy semantics,
arm-controller logic, or task-success logic.

The client exposes two explicit command primitives:

- `request_move(width)` for free-space opening/positioning through
  `/franka_gripper/move`;
- `request_grasp(width, force, epsilon_inner, epsilon_outer)` for force-controlled
  closing through `/franka_gripper/grasp`.

Both methods only queue intent. A 5 ms executor timer advances asynchronous ROS
action/service futures without blocking or spinning internally. The caller owns
the executor lifetime, must check latched failures before issuing further robot
actions, and must keep the executor alive through bounded cleanup.

## ROS and franka_ros2 contract

The inspected local `franka_ros2` source revision is:

```text
1369a2cb200d0f7b3da11c7728c7ca2e6975ca00
```

with `franka_gripper` 2.0.2.

The wrapper uses:

```text
/franka_gripper/move   franka_msgs/action/Move
/franka_gripper/grasp  franka_msgs/action/Grasp
/franka_gripper/stop   std_srvs/srv/Trigger
```

`Move` carries width and speed. `Grasp` carries width, speed, force, and explicit
inner/outer epsilon. `Stop` calls the Franka Hand stop service.

`control_msgs/action/GripperCommand` also exists in the underlying stack, but is
not used here because its closing behavior is not the same contract as the
explicit object-agnostic `Grasp` path required by the policy adapter.

## Lifecycle and replacement semantics

The local Franka gripper action server can have asynchronous action work in
flight; the wrapper therefore never assumes that sending a new goal preempts the
old one.

For a replacement while an action is active, the wrapper:

1. requests cancellation;
2. records the cancellation response;
3. waits for the old terminal action result;
4. only then issues the latest queued replacement.

A natural success/cancel race is accepted. A rejected cancellation does not
release a still-active goal. The latest queued command wins, while exact duplicate
commands are suppressed even after completion.

Move and Grasp are different commands even when they use the same target width.

Failures are latched. Rejected goals, unsuccessful action results, lifecycle
timeouts, transport exceptions, or failed stop requests suppress further
commands until the caller handles the failure. There is no automatic retry.

## Grasp hold and release

A successful `Grasp` is treated as an active physical grasp state. The wrapper
records `holding_grasp = True` after a successful Grasp result.

When a later command is queued while a grasp is being held, the wrapper first
issues a nonterminal `/franka_gripper/stop` request. Only after a successful stop
result is recorded is the next command allowed to start.

For the normal release sequence this gives:

```text
successful Grasp
→ hold
→ queued Move(open)
→ release_stop_requested
→ release_stop_result(success=true)
→ Move(open)
```

This avoids overlapping an opening Move with the force-holding grasp state.

A failed release stop latches `gripper_release_stop_failed` and blocks the
following Move.

Explicit shutdown still uses the terminal stop path. The wrapper retains
separate evidence for normal grasp release and terminal cleanup.

## Evidence

`evidence()` returns detached request and lifecycle records including:

- request ID;
- command type (`move` or `grasp`);
- target width;
- speed;
- force and epsilon for Grasp;
- request/issue timestamps;
- disposition;
- goal acceptance;
- cancellation request/response;
- terminal result;
- release-stop request/result;
- terminal stop request/result;
- latched error state;
- settled state.

A queued request is not counted as an issued physical command.

Callers may add policy values, measured gripper state, task semantics, or
higher-level validation evidence. The lower-level wrapper itself remains
policy-independent.

## Validation status

Software validation covers:

- nonblocking queueing;
- duplicate suppression;
- Move/Grasp distinction at identical width;
- explicit Grasp parameter dispatch;
- replacement and cancellation ordering;
- rejected cancellation;
- completion/cancel race;
- transport errors;
- goal rejection;
- failed action results;
- stop failure;
- grasp-hold tracking;
- successful Grasp → Stop → Move(open);
- failed release stop blocking the open Move.

The focused hand suite contains 16 tests and is also registered in the normal
`colcon test` package suite.

The complete `fr3_lab_stack` package was validated after the Move+Grasp changes
with:

```text
120 tests
0 errors
0 failures
0 skipped
```

Physical G1B validation was then performed by the SAPS physical runtime using
the wrapper in one continuous session. The qualifying run executed:

```text
Move(open)
→ Grasp(width=0, speed=0.1 m/s, force=20 N,
        epsilon_inner=0.001 m, epsilon_outer=0.08 m)
→ unsupported object retention for approximately 3 s
→ Stop
→ Move(open)
```

The grasp completed at approximately 46.25 mm total width. Across the 3 s dwell
the measured width span was approximately 0.069 mm. The object remained
physically retained without support. Release stop succeeded, the hand reopened
to approximately 79.80 mm, the arm hold remained unchanged, and no Franka health
violation was observed.

The object width was not supplied to the wrapper, validator, or policy adapter.
The Grasp force, speed, and epsilon values are fixed embodiment parameters rather
than object-specific inputs.

The qualifying physical evidence is retained by the SAPS physical runtime and
is intended to be archived with the corresponding G1B validation record.

## Tests

Focused hand tests:

```bash
python3 -m unittest discover -s test -p 'test_franka_hand.py' -v
```

Full package regression:

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws/install/setup.bash

colcon build     --packages-select fr3_lab_stack     --cmake-args -DAMENT_CMAKE_SYMLINK_INSTALL=OFF

colcon test     --packages-select fr3_lab_stack     --event-handlers console_direct+

colcon test-result     --test-result-base build/fr3_lab_stack/test_results
```

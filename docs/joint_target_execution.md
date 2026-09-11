# FR3 Joint-Target Execution

## Purpose

`fr3_lab_stack` provides a policy-agnostic robot-side primitive for realizing an
absolute seven-joint FR3 target through the standard MoveIt execution path.

The package does **not** interpret OpenPI, DROID, SAPS, or other policy actions.
Policy-specific action semantics remain outside this repository.

The validated execution chain is:

```text
absolute q_target [7]
        |
        v
/plan_kinematic_path
        |
        v
validated RobotTrajectory
        |
        v
/execute_trajectory
        |
        v
fr3_arm_controller
        |
        v
FR3
```

The implementation intentionally keeps the standard
`fr3_arm_controller` active. It does not switch controllers, publish directly
to `/fr3_arm_controller/joint_trajectory`, invoke MoveIt Servo, or use a
separate position controller.

## Current executable

The validated executable is:

```bash
ros2 run fr3_lab_stack fr3_joint_target
```

It accepts either an absolute seven-joint target or, for commissioning, a
relative seven-joint delta that is converted to an absolute target from a fresh
measured state.

Planning is the default behavior. Physical execution requires the explicit
`--execute` flag.

Example plan-only invocation:

```bash
ros2 run fr3_lab_stack fr3_joint_target   --delta 0.01 0 0 0 0 0 0
```

Example explicit execution:

```bash
ros2 run fr3_lab_stack fr3_joint_target   --delta 0.01 0 0 0 0 0 0   --execute
```

## Robot-side responsibilities

The utility owns robot-specific target realization:

- acquisition of fresh FR3 joint and Franka robot state,
- controller and ROS-graph ownership checks,
- validation of target and joint limits,
- MoveIt planning through `/plan_kinematic_path`,
- validation of the returned `RobotTrajectory`,
- a final pre-execution state/health check,
- execution through `/execute_trajectory`,
- measured execution telemetry,
- final state and Franka-health evidence.

The implementation permits at most one execution goal per invocation and does
not automatically retry or issue a hold trajectory after a failure.

## Safety and admissibility checks

Before planning or execution, the utility fails closed when required state or
control conditions are not satisfied.

The current checks include:

- fresh and finite FR3 joint telemetry,
- stationary-arm requirement,
- active standard `fr3_arm_controller`,
- no competing direct arm trajectory publisher,
- no active Servo/SpaceMouse command path,
- commandable Franka robot mode,
- no active `current_errors`,
- no active Franka collision indicators,
- valid joint position and velocity limits from the live robot description,
- enabled MoveIt joint acceleration limits,
- target position within joint bounds,
- validated trajectory start, ordering, timestamps, endpoint, velocity, and
  acceleration,
- pre-execution drift from the planning state no greater than `0.002 rad`.

Contact indicators and `last_motion_errors` are retained as evidence but are
not, by themselves, permanent rejection conditions. This distinction is
important for later manipulation experiments in which task contact can be
legitimate.

## Validated planning configuration

The current commissioning configuration uses:

```text
MoveIt group:                    fr3_arm
planning attempts:               1
allowed planning time:           2.0 s
velocity scaling factor:         0.1
acceleration scaling factor:     0.1
joint goal tolerance:            1e-4 rad
stationary threshold:            0.01 rad/s
pre-execution drift tolerance:   0.002 rad
```

The trajectory validator uses position and velocity bounds from the live FR3
URDF and acceleration limits from the live MoveIt
`robot_description_planning.joint_limits` parameters.

## Physical validation

The repository implementation was validated on the lab FR3 using a small
commissioning target:

```text
q_target = q_measured + [0.01, 0, 0, 0, 0, 0, 0] rad
```

### Plan-only regression

The repository implementation reproduced the previously commissioned planning
behavior:

```text
MoveIt result:              SUCCESS
trajectory points:          5
trajectory duration:        0.327501 s
J1 planned peak velocity:   0.047625 rad/s
J1 planned peak accel.:     0.375000 rad/s^2
maximum endpoint error:     8.54e-05 rad
execution attempts:         0
```

### Physical execution regression

A single explicit execution of the same commissioning target succeeded:

```text
ExecuteTrajectory status:   SUCCEEDED
MoveIt result:              SUCCESS
execution attempts:         1
trajectory points:          5
trajectory duration:        0.325053 s
J1 planned peak velocity:   0.046875 rad/s
J1 planned peak accel.:     0.375000 rad/s^2
J1 measured peak velocity:  0.057912 rad/s
maximum final target error: 0.001050 rad
```

The pre-execution configuration drift was approximately `5e-6 rad`, and no
Franka current error, reflex, collision indicator, or contact indicator was
present before or after execution. `control_command_success_rate` remained
`1.0`.

Measured derivative peaks are sampled telemetry values and are therefore not
assumed to be exact continuous-time maxima. Planned and measured dynamics are
reported separately.

## Intended SAPS boundary

The lab stack must remain independent of DROID/OpenPI action semantics.

For the SAPS physical integration, SAPS should:

1. obtain a fresh measured arm state `q_ref`,
2. select the next policy action,
3. compute the DROID target

   ```text
   q_desired = q_ref + 0.2 * clip(u, -1, 1)
   ```

4. apply policy-side and experiment-specific admissibility checks,
5. send `q_ref` and `q_desired` to the lab-stack executor,
6. receive the execution result and measured final state,
7. obtain a new fresh measured state before constructing the next DROID target.

`fr3_lab_stack` must not receive the full OpenPI action chunk and must not
reimplement the DROID target conversion. If the robot state has drifted beyond
the accepted tolerance relative to the supplied `q_ref`, the executor should
reject the request rather than recompute or alter `q_desired`.

This keeps the scientific policy semantics in `saps-openpi-replication` and the
robot-specific realization in `fr3_lab_stack`.

## Planned persistent interface

The next implementation step is a persistent single-target ROS action server
around the validated primitive.

The intended request contains:

```text
reference_q[7]
target_q[7]
request identifier / timestamp
```

The intended result contains:

```text
outcome
final_q[7]
final_dq[7]
planning status
execution status
robot-health evidence
```

Only one target should be processed at a time. The server must not internally
consume an OpenPI action chunk or queue multiple DROID actions autonomously.

A launch file will start this server for later SAPS integration.

## Temporal limitation

The current MoveIt realization is deliberately conservative and is not yet a
15 Hz DROID control realization.

The validated single target took approximately `0.325 s`, whereas the nominal
DROID control interval at 15 Hz is approximately `0.0667 s`.

Therefore, sequentially planning and executing each of the eight policy actions
currently selected from a pi0.5 action chunk would not reproduce the original
DROID temporal realization.

This is a separate follow-on problem. The policy-side state/action semantics
should remain unchanged while a faster robot-side realization is investigated.

## Scope status

Validated now:

```text
absolute target
-> MoveIt planning
-> trajectory validation
-> one explicit execution
-> measured result
```

Not yet validated:

```text
persistent action server
SAPS client integration
one real pi0.5/DROID-derived physical action
multi-action execution
15 Hz-compatible realization
recovery/resumption
```

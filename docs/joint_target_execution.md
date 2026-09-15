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

## Current interfaces

### Command-line executable

The validated one-shot executable is:

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
ros2 run fr3_lab_stack fr3_joint_target \
  --delta 0.01 0 0 0 0 0 0
```

Example explicit execution:

```bash
ros2 run fr3_lab_stack fr3_joint_target \
  --delta 0.01 0 0 0 0 0 0 \
  --execute
```

### Persistent action server

The validated persistent interface is:

```bash
ros2 launch fr3_lab_stack joint_target_server.launch.py
```

The server defaults to plan-only operation:

```text
execute=False
```

Physical execution must be enabled explicitly:

```bash
ros2 launch fr3_lab_stack joint_target_server.launch.py execute:=true
```

The ROS action name is:

```text
/fr3_joint_target
```

with type:

```text
fr3_lab_stack_interfaces/action/ExecuteJointTarget
```

The request contains:

```text
string request_id
float64[7] reference_q
float64[7] target_q
builtin_interfaces/Time reference_stamp
```

The result contains success/outcome, final measured joint state, MoveIt
planning/execution status, whether execution was attempted, and strict JSON
evidence. Feedback reports the current stage and measured arm state.

Only one target is processed at a time. A concurrent goal is rejected rather
than queued. The first server version deliberately rejects external
cancellation requests; runtime faults or timeouts can still cancel an
underlying MoveIt execution.

`fr3_lab_stack_interfaces` is currently kept as a local sibling ROS package in
the lab workspace. `fr3_lab_stack` therefore has a local workspace dependency
on that package until its long-term repository layout is finalized.

## Robot-side responsibilities

The utility and persistent server own robot-specific target realization:

- acquisition of fresh FR3 joint and Franka robot state,
- controller and ROS-graph ownership checks,
- validation of target and joint limits,
- validation that a supplied `reference_q` still matches fresh measured state,
- MoveIt planning through `/plan_kinematic_path`,
- validation of the returned `RobotTrajectory`,
- a final pre-execution state/health check,
- execution through `/execute_trajectory`,
- measured execution telemetry,
- final state and Franka-health evidence.

The one-shot utility permits at most one execution goal per invocation. The
persistent server permits at most one active target and one underlying
execution attempt per accepted goal. Neither path automatically retries or
issues a hold trajectory after a failure.

## Safety and admissibility checks

Before planning or execution, the runtime fails closed when required state or
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
- supplied reference timestamp no older than the configured freshness bound,
- measured planning state within `0.002 rad` of supplied `reference_q`,
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
telemetry/reference freshness:   0.25 s
reference-state tolerance:       0.002 rad
pre-execution drift tolerance:   0.002 rad
execution timeout:               5.0 s
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

### One-shot CLI plan-only regression

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

### One-shot CLI physical execution regression

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

### Persistent server plan-only regression

A real ROS action request was sent using a fresh measured `reference_q` and
timestamp, with the server running in its default `execute=False` mode.

```text
action result:                   SUCCEEDED
outcome:                         plan_only_validated
reference age at execution:      0.001931 s
max |planning_q - reference_q|:  3.68e-06 rad
execution attempts:              0
trajectory points:               5
trajectory duration:             0.327063 s
J1 planned peak velocity:        0.047625 rad/s
J1 planned peak accel.:          0.375000 rad/s^2
maximum endpoint error:          8.56e-05 rad
```

The requested absolute target preserved the exact commissioning displacement:

```text
target_q[0] - reference_q[0] = 0.01 rad
```

### Persistent server physical execution regression

The same request path was then validated with `execute=True`. Exactly one
MoveIt execution attempt was made.

```text
action result:                   SUCCEEDED
outcome:                         execution_succeeded
MoveIt planning result:          SUCCESS
MoveIt execution result:         SUCCESS
execution attempts:              1
reference age at execution:      0.001703 s
max |planning_q - reference_q|:  4.92e-06 rad
max pre-execution start drift:   5.04e-06 rad
trajectory points:               5
trajectory duration:             0.327321 s
J1 planned peak velocity:        0.047625 rad/s
J1 planned peak accel.:          0.375000 rad/s^2
J1 measured peak velocity:       0.060924 rad/s
maximum final target error:      0.001272 rad
maximum final |dq|:              0.001807 rad/s
telemetry errors:                0
```

No Franka current error, reflex, collision indicator, or contact indicator was
present before or after execution. `control_command_success_rate` remained
`1.0`. No retry or second hold/motion command was issued.

Measured derivative peaks are sampled telemetry values and are therefore not
assumed to be exact continuous-time maxima. Planned and measured dynamics are
reported separately.

## Manual server regression client

The commissioning action request used during validation is retained as:

```text
scripts/manual_joint_target_server_regression.py
```

This is a manual regression/commissioning helper, not the production SAPS
client and not an automatically executed test. It always constructs the known
commissioning target from a fresh `/joint_states` sample:

```text
target_q = reference_q + [0.01, 0, 0, 0, 0, 0, 0] rad
```

The helper queries the server's `execute` parameter before sending a goal. If
physical execution is enabled, it refuses to continue unless the operator also
passes `--allow-execution`. This prevents a copied plan-only command from
silently becoming a physical-motion command.

The eventual SAPS client must be implemented separately because it owns the
DROID/OpenPI action interpretation and experiment-specific admissibility
checks.

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
the accepted tolerance relative to the supplied `q_ref`, the executor rejects
the request rather than recomputing or altering `q_desired`.

This keeps the scientific policy semantics in `saps-openpi-replication` and the
robot-specific realization in `fr3_lab_stack`.

## Temporal evaluation and controller transition

The MoveIt/`fr3_arm_controller` path remains the validated conservative baseline for commissioning, pose reset, and isolated targets, but it does not satisfy the DROID temporal requirement.

A small commissioning target typically required about `0.33 s` of planned trajectory time. More decisively, a real π0.5/DROID-derived physical action required:

```text
planned trajectory duration: 1.1048 s
request-to-result time:       1.6032 s
DROID action period:          0.0667 s
eight-action DROID window:    0.5333 s
```

Sequential MoveIt planning/execution would therefore change the policy's temporal realization substantially. The trajectory path is retained as a baseline and safe reset mechanism rather than used for 15 Hz policy execution.

## Streaming hybrid impedance realization

For policy-rate execution, `fr3_lab_stack` now provides:

```text
fr3_lab_stack/StreamingJointImpedanceController
```

The controller runs in the existing 1 kHz `ros2_control` loop, claims the seven FR3 effort interfaces, and accepts the latest absolute seven-joint equilibrium target. On activation it initializes `q_desired` from measured joint state and produces no internal motion. Each valid external target atomically replaces `q_desired`; targets are not queued, interpolated, rescaled, or converted into trajectories. If updates stop, the last equilibrium target is held.

Hybrid mode uses the Polymetis-style structure

```text
Kp(q) = J(q)^T Kx J(q) + Kq
Kd(q) = J(q)^T Kxd J(q) + Kqd
tau   = Kp(q)(q_desired - q) - Kd(q)dq + coriolis
```

with the configured gains:

```text
Kq  = [40, 30, 50, 25, 35, 25, 10]
Kqd = [4, 6, 5, 5, 3, 2, 1]
Kx  = [750, 750, 750, 15, 15, 15]
Kxd = [37, 37, 37, 2, 2, 2]
```

Policy semantics remain outside this package. For DROID/π0.5 integration, the policy layer still constructs each new target from fresh measured state:

```text
q_desired = q_measured + 0.2 * clip(u, -1, 1)
```

The controller only realizes the supplied absolute target.

## Streaming controller commissioning

A preliminary fixed-gain joint-impedance version was rejected after a zero-target activation produced `13.30 mrad` maximum pose drift. The hybrid controller reduced the same commissioning metric to `2.90 mrad`, with no external target applied.

Static J1 characterization showed a small equilibrium offset rather than a proportional tracking loss. After returning to approximately the same starting configuration:

```text
command      steady displacement    steady residual
+5 mrad      2.936 mrad             2.064 mrad
+30 mrad     27.928 mrad            2.072 mrad
```

These values characterize the tested J1 configuration only; they are not assumed to be universal across joints or configurations.

The first 15 Hz replacement test sent two targets using fresh measured-state anchoring at each tick:

```text
requested period:                 66.667 ms
publisher interval:               66.807 ms
publisher interval error:         +0.141 ms
accepted target sequences:        1, 2
target-2 displacement from q_measured: 5.000 mrad
desired-target mismatch:          0
```

The controller therefore supports the required *functional* target-replacement pattern. Current state telemetry is published at 100 Hz: the first observed target ages (`6.9--10.7 ms` in the two-target test) are telemetry-quantized observations, not precise target receipt-to-application latency measurements. Exact callback receipt, first-control-cycle application, and 1 kHz controller-period/jitter instrumentation remain follow-on work before quantitative timing claims or the prerecorded eight-action DROID test.

## Scope status

Validated now:

```text
absolute target -> MoveIt/JTC single-target execution
persistent one-goal-at-a-time MoveIt action server
hybrid streaming impedance controller build/tests/plugin loading
zero-target hybrid activation
single-target hybrid execution
two fresh-state-anchored target replacements at 15 Hz
static J1 equilibrium characterization over 5--30 mrad commands
```

Next:

```text
precise target receipt/application and controller-period timing instrumentation
prerecorded eight-action DROID sequence at 15 Hz
live π0.5 multi-action execution
SAPS arbitration / recovery / resumption integration
```

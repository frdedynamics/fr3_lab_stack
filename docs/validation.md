# Validation record — 2026-09-08

## Current status

**Overall: PARTIAL / INCOMPLETE.**

Software validation passed. Dual-camera operation was stable during the latest **525.137 s** integrated observation. The required **15 uninterrupted minutes** of full physical acceptance have not yet been completed.

## Latest integrated observation

| Item | Result |
| --- | --- |
| Start | `2026-09-08T13:15:39.145169+02:00` |
| End | `2026-09-08T13:24:24.281703+02:00` |
| Duration | **525.137 s — 8 min 45.137 s** |
| Planned duration | 900 s |
| Outcome | Interrupted by MoveIt Servo singularity protection |

### Camera performance

| Metric | Wrist | External |
| --- | ---: | ---: |
| Serial | `342222073510` | `244222076317` |
| CameraInfo samples | 15,743 | 15,743 |
| Mean observed rate | 29.978531 Hz | 29.978359 Hz |
| Median 5 s rate | 29.979239 Hz | 29.979359 Hz |
| Maximum arrival gap | 42.536 ms | 42.295 ms |
| Stall / restart / disconnect | None | None |

Both RViz RGB panes remained visible and updating throughout the monitored interval.

### Manual control checks

The operator completed the requested:

- ±X / ±Y / ±Z translation checks;
- ±Rx / ±Ry / ±Rz rotation checks;
- gripper open/close/repeated-toggle checks.

No `communication_constraints_violation`, joint/cartesian reflex caused by contact, controller timing failure, controller deactivation, or camera-node restart was captured during this observation.

## Singularity event and recovery interpretation

MoveIt Servo entered `HALT_FOR_SINGULARITY` when the arm approached a singular configuration.

During the monitored run, Cartesian teleoperation did not successfully move the arm out of the singular region and the 15-minute qualification was therefore not completed.

After the monitored interval, the operator used MoveIt planning to move the arm to a nonsingular configuration. SpaceMouse Cartesian teleoperation then resumed normally.

This supports interpretation of the event as **expected singularity protection**, not a persistent Servo, controller, Franka communication, or SpaceMouse failure. The recovery was an operator observation after the monitored interval and was not captured by the validation observer.

A future acceptance run should remain in a comfortable workspace. If a singularity halt occurs, use the documented MoveIt recovery procedure and restart the uninterrupted 15-minute observation window.

## USB topology

The latest candidate stable topology is:

| Role | RealSense serial | USB path | Controller |
| --- | --- | --- | --- |
| Wrist | `342222073510` | `6-1` | `0000:78:00.0` |
| External | `244222076317` | `6-2` | `0000:78:00.0` |

Both devices remained continuously enumerated during the 525.137 s observation. No USB/xHCI/UVC/RealSense kernel event occurred inside the exact observation window.

### Historical USB issue

An earlier integrated run lost the wrist camera when its previous `10-1` xHCI path failed. After moving the wrist camera to the current `6-1` port, no USB/xHCI event or camera interruption occurred during the subsequent 525-second integrated observation.

This supports the current topology as the preferred configuration, but does not establish a definitive hardware root cause and does not replace the pending 15-minute qualification.

## Software validation

- package-select build passed;
- dual-camera launch evaluation passed;
- fixed wrist/external serial binding passed;
- RGB8 `1280x720x30` configuration passed;
- depth-disabled default passed;
- optional depth produced images from both devices;
- RViz configuration includes both RGB image displays while preserving the MoveIt visualization;
- project checks/tests passed.

The initial build selected Miniconda Python and failed because required ROS Python packages were unavailable there. Rebuilding with `/usr/bin/python3` resolved the issue; no additional Python packages were installed.

## Git provenance for latest monitored run

| Repository / Git root | HEAD |
| --- | --- |
| `~/franka_ros2_ws/src/fr3_lab_stack` | `7795d7ad6aedc33465fc75edba2624a45d40b045` |
| `~/franka_ros2_ws/src/igd_fr3_control` | `438e4ae3145944042033395940848bf107cb9965` |
| Franka root `~/franka_ros2_ws/src` | `1369a2cb200d0f7b3da11c7728c7ca2e6975ca00` |

No dependency modification was made by the validation observer.

## Raw evidence

Detailed evidence for the latest full-stack observation is stored under:

```text
docs/validation/2026-09-08_full_stack_acceptance_131539/
```

This directory contains the machine-readable event stream, manual observations, camera cadence data, node/process snapshots, USB/kernel captures, repository provenance, and integrity manifest.

The raw evidence should be used for detailed debugging or audit. This document intentionally keeps only the current commissioning conclusions and the evidence needed to interpret them.

## Remaining acceptance item

Repeat the complete FR3 + dual-RGB + MoveIt + Servo + SpaceMouse + gripper stack for **15 uninterrupted minutes** using the procedure in [`commissioning.md`](commissioning.md).

The stack should be considered fully commissioned only after that run completes without camera/USB failure, robot communication failure, unexpected controller failure, or unrecoverable Servo state.

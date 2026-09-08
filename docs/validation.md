# Validation record — 2026-09-08

## Scope and environment

Repository/package: `fr3_lab_stack` (operator explicitly selected this name over
the inconsistent `fr3_research_bringup` name in the original brief).
Local repository branch `main`, contributor identity
`ehsann90 <ehsan.kh69@gmail.com>` taken from the existing lab repository.
Intended organization: `frdedynamics`. No hosted repository, remote, push, or
GitHub access grant has been made; organization ownership is not yet established.

ROS 2 Jazzy; installed RealSense ROS 4.57.7 and librealsense 2.57.7 confirmed
by package manager and camera startup logs. USB enumeration found:

| Role | Serial | Firmware | USB |
| --- | --- | --- | --- |
| Wrist | 342222073510 | 5.13.0.55 | 3.2 |
| External | 244222076317 | 5.17.3.10 | 3.2 |

Firmware, drivers, QoS, synchronization, and timestamp settings were not changed.

## Software results

- Package-select build passed. Initial plain build selected Miniconda Python
  and failed with missing `catkin_pkg`, plus a missing `pytest` warning.
  Reconfiguration with `-DPython3_EXECUTABLE=/usr/bin/python3` resolved this;
  subsequent `PYTHONNOUSERSITE=1 colcon build --packages-select fr3_lab_stack`
  passed. A redundant `PYTHON_EXECUTABLE` option in the recovery command produced
  an unused-variable CMake warning; it is not required.
- Four pytest cases passed: actual upstream launch evaluation with default,
  explicit false, and true depth arguments; RViz image display validation.
  CTest reports five entries when counting its aggregate test as well.
- Initial tests hit sandbox restrictions writing `~/.ros/log`; test logs now
  live under the package build directory. A test helper initially failed to
  normalize a LaunchConfiguration; corrected before the passing test run.
- `--show-args` succeeded. It lists included upstream arguments as well as the
  project depth argument; upstream printed defaults do not describe the fixed
  values passed by each include. The evaluated launch tests check actual values.
- Semantic comparison against the current Franka RViz file confirmed all
  non-Image displays and all other Visualization Manager settings are identical.
- Only the new package was built; this was not a clean rebuild of all dependencies.

## Initial camera run

The operator stopped the two pre-existing camera launches. The new single
`ros2 launch fr3_lab_stack dual_realsense.launch.py` command started both nodes.
Both logged the correct **Device Serial No**, USB 3.2, sync off, and active
Color profile RGB8 / 1280×720 / 30 FPS. Enumeration logs can list both devices
before selection; use the selected Device Serial No line to establish identity.

Both nodes' queried parameters matched:

```text
serial_no: _342222073510 (wrist), _244222076317 (external)
rgb_camera.color_profile: 1280x720x30
rgb_camera.color_format: RGB8
enable_depth: false
enable_gyro: false
enable_accel: false
```

No `/camera/{wrist_camera,external_camera}/depth/...` topics were advertised in
this default run, and only the RGB sensor was started. The
`extrinsics/depth_to_color` topic remained advertised; it is calibration metadata,
not a depth image stream. Initialization of depth profiles and IMU sample-rate
parameters in the driver log does not mean those streams are enabled.

Simultaneous 15-second CameraInfo rate probes:

- Wrist: final average **29.978 Hz**, 421-message window; displayed averages
  29.971–29.980 Hz.
- External: **no CameraInfo samples**, no rate available. Repeated librealsense
  and ROS warnings: `Frames didn't arrived within 5 seconds` / `Frames Timeout`.
  A clean stop/restart reproduced this. USB reconnection was requested.
- The no-data CLI probe emitted an invalid-context/wait-set error when timeout
  terminated it; no driver process crash occurred. Intentional camera shutdowns
  produced the normal launch SIGINT warning and clean process exits.

Initial launch logs:
`~/.ros/log/2026-09-08-11-49-13-175075-hvl-robotics2404-59573/launch.log`
and `~/.ros/log/2026-09-08-11-51-01-104826-hvl-robotics2404-60397/launch.log`.

## Integrated acceptance status

USB reconnection cleared the initial external RGB timeout. A subsequent default
run opened both cameras without warnings. Four simultaneous 15-second CLI probes
measured final CameraInfo averages **29.982 Hz wrist / 29.981 Hz external**;
image CLI averages were **26.768 Hz wrist / 17.561 Hz external**. Both one-shot
image subscriptions confirmed `rgb8` and reported one lost message on startup.
The lower full-image CLI rates remain a receiver-performance observation to
investigate with the lightweight serialized-message checker; CameraInfo rates
alone are not presented as proof of full RGB delivery.

The opt-in launch was tested on hardware: both `enable_depth` parameters became
true and both depth topics delivered stamped images with the corresponding
`*_depth_optical_frame`. Default depth profile was Z16 / 848×480 / 30 FPS.
The external RGB timeout returned after this restart despite working depth.
Both camera processes were stopped cleanly and a second USB reconnection was
requested before restoring the default RGB-only launch. No reset workaround,
firmware change, package upgrade, or dependency edit was introduced.

After a second external USB reconnection, the RGB-only launch was restored and
both depth parameters were verified false. The lightweight RGB checker measured
**28.993 Hz wrist / 28.936 Hz external** over 20 seconds, with 550/548 received
1280×720 `rgb8` images and a maximum arrival gap of 0.368 seconds for each.

The operator loaded the project config through File → Open Config in the existing
MoveIt RViz process and confirmed: **both images visible; starting the 15-minute
test**. A process check found one RViz and one Servo process. ROS graph warnings
about duplicate `rviz2` and `servo_node` names reflected multiple nodes within
those processes, not two launches of either executable.

The physical run was interrupted after a few minutes and **did not pass the
15-minute acceptance test**:

- During the first 45 seconds of monitoring, both CameraInfo streams continued
  with maximum arrival gaps below 0.035 seconds while all seven arm joint
  positions changed. Servo logged deceleration when approaching/leaving a
  singularity. This confirms concurrent telemetry during motion, not subjective
  smoothness of every SpaceMouse axis.
- Between the 45- and 90-second monitor reports, libfranka reported
  `Move command aborted: motion aborted by reflex! ["cartesian_reflex"]`.
  Controller manager deactivated the hardware and arm/state controllers because
  that exception surfaced in the read cycle. The operator explicitly confirmed
  the robot contacted an object while attempting a grasp, triggering the stop.
  **This is not evidence of an Ethernet communication dropout.**
- The wrist camera then reported frame and UVC control timeouts. Kernel messages
  at 12:02:31 CEST report xHCI controller `0000:7a:00.4` not responding, being
  assumed dead, and USB `10-1` disconnecting. The camera driver reported
  `No such device` while queueing/stopping frames. The root cause of the USB
  controller failure and any relationship to robot contact are unestablished.
- A 20-second image probe overlapping the failure measured **15.876 Hz wrist**
  with an **8.483-second arrival gap**, versus **29.995 Hz external** with a
  0.053-second maximum gap. The longer CameraInfo monitor recorded a wrist gap
  of 9.607 seconds before disconnection. Startup/recovery rates cannot be treated
  as sustained integrated wrist performance.
- Finger positions changed by about 0.03021 m during the monitored interval;
  this does not establish successful button-toggle open/close cycles.

Recorded kernel messages:

```text
12:01:56 usb 10-1: Failed to query (SET_CUR) UVC control 1 on unit 3: -110
12:02:09 usb 10-1: Failed to query (SET_CUR) UVC control 1 on unit 3: -110
12:02:31 xhci_hcd 0000:7a:00.4: xHCI host not responding to stop endpoint command
12:02:31 xhci_hcd 0000:7a:00.4: xHCI host controller not responding, assume dead
12:02:31 xhci_hcd 0000:7a:00.4: HC died; cleaning up
12:02:31 usb 10-1: USB disconnect, device number 2
12:02:49 xhci_hcd 0000:7a:00.4: WARNING: Host Controller Error
```

See [camera log](validation/2026-09-08-cameras.log) and
[read-only monitor log](validation/2026-09-08-monitor.log) for the captured
runtime messages, including the full controller exception. The monitor's final
KeyboardInterrupt is from deliberately stopping the diagnostic process.
The camera log removes ANSI escapes and compacts consecutive repeats with their
counts and last occurrence: the disconnected wrist driver emitted 58,498 copies
of the same frame-queue `No such device` error. The raw 10 MB capture remains at
`/tmp/fr3-integrated-cameras.log` on the commissioning host.
Both camera nodes were then stopped and exited cleanly. Robot/Servo/SpaceMouse
processes were left under operator control; no robot commands or fault recovery
commands were sent by this package or the validation monitor.

Still unverified: smooth XYZ translation and XYZ rotation across the full
checklist, button-1 open/close toggle, sustained simultaneous RGB updates during
motion, a completed 10–15 minute integrated run, and absence of communication
dropouts over that full interval. Recovery of the wrist USB connection and an
operator-supervised repeat test are required. No dependency modification was
attempted to address these hardware/physical-test failures.

## Repository isolation baseline

Actual Franka Git root: `/home/hvl-robotics2404/franka_ros2_ws/src`.
Pre-existing status:

```text
 M franka_fr3_moveit_config/rviz/moveit.rviz
?? franka_fr3_moveit_config/config/pilz_cartesian_limits.yaml
?? igd_fr3_control/
?? olvx_descriptions_module/
```

Pre-existing `src/igd_fr3_control` status:

```text
 M igd_fr3_control/view_capture.py
?? igd_fr3_control/display_grasp_rviz.py
?? igd_fr3_control/test_grasp.py
?? igd_fr3_control/view_capture2.py
```

Both tracked diffs were compared byte-for-byte with the inspection baseline;
the igd status was also identical. The only intended additional parent status
entry is `?? fr3_lab_stack/`. No parent Git index, configuration, or ignore file
was edited. No existing source file was modified by this task; generated
workspace build/install/log artifacts were produced by colcon.

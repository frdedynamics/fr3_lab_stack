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

Pending camera recovery and physical test. No claim yet of simultaneous visible
RGB in RViz, image updates during arm motion,
or a 10–15 minute integrated FR3/Servo/SpaceMouse/gripper run. Prior dependency
commissioning results and supplied historical camera rates are not substituted
for this run. Translation/rotation smoothness, gripper toggle, Servo operation,
and absence of new ros2_control dropouts require operator observation and logs.

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

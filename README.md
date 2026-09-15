# FR3 Lab Stack

ROS 2 bringup, validation, and robot-side execution utilities for the HVL FR3 lab setup. The package provides reproducible dual-RealSense RGB bringup and RViz integration, together with validated FR3 joint-target execution paths used during learned-policy deployment.

## Tested environment

- Ubuntu 24.04
- ROS 2 Jazzy
- `realsense2_camera` 4.57.7
- librealsense 2.57.7
- Franka FR3 with gripper
- MoveIt Servo + SpaceMouse teleoperation

## Build

```bash
cd ~/franka_ros2_ws
source /opt/ros/jazzy/setup.bash

PYTHONNOUSERSITE=1 colcon build --packages-select fr3_lab_stack
source install/setup.bash
```

If CMake selects the Miniconda interpreter and reports missing ROS Python packages, rebuild once with:

```bash
PYTHONNOUSERSITE=1 colcon build --packages-select fr3_lab_stack \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

## Dual RealSense cameras

Start both cameras:

```bash
ros2 launch fr3_lab_stack dual_realsense.launch.py
```

Default configuration:

| Role | Serial | RGB topic |
| --- | --- | --- |
| Wrist | `342222073510` | `/camera/wrist_camera/color/image_raw` |
| External | `244222076317` | `/camera/external_camera/color/image_raw` |

Both cameras use RGB8 at `1280x720x30`. Depth, gyro, and accelerometer are disabled by default.

Enable depth when needed:

```bash
ros2 launch fr3_lab_stack dual_realsense.launch.py enable_depth:=true
```

The cameras are not hardware synchronized. The launch keeps fixed serial-to-role binding but does not implement calibration, pairing, preprocessing, or policy inference.

## FR3 + MoveIt + RViz

Start the existing FR3 MoveIt stack:

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=192.170.10.101 \
  robot_type:=fr3 \
  use_fake_hardware:=false \
  load_gripper:=true
```

In the running RViz session, use **File → Open Config** and load:

```text
~/franka_ros2_ws/src/fr3_lab_stack/rviz/fr3_lab_stack.rviz
```

The project RViz configuration preserves the normal MoveIt visualization and adds:

- **Wrist RGB**
- **External RGB**

## Servo + SpaceMouse

Start Servo:

```bash
ros2 launch igd_fr3_control fr3_spacemouse_servocontrol.launch.py
```

Start the SpaceMouse publisher:

```bash
ros2 run igd_fr3_control spacemouse_twiststamped_publisher \
  --ros-args \
  -p topic:=/servo_node/delta_twist_cmds
```

See [`docs/commissioning.md`](docs/commissioning.md) for the integrated acceptance procedure.

## Learned-policy joint-target execution

Two robot-side target realizations are retained:

- **MoveIt + `fr3_arm_controller`** remains the conservative commissioning, reset, and single-target baseline. It executes targets reliably but is too slow for the DROID/π0.5 15 Hz control period: a real π0.5-derived action produced a **1.105 s** planned trajectory and **1.603 s** request-to-result time, compared with **66.7 ms** per DROID action.
- **Streaming hybrid impedance control** is the 15 Hz candidate. A persistent 1 kHz effort controller accepts absolute seven-joint equilibrium targets and allows the policy layer to replace them from fresh measured state without trajectory generation or stop-to-stop execution.

Commissioning tests accepted two fresh-state-anchored targets **66.807 ms** apart (0.141 ms later than the nominal 15 Hz period), with exact desired-target replacement. At one controlled J1 configuration, +5 mrad and +30 mrad static tests left approximately **2.1 mrad** steady equilibrium error. The state telemetry is currently 100 Hz; observed target ages of roughly 7--11 ms therefore do **not** constitute a precise callback-to-control-loop latency measurement. Higher-resolution receipt/application and controller-period timing instrumentation remains pending.

See [`docs/joint_target_execution.md`](docs/joint_target_execution.md) for the execution architecture and validation record.

## Validation status

Software build, launch, camera identity, optional depth, and RViz image-display checks have passed.

In the latest integrated observation on 2026-09-08, both RGB camera streams remained stable for **525.137 s** at approximately **29.98 Hz**, with no camera restart, disconnect, or new USB/xHCI error. The run was interrupted by a MoveIt Servo singularity safeguard before the required uninterrupted 15-minute acceptance period was completed.

After the monitored interval, the operator used MoveIt planning to move the arm to a nonsingular configuration and SpaceMouse teleoperation resumed. This supports interpretation of the event as expected singularity protection rather than a persistent Servo, controller, or communication failure.

The final uninterrupted **15-minute physical acceptance run remains pending**.

See [`docs/validation.md`](docs/validation.md) for the current validation record and evidence references.

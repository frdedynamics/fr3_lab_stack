# FR3 Lab Stack

ROS 2 bringup and validation utilities for the HVL FR3 lab setup. The package currently provides reproducible dual-RealSense RGB bringup and an RViz configuration that shows both camera streams alongside the existing Franka MoveIt visualization.

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

## Validation status

Software build, launch, camera identity, optional depth, and RViz image-display checks have passed.

In the latest integrated observation on 2026-09-08, both RGB camera streams remained stable for **525.137 s** at approximately **29.98 Hz**, with no camera restart, disconnect, or new USB/xHCI error. The run was interrupted by a MoveIt Servo singularity safeguard before the required uninterrupted 15-minute acceptance period was completed.

After the monitored interval, the operator used MoveIt planning to move the arm to a nonsingular configuration and SpaceMouse teleoperation resumed. This supports interpretation of the event as expected singularity protection rather than a persistent Servo, controller, or communication failure.

The final uninterrupted **15-minute physical acceptance run remains pending**.

See [`docs/validation.md`](docs/validation.md) for the current validation record and evidence references.

# Commissioning

Source ROS and the workspace overlay in every terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ros2_ws/install/setup.bash
```

Do not launch duplicate camera or robot stacks.

## 1. Camera identity and RGB baseline

Start both cameras:

```bash
ros2 launch fr3_lab_stack dual_realsense.launch.py
```

Verify nodes and fixed serial bindings:

```bash
ros2 node list | grep -E 'wrist_camera|external_camera'

for camera in wrist_camera external_camera; do
  for parameter in serial_no camera_name enable_color enable_depth enable_gyro enable_accel rgb_camera.color_profile rgb_camera.color_format; do
    ros2 param get /camera/$camera "$parameter"
  done
done
```

Expected configuration:

| Role | Serial | RGB |
| --- | --- | --- |
| Wrist | `342222073510` | RGB8, `1280x720x30` |
| External | `244222076317` | RGB8, `1280x720x30` |

Depth, gyro, and accelerometer must be disabled by default.

Check the low-bandwidth CameraInfo cadence:

```bash
timeout 15 ros2 topic hz /camera/wrist_camera/color/camera_info
timeout 15 ros2 topic hz /camera/external_camera/color/camera_info
```

Both should remain close to 30 Hz.

For a simultaneous RGB reception check:

```bash
ros2 run fr3_lab_stack check_rgb_rates.py --seconds 20
```

Large Python CLI subscriptions to `image_raw` can under-report on a loaded host, so CameraInfo cadence plus visible RGB continuity is the primary commissioning check.

## 2. Optional depth

Depth is not part of the initial SAPS/π0.5 RGB baseline.

To verify depth when needed, stop the default camera launch and restart with:

```bash
ros2 launch fr3_lab_stack dual_realsense.launch.py enable_depth:=true
```

Confirm both depth streams are enabled and producing data, then return to the default RGB-only launch for the integrated run.

## 3. Current USB topology

The candidate stable topology observed on 2026-09-08 is:

| Role | Serial | USB path | Controller |
| --- | --- | --- | --- |
| Wrist | `342222073510` | `6-1` | `0000:78:00.0` |
| External | `244222076317` | `6-2` | `0000:78:00.0` |

Bus/device numbers may change after re-enumeration. Camera serial numbers remain the authoritative identity.

If a camera stalls or disappears, capture evidence before changing configuration:

```bash
sudo journalctl -k --since "-30 min" | \
  grep -Ei 'usb|xhci|uvc|realsense|disconnect|reset|host controller|descriptor|error'

lsusb -t
rs-enumerate-devices -s
```

## 4. FR3 + MoveIt + RViz

Start the existing FR3 MoveIt stack:

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=192.170.10.101 \
  robot_type:=fr3 \
  use_fake_hardware:=false \
  load_gripper:=true
```

In the running RViz session, choose **File → Open Config** and load:

```text
~/franka_ros2_ws/src/fr3_lab_stack/rviz/fr3_lab_stack.rviz
```

Confirm both **Wrist RGB** and **External RGB** panes are visible and updating.

## 5. Servo + SpaceMouse

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

## 6. Physical integrated acceptance

Run the complete stack for **15 uninterrupted minutes** under operator supervision.

Verify:

- wrist and external RGB panes remain live throughout;
- translation works smoothly along ±X, ±Y, and ±Z;
- rotation works smoothly about ±Rx, ±Ry, and ±Rz;
- Button 1 opens and closes the gripper;
- Servo remains operational;
- no `communication_constraints_violation` occurs;
- no new USB/xHCI failure or camera disconnection occurs;
- no unexpected controller or camera process exit occurs.

Keep the robot in free space during the stability run to avoid contact reflexes as a confound.

### Singularity handling

`HALT_FOR_SINGULARITY` is a MoveIt Servo safeguard and is not by itself a commissioning failure.

If Cartesian teleoperation cannot move the arm away from the singular region:

1. release the SpaceMouse;
2. use MoveIt planning to move the arm to a known nonsingular configuration;
3. execute the plan;
4. confirm Servo returns to a normal state;
5. verify SpaceMouse teleoperation resumes.

For a clean stability qualification, restart the 15-minute observation window after such a recovery.

Do not change Servo singularity thresholds simply to make the acceptance test pass.

## 7. Software checks

```bash
cd ~/franka_ros2_ws

PYTHONNOUSERSITE=1 colcon build --packages-select fr3_lab_stack
source install/setup.bash

PYTHONNOUSERSITE=1 colcon test \
  --packages-select fr3_lab_stack \
  --event-handlers console_direct+

colcon test-result \
  --test-result-base build/fr3_lab_stack \
  --verbose
```

Also verify:

```bash
ros2 launch fr3_lab_stack dual_realsense.launch.py --show-args
```

The project package must not modify `franka_ros2` or `igd_fr3_control`.

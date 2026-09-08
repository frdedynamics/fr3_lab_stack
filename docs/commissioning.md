# Commissioning

Source `/opt/ros/jazzy/setup.bash` and `~/franka_ros2_ws/install/setup.bash` in
each terminal. Follow the build, camera, and MoveIt/RViz procedure in the README.
Do not launch duplicate camera or robot stacks.

## Camera identity, defaults, and rates

```bash
ros2 node list | grep -E 'wrist_camera|external_camera'
ros2 topic list | grep -E '/camera/(wrist_camera|external_camera)'
for camera in wrist_camera external_camera; do
  for parameter in serial_no camera_name enable_color enable_depth enable_gyro enable_accel rgb_camera.color_profile rgb_camera.color_format; do
    ros2 param get /camera/$camera "$parameter"
  done
done
timeout 15 ros2 topic hz /camera/wrist_camera/color/camera_info
timeout 15 ros2 topic hz /camera/external_camera/color/camera_info
timeout 15 ros2 topic hz /camera/wrist_camera/color/image_raw
timeout 15 ros2 topic hz /camera/external_camera/color/image_raw
ros2 topic echo /camera/wrist_camera/color/image_raw --once --field encoding
ros2 topic echo /camera/external_camera/color/image_raw --once --field encoding
ros2 topic list | grep -E '/camera/(wrist_camera|external_camera)/depth'
```

Expect RGB encoding `rgb8`, 1280×720 in CameraInfo/Image, and about 30 Hz for
each camera. `camera_info` is a low-bandwidth rate check; also check image rates
and visible updates. Large Python CLI image subscriptions can under-report on a
loaded host. Do not tune QoS to mask a problem. `timeout` exit 124 after a rate
measurement is expected.

For a simultaneous check with less Python deserialization overhead:

```bash
ros2 run fr3_lab_stack check_rgb_rates.py --seconds 20
```

It measures received image messages and monotonic arrival gaps, deserializing
only the first image per camera to check size/encoding. It uses ordinary default
ROS subscription QoS, changes no driver settings, and stores no images. Exit 0
means each camera produced at least two samples; inspect the printed rates
against the approximately 30 Hz requirement. Subscriber rates are not a
hardware synchronization measurement. With an active Conda environment, run
`/usr/bin/python3 ~/franka_ros2_ws/src/fr3_lab_stack/scripts/check_rgb_rates.py
--seconds 20` to select the ROS interpreter explicitly.

Confirm logs associate the requested serial with the correct node and report
the active Color profile. Serial parameters may retain the leading underscore.
Both `enable_depth`, `enable_gyro`, and `enable_accel` must be false by default.
Driver versions may advertise topics without producing samples: topic existence
alone does not demonstrate an enabled depth stream. If depth topics exist,
check the parameters, startup stream log, and use a bounded subscription:

```bash
timeout 5 ros2 topic echo /camera/wrist_camera/depth/image_rect_raw --once --field header
timeout 5 ros2 topic echo /camera/external_camera/depth/image_rect_raw --once --field header
```

For the opt-in test, stop the camera launch, restart with `enable_depth:=true`,
verify both enable parameters and incoming depth messages, then stop and restart
the default launch for the RGB-only integrated run. No alignment or point clouds
are enabled; depth profile remains the driver's device default.

## Physical integrated run: 15 minutes

An operator must supervise and physically operate the FR3. Start the existing
MoveIt command and load the project RViz config as described in the README.
Then start these in separate terminals, preserving the commissioned settings:

```bash
ros2 launch igd_fr3_control fr3_spacemouse_servocontrol.launch.py
```

```bash
ros2 run igd_fr3_control spacemouse_twiststamped_publisher \
  --ros-args \
  -p topic:=/servo_node/delta_twist_cmds
```

Record start/end time and check all of the following over at least 10–15 minutes:

- MoveIt robot visualization behaves normally.
- Wrist RGB and External RGB are simultaneously visible and update during motion.
- Smooth translation along X, Y, and Z independently.
- Smooth rotation about X, Y, and Z independently.
- Button 1 opens/closes the gripper and teleoperation continues afterward.
- Servo remains active; no new ros2_control communication dropout, controller
  process exit, or robot communication exception occurs.

Record any warnings with timestamps and the affected process. A successful
camera-only run is not evidence that the integrated motion test passed.
Existing SpaceMouse commissioning guidance remains in the dependency's
`docs/fr3_spacemouse_commissioning.md`. No Servo or publisher is auto-started here.

## Software checks and repository isolation

```bash
cd ~/franka_ros2_ws
PYTHONNOUSERSITE=1 colcon build --packages-select fr3_lab_stack
source install/setup.bash
PYTHONNOUSERSITE=1 colcon test --packages-select fr3_lab_stack --event-handlers console_direct+
colcon test-result --test-result-base build/fr3_lab_stack --verbose
ros2 launch fr3_lab_stack dual_realsense.launch.py --show-args

cd ~/franka_ros2_ws/src/fr3_lab_stack
git status --short
git log --oneline --decorate -5
cd ~/franka_ros2_ws/src/igd_fr3_control
git status --short
cd ~/franka_ros2_ws/src
git rev-parse --show-toplevel
git status --short
```

The Franka Git root on this workstation is `~/franka_ros2_ws/src`.
The new nested independent repository appears as `?? fr3_lab_stack/` in that
parent's status. Do not stage it into Franka or edit the parent's ignore files.
Compare existing changes against the baseline in `validation.md`.

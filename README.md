# FR3 lab stack

Tested target: Ubuntu 24.04 / ROS 2 Jazzy, `realsense2_camera` 4.57.7,
librealsense 2.57.7. See [commissioning](docs/commissioning.md) for validation
commands and [validation results](docs/validation.md) for what was actually tested.

Current validation: build and launch checks passed, both RGB panes were confirmed
visible, and optional depth produced images. The integrated motion test stopped
early after operator-observed object contact triggered `cartesian_reflex`; the
wrist USB host controller also failed. A full 15-minute acceptance run remains
outstanding. Details and captured logs are in the validation record.

## Build and cameras

```bash
cd ~/franka_ros2_ws
source /opt/ros/jazzy/setup.bash
PYTHONNOUSERSITE=1 colcon build --packages-select fr3_lab_stack
source install/setup.bash
ros2 launch fr3_lab_stack dual_realsense.launch.py
```

Stop any existing processes using either camera before launching. To enable
depth on both devices, restart with:

```bash
ros2 launch fr3_lab_stack dual_realsense.launch.py enable_depth:=true
```

If CMake selects Miniconda Python and reports missing `catkin_pkg` or `pytest`,
select the installed ROS system interpreter without installing packages:

```bash
PYTHONNOUSERSITE=1 colcon build --packages-select fr3_lab_stack \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

This choice is cached for subsequent plain builds.

| Role / camera name | Serial | RGB topic |
| --- | --- | --- |
| `wrist_camera` | `342222073510` | `/camera/wrist_camera/color/image_raw` |
| `external_camera` | `244222076317` | `/camera/external_camera/color/image_raw` |

Both use namespace `/camera`, RGB8, `1280x720x30`, color enabled, depth disabled
by default, gyro and accelerometer disabled. All other settings inherit the
installed driver's defaults, including QoS, timestamps, and synchronization.
No firmware or package upgrade is part of bringup. The underscore preceding
serials in the launch prevents YAML integer conversion; the driver strips it.
Separate launch scopes prevent camera arguments leaking between devices.

The cameras are **not hardware synchronized**. Previously measured online RGB
offsets supplied by the operator were mean 18.94 ms, median 18.97 ms,
p95 20.34 ms, maximum 20.63 ms; these are historical measurements, not guarantees
or measurements made by this package. RGB supports the initial SAPS/π0.5
baseline; inference, calibration, image pairing, and preprocessing are outside
this package.

## One MoveIt RViz session with both images

Start the existing physical robot stack in a separate sourced terminal:

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=192.170.10.101 \
  robot_type:=fr3 \
  use_fake_hardware:=false \
  load_gripper:=true
```

In that RViz window choose **File → Open Config** and load:

```text
~/franka_ros2_ws/src/fr3_lab_stack/rviz/fr3_research.rviz
```

If the dialog does not expand `~`, paste the full path. The installed copy is
also available at the path printed by:

```bash
ros2 pkg prefix --share fr3_lab_stack
```

Append `/rviz/fr3_research.rviz` to that path. Enable both image panes under
**Panels** if needed and dock **Wrist RGB** and **External RGB** beside the
robot view so both remain visible (not tabs behind one another). The old Qt
dock-state blob was reset because it referenced two identically named `Image`
panes. Save layout adjustments only to this project's source RViz file.

The installed Franka launch hardcodes its RViz file and offers neither a config
argument nor a switch to suppress RViz. Loading the project file in the same
process preserves the robot description, semantic description, planning pipeline,
and kinematics parameters supplied at startup. There is deliberately no
`fr3_full_stack.launch.py`: separate launches plus File → Open Config avoid
duplicating or depending on private internals of the Franka launch.

Inspection also found `robot_type` and `load_gripper` are not declared or read by
this particular MoveIt launch: it always selects FR3 and includes the gripper.
The command above preserves the established operator procedure.

The RViz baseline is the **locally used, already modified** Franka config at
checkout `1369a2c`; its source hash and attribution are in [NOTICE](NOTICE).
All non-Image displays, MoveIt settings, tools, fixed frame, and views are
preserved. Legacy Marker/PointCloud/TF displays may report missing data from
older experiments. RGB Image displays do not need extrinsic calibration or a
transform from the cameras to the robot. No camera-to-robot transform is invented.

Start Servo and SpaceMouse separately after robot bringup; commands and the
operator test checklist are in [commissioning](docs/commissioning.md).

## Repository hosting

This is an independent local Git repository. Local author identity is
`ehsann90 <ehsan.kh69@gmail.com>`, matching the existing lab commits.
The workstation filesystem owner is the local OS user, not a GitHub organization.
GitHub organization ownership and access require a hosted repository. No remote
is created or pushed automatically. After confirming destination and visibility:

```bash
cd ~/franka_ros2_ws/src/fr3_lab_stack
gh repo create frdedynamics/fr3_lab_stack --private --source=. --remote=origin
git push -u origin main
```

If the lab repository already exists, use `git remote add origin
git@github.com:frdedynamics/fr3_lab_stack.git` instead of `gh repo create`, then
inspect its history before pushing. Creating the repository requires permission
in `frdedynamics`; commit attribution alone does not grant collaborator access.

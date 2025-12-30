# MuJoCo RM Workspace

This workspace bootstraps a walker-style MuJoCo environment (ground plane, lighting, and skybox) so you can focus on iterating over each limb/part without worrying about the scene boilerplate.

## Layout

- `models/walker/walker.xml` – entry-point MJCF that loads only the background (skybox + plane) and exposes a `builder_root` body/site so you can start attaching your own parts.
- `models/walker/include/` – modular snippets that you can edit or swap in new parts without touching the main file:
  - `defaults.xml` – shared defaults/classes for geoms, joints, and actuators.
  - `assets.xml` – textures/materials; drop custom PNGs into `models/walker/assets/` and reference them here.
  - `arena.xml` – floor plane, lighting, and spawn helpers.
  - `base_link.xml` – imports the 13-piece chassis mesh stack, nests the wheel/sensor/extension modules as children, and anchors everything under `builder_root`.
  - `front_left_wheel.xml`, `front_right_wheel.xml`, `rear_left_wheel.xml`, `rear_right_wheel.xml` – wheel modules (included by `base_link.xml`) that load each pair of wheel meshes, attach a hinge joint, and use the URDF poses/orientations.
  - `front_sensors.xml`, `rear_sensors.xml`, `left_sensors.xml`, `right_sensors.xml` – hit-sensor + LED assemblies positioned via the URDF offsets so you can toggle them as a group.
  - `accelerometer.xml` – base-mounted IMU housing positioned per URDF; this module includes `gyrosensor.xml` so the IMU stack moves as a unit.
  - `gyrosensor.xml` – gyro PCB/cover that piggybacks on the accelerometer mount (auto-included; edit when you need to offset or add sensors).
  - `camera_link.xml` – arm-tip camera body + live MuJoCo `<camera>` so you can render RGB feeds that follow the manipulator (included under `arm_2_link.xml`).
  - `extension.xml` – rear extension plate meshes mounted under the base per the URDF.
  - `controller.xml` – loads the three-part intelligent controller cover at the URDF pose.
  - `arm_base_link.xml` – mounts the arm base housing so you can attach the manipulator joints in later steps.
  - `arm_1_link.xml` – first arm segment with hinge joint, visual meshes, and collision shell.
  - `arm_2_link.xml` – second arm segment hanging from `arm_1_link`.
  - `triangle_link.xml` – intermediate bracket between `arm_1` and the rod/gripper subassembly (now pulls in `rod_1_link.xml` so the tip chain follows this branch).
  - `endpoint_bracket_link.xml` – bracket assembly mounted at the end of `arm_2`.
  - `rod_1_link.xml` – first rod off the bracket, preceding the force sensor/gripper.
  - `gripper_link.xml` – gripper housing (force-sensor mount) attached to the bracket.
  - `left_gripper.xml`, `right_gripper.xml` – seven-link finger chains (meshes + locked hinges) that hang from the gripper slider so you can later actuate each jaw independently.
  - `rod_2_link.xml` – auxiliary rod attached to the arm base (matches the URDF `rod_2` linkage).
  - `rod_link.xml` – secondary rod driven by `servo_motor_0`, including its own hinge/meshes.
  - `rod_3_link.xml` – third rod segment attached to `rod_link` with its revolute joint.
  - `equalities.xml` – weld constraints that act like rigid 보강재, tying `rod_2_link`·`rod_link` to `arm_2_link` so 보조 로드가 팔 말단과 함께 움직입니다.
  - `torso.xml`, `leg_left.xml`, `leg_right.xml` – example body definitions (update or add new `<body>` trees per module as you build up the robot).
  - `actuators.xml` – default motors for hips/knees/ankles. Include this file once those joints exist in your model.
  - `sensors.xml` – example joint/world sensors; extend to match your control stack.

## Usage

1. Launch `mjviewer models/walker/walker.xml` (or open the file in MuJoCo GUI). You should see only the checker plane, horizon, and a `build_anchor` site hovering at the origin height.
2. Start building parts under the `builder_root` body. Copy snippets from `include/torso.xml`, `include/leg_*.xml`, `include/*wheel*.xml`, etc., or create new include files and reference them from `walker.xml`.
3. Keep assets such as textures/meshes in `models/walker/assets/` and reference them via `<asset>` definitions in `assets.xml`.
4. When joints/actuators are ready, add `<actuator><include file="include/actuators.xml"/></actuator>` (and similarly for sensors) back into `walker.xml` so the simulation exposes the control hooks you need.

### Tips

- Keep the `builder_root` body centered over the origin—adjust `body pos` offsets instead of translating the entire model.
- Add `site` elements where you plan to attach new limbs or sensors; they make it easier to align connections.
- Use the `spawn` site in `arena.xml` to visualize the nominal COM height when tuning proportions.
- When importing meshes (like the wheels), convert Collada files to `.obj` (e.g. `assimp export part.dae part.obj`), drop them into `models/walker/assets/`, add a `<mesh>` entry in `include/assets.xml`, and create a companion include file (for transforms/joints) so you can toggle modules on/off via `<include>` statements.
- When importing meshes, remember to scale/align them in a DCC tool so that only simple `<geom>` transforms are needed inside the include files.

## ROS 2 bringup (MuJoCo simulator + teleop)

All ROS 2 code lives in `ros2/` as a single package `mujoco_rm_bringup`. It wraps the same MuJoCo model and provides joystick teleop, camera streaming, and arm/gripper control.

### Key files

- `ros2/src/mujoco_rm_bringup/launch/teleop_linux.launch.xml` — main launch for MuJoCo sim + joystick teleop + camera publisher. Namespaced (default `mujoco_rm`). Toggle `start_joy_node`, `joy_topic`, `cmd_vel_topic`, `cmd_joint_states_topic`, `cmd_gripper_topic`, etc. Deadman and axis/button mappings are exposed as params (see file defaults).
- `ros2/src/mujoco_rm_bringup/mujoco_rm_bringup/mujoco_sim_node.py` — MuJoCo simulator node. Publishes camera (`arm_camera/image_raw`), applies `/cmd_vel_joy`, `/cmd_joint_states`, optional `/cmd_gripper` and RoboMaster-style `/cmd_arm(_vel)`. Includes arm/gripper telemetry logs (`arm_telemetry_hz`, `gripper_telemetry_hz`).
- `ros2/src/mujoco_rm_bringup/mujoco_rm_bringup/joy_base_teleop_node.py` — joystick → `/cmd_vel_joy` (vx/vy/wz) with deadman and scaling similar to `teleop_twist_joy` but without spurious zeros.
- `ros2/src/mujoco_rm_bringup/mujoco_rm_bringup/joy_servo_teleop_node.py` — joystick → `/cmd_joint_states` for arm (servo0/servo1) and latched gripper open/close actions (no mid-stop). Buttons are set in the launch file; deadman required.
- `ros2/src/mujoco_rm_bringup/mujoco_rm_bringup/servo_config.py` + `models/walker/include/actuators.xml` — MuJoCo actuator ranges/limits; the ROS teleop nodes query joint ranges directly to map joystick to ctrl.

### Running

```bash
cd /mujoco_rm_ws/ros2
colcon build --symlink-install
source install/setup.bash
ros2 launch mujoco_rm_bringup teleop_linux.launch.xml
```

Default topics (namespaced with `ns:=mujoco_rm`):
- Base command: `/mujoco_rm/cmd_vel_joy`
- Arm/servos: `/mujoco_rm/cmd_joint_states` (servo_motor_0_act, servo_motor_1_act, gripper_pos)
- Camera: `/mujoco_rm/arm_camera/image_raw` (`camera_fps`, `camera_width/height` tunable)

Gripper actions (latched, action-like):
- `LB + A` → OPEN to the upper limit (default joint range `-0.08 .. 0.50`)
- `LB + X` → CLOSE to the lower limit
If already at the requested end-stop, the node logs `[gripper_action] already OPEN/CLOSED` and ignores the new command.

### Model notes (arm/gripper)

- Arm joint limits are defined in `models/walker/include/arm_1_link.xml` and `arm_2_link.xml`.
- Servo actuators and gains are in `models/walker/include/actuators.xml`.
- Gripper joint/ctrl limits currently: `gripper_hinge_left range/ctrlrange = [-0.08, 0.5]` (open near -0.08, close near +0.5). Adjust here if you need different travel.
- Gripper contact uses the actual finger meshes (collision ON) so visual and physical extents match. The ball uses `soft_ball_geom` in `walker_soft_ball.xml` with matching friction/margins.

### Camera

- Physical camera is defined in `models/walker/include/camera_link.xml` on the arm tip. The sim node renders it via `mujoco.Renderer` and publishes RGB + CameraInfo. Resolution/FPS are set via launch args.

## How to extend

- Add new MuJoCo modules under `models/walker/include/` and include them from `walker_soft_ball.xml` or your own top-level MJCF.
- To add new ROS control topics, copy the patterns in `mujoco_sim_node.py` (declare params, create subscriptions, clamp to `actuator_ctrlrange`).
- For different joysticks, override button/axis params in `teleop_linux.launch.xml` rather than editing code. Use `arm_joy_debug_hz` or `base_debug_print_hz` to print mappings live.

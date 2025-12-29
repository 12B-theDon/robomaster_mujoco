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

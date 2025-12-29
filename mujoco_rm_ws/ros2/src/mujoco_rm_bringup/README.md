# mujoco_rm_bringup

ROS 2 bringup for the MuJoCo walker model:
- Joystick teleop (`joy` + `teleop_twist_joy`) publishing `/cmd_vel`
- MuJoCo simulation node subscribing `/cmd_vel` and publishing camera images

## Build

From the workspace root:

```bash
cd /mujoco_rm_ws/ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run (joystick + sim + camera)

```bash
ros2 launch mujoco_rm_bringup teleop_linux.launch.xml
```

Parameters:
- `model_path` (default: `/mujoco_rm_ws/models/walker/walker_soft_ball.xml`)
- `camera_name` (default: `arm_camera`)
- `camera_width` / `camera_height` (default: `640` / `480`)
- `camera_fps` (default: `15`)

Example:

```bash
ros2 launch mujoco_rm_bringup joy_mujoco_cam.launch.py camera_fps:=30
```

## View the camera

```bash
ros2 run rqt_image_view rqt_image_view
```

Select topic:
- `/arm_camera/image_raw`

## Notes

- Teleop mapping is in `mujoco_rm_bringup/config/teleop_twist_joy.yaml`. Adjust axes/buttons to your controller.
- Arm/gripper teleop uses position targets published on `/cmd_joint_states` (not raw speed commands), similar to RoboMaster MoveServo semantics.

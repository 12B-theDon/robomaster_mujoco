"""Launches a MuJoCo viewer with the background/arena template."""
from __future__ import annotations

import time
import os
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SOFT_BALL_MODEL_PATH = ROOT / "models" / "walker" / "walker_soft_ball.xml"
DEFAULT_MODEL_PATH = SOFT_BALL_MODEL_PATH if SOFT_BALL_MODEL_PATH.exists() else (ROOT / "models" / "walker" / "walker.xml")

BASE_CMD_EPS = 1e-3
MECANUM_SMOOTH_TAU_S = 0.05


def main() -> None:
    if not DEFAULT_MODEL_PATH.exists():
        raise SystemExit(f"Model file not found: {DEFAULT_MODEL_PATH}")

    model = mujoco.MjModel.from_xml_path(str(DEFAULT_MODEL_PATH))
    data = mujoco.MjData(model)

    cam_name = os.getenv("CAM_NAME", "arm_camera")
    dump_cam_pose = os.getenv("DUMP_CAM_POSE", "0") == "1"
    dump_cam_every = int(os.getenv("DUMP_CAM_EVERY", "30"))
    dump_cam_img = os.getenv("DUMP_CAM_IMG", "0") == "1"
    cam_img_w = int(os.getenv("CAM_IMG_W", "640"))
    cam_img_h = int(os.getenv("CAM_IMG_H", "480"))
    cam_img_every = int(os.getenv("CAM_IMG_EVERY", "30"))
    cam_img_dir = Path(os.getenv("CAM_IMG_DIR", "/tmp/mujoco_cam_frames"))
    cam_img_format = os.getenv("CAM_IMG_FORMAT", "ppm").lower()
    list_cameras = os.getenv("LIST_CAMERAS", "0") == "1"
    dump_cam_latest = os.getenv("DUMP_CAM_LATEST", "0") == "1"
    cam_latest_path = Path(os.getenv("CAM_LATEST_PATH", "/tmp/arm_camera_latest.ppm"))
    cam_latest_every = int(os.getenv("CAM_LATEST_EVERY", "10"))

    def _safe_id(obj_type: mujoco.mjtObj, name: str) -> int | None:
        try:
            return int(mujoco.mj_name2id(model, obj_type, name))
        except ValueError:
            return None

    cam_id = _safe_id(mujoco.mjtObj.mjOBJ_CAMERA, cam_name)
    if dump_cam_pose and cam_id is None:
        raise SystemExit(f"Camera not found: {cam_name}")
    if dump_cam_img and cam_id is None:
        raise SystemExit(f"Camera not found: {cam_name}")
    if dump_cam_latest and cam_id is None:
        raise SystemExit(f"Camera not found: {cam_name}")

    view_cam_name = os.getenv("VIEW_CAMERA", "")
    view_cam_id = None
    if view_cam_name:
        view_cam_id = _safe_id(mujoco.mjtObj.mjOBJ_CAMERA, view_cam_name)
        if view_cam_id is None:
            raise SystemExit(f"Camera not found: {view_cam_name}")
        print(f"Viewer fixed camera: {view_cam_name} (id={view_cam_id})")

    renderer: mujoco.Renderer | None = None
    if dump_cam_img or dump_cam_latest:
        try:
            if dump_cam_img:
                cam_img_dir.mkdir(parents=True, exist_ok=True)
            cam_latest_path.parent.mkdir(parents=True, exist_ok=True)
            renderer = mujoco.Renderer(model, height=cam_img_h, width=cam_img_w)
        except Exception as exc:  # noqa: BLE001
            print(f"Failed to create offscreen renderer; disabling camera image output: {exc}")
            renderer = None
            dump_cam_img = False
            dump_cam_latest = False

    def _write_ppm(path: Path, rgb: np.ndarray) -> None:
        if rgb.dtype != np.uint8:
            rgb = rgb.astype(np.uint8, copy=False)
        h, w, c = rgb.shape
        if c != 3:
            raise ValueError(f"Expected RGB image with 3 channels, got shape {rgb.shape}")
        header = f"P6\n{w} {h}\n255\n".encode("ascii")
        with path.open("wb") as f:
            f.write(header)
            f.write(rgb.tobytes())

    def _write_ppm_atomic(path: Path, rgb: np.ndarray) -> None:
        tmp = path.with_suffix(path.suffix + ".tmp")
        _write_ppm(tmp, rgb)
        tmp.replace(path)

    servo0_act_id = _safe_id(mujoco.mjtObj.mjOBJ_ACTUATOR, "servo_motor_0_act")
    servo1_act_id = _safe_id(mujoco.mjtObj.mjOBJ_ACTUATOR, "servo_motor_1_act")
    servo0_joint_id = _safe_id(mujoco.mjtObj.mjOBJ_JOINT, "servo_motor_0")
    servo1_joint_id = _safe_id(mujoco.mjtObj.mjOBJ_JOINT, "servo_motor_1")
    servo0_qpos_adr = int(model.jnt_qposadr[servo0_joint_id]) if servo0_joint_id is not None else None
    servo1_qpos_adr = int(model.jnt_qposadr[servo1_joint_id]) if servo1_joint_id is not None else None

    # Equality couplers for the auxiliary linkage (used to keep the mechanism from dragging
    # the "held" assembly through the closed kinematic loop).
    eq_triangle_rod2_a = _safe_id(mujoco.mjtObj.mjOBJ_EQUALITY, "triangle_rod2_pin_a")
    eq_triangle_rod2_b = _safe_id(mujoco.mjtObj.mjOBJ_EQUALITY, "triangle_rod2_pin_b")
    eq_triangle_rod1_a = _safe_id(mujoco.mjtObj.mjOBJ_EQUALITY, "triangle_rod1_pin_a")
    eq_triangle_rod1_b = _safe_id(mujoco.mjtObj.mjOBJ_EQUALITY, "triangle_rod1_pin_b")
    prev_servo0_ctrl = float("nan")
    prev_servo1_ctrl = float("nan")
    lock_servo0: float | None = None
    lock_servo1: float | None = None
    active_servo: int | None = None
    last_active_servo: int | None = None
    idle_steps = 0
    CMD_EPS = 1e-4
    RELEASE_IDLE_STEPS = 25
    SERVO_QVEL_EPS = 5e-3

    servo0_dof_adr = int(model.jnt_dofadr[servo0_joint_id]) if servo0_joint_id is not None else None
    servo1_dof_adr = int(model.jnt_dofadr[servo1_joint_id]) if servo1_joint_id is not None else None
    # Keep position-controlled servos at the model's nominal pose on startup.
    for actuator_name, joint_name in (
        ("servo_motor_0_act", "servo_motor_0"),
        ("servo_motor_1_act", "servo_motor_1"),
    ):
        try:
            actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        except ValueError:
            continue
        qpos_adr = int(model.jnt_qposadr[joint_id])
        data.ctrl[actuator_id] = float(model.qpos0[qpos_adr])

    # Start with gripper closed (open=+0.05, close=-1.0).
    try:
        gripper_act = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "gripper_pos")
        data.ctrl[gripper_act] = -1.0
    except ValueError:
        pass

    # --- Base drive setup (mecanum) ---
    base_body_id = _safe_id(mujoco.mjtObj.mjOBJ_BODY, "base_link")
    root_body_id = _safe_id(mujoco.mjtObj.mjOBJ_BODY, "builder_root")

    wheel_radius_m = 0.05
    wheel_geom_id = _safe_id(mujoco.mjtObj.mjOBJ_GEOM, "wheel_front_left_collision")
    if wheel_geom_id is not None:
        # Only trust geom_size for primitives; for meshes it is not a wheel radius.
        if int(model.geom_type[wheel_geom_id]) == int(mujoco.mjtGeom.mjGEOM_CYLINDER):
            wheel_radius_m = float(model.geom_size[wheel_geom_id][0])

    # Derive wheelbase geometry from the wheel mount positions (in base_link frame).
    lx = 0.10
    ly = 0.10
    wheel_body_names = (
        "wheel_front_left",
        "wheel_front_right",
        "wheel_rear_left",
        "wheel_rear_right",
    )
    wheel_positions = []
    for wheel_name in wheel_body_names:
        wheel_id = _safe_id(mujoco.mjtObj.mjOBJ_BODY, wheel_name)
        if wheel_id is None:
            continue
        wheel_positions.append(model.body_pos[wheel_id])
    if wheel_positions:
        lx = float(max(abs(p[0]) for p in wheel_positions))
        ly = float(max(abs(p[1]) for p in wheel_positions))
    k_geom = lx + ly

    # Torque limit from wheel actuators (per wheel).
    wheel_tau_max = 0.0
    wheel_act_id = _safe_id(mujoco.mjtObj.mjOBJ_ACTUATOR, "wheel_front_left_act")
    if wheel_act_id is not None:
        lo, hi = model.actuator_forcerange[wheel_act_id]
        wheel_tau_max = float(min(abs(lo), abs(hi)))
    if wheel_tau_max <= 0:
        wheel_tau_max = 0.25  # fallback

    # Speed limits from command actuators.
    def _act_ctrlrange(name: str, default: tuple[float, float]) -> tuple[float, float]:
        act_id = _safe_id(mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if act_id is None:
            return default
        lo, hi = model.actuator_ctrlrange[act_id]
        return float(lo), float(hi)

    vx_lo, vx_hi = _act_ctrlrange("base_vx", (-2.5, 3.5))
    vy_lo, vy_hi = _act_ctrlrange("base_vy", (-2.8, 2.8))
    wz_lo, wz_hi = _act_ctrlrange("base_wz", (-10.472, 10.472))
    v_max = max(abs(vx_lo), abs(vx_hi), abs(vy_lo), abs(vy_hi))
    wz_max = max(abs(wz_lo), abs(wz_hi))

    # Compute effective inertia about yaw once at nominal pose.
    Izz = None
    free_qpos_adr = None
    free_dof_wz = None
    free_joint_id = _safe_id(mujoco.mjtObj.mjOBJ_JOINT, "builder_free")
    if free_joint_id is not None:
        mujoco.mj_forward(model, data)
        free_qpos_adr = int(model.jnt_qposadr[free_joint_id])
        dof0 = int(model.jnt_dofadr[free_joint_id])
        dof_wz = dof0 + 5
        free_dof_wz = dof_wz
        fullM = np.zeros((model.nv, model.nv), dtype=np.float64)
        mujoco.mj_fullM(model, fullM, data.qM)
        Izz = float(fullM[dof_wz, dof_wz])

    subtree_mass = 0.0
    if root_body_id is not None:
        subtree_mass = float(model.body_subtreemass[root_body_id])
    if subtree_mass <= 0:
        subtree_mass = float(model.body_mass.sum())

    # Max planar force and yaw torque implied by wheel torque limits (ideal mecanum wrench bounds).
    F_xy_max = 4.0 * wheel_tau_max / max(wheel_radius_m, 1e-6)
    Tz_max = F_xy_max * k_geom

    # Max accel implied by limits (ideal planar wrench bounds).
    a_xy_max = F_xy_max / max(subtree_mass, 1e-6)
    alpha_z_max = None
    if Izz is not None:
        alpha_z_max = Tz_max / max(Izz, 1e-6)

    # Base control: map (vx, vy, wz) command actuators -> wheel velocity actuators.
    try:
        data.ctrl[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "base_vx")] = 0.0
        data.ctrl[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "base_vy")] = 0.0
        data.ctrl[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "base_wz")] = 0.0
    except ValueError:
        pass

    omega_fl_filt = 0.0
    omega_fr_filt = 0.0
    omega_rl_filt = 0.0
    omega_rr_filt = 0.0
    yaw_hold: float | None = None

    def _quat_to_yaw(qw: float, qx: float, qy: float, qz: float) -> float:
        # ZYX yaw from quaternion (w, x, y, z).
        return float(np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)))

    def _wrap_pi(angle: float) -> float:
        return float((angle + np.pi) % (2.0 * np.pi) - np.pi)

    def control_callback(m: mujoco.MjModel, d: mujoco.MjData) -> None:
        nonlocal omega_fl_filt, omega_fr_filt, omega_rl_filt, omega_rr_filt, yaw_hold
        if base_body_id is not None:
            # Clear any previous applied forces on the base; we only use this for the mecanum drive assist.
            d.xfrc_applied[base_body_id][:] = 0.0
        if root_body_id is not None:
            d.xfrc_applied[root_body_id][:] = 0.0
        try:
            vx_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "base_vx")
            vy_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "base_vy")
            wz_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "base_wz")
            fl_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "wheel_front_left_act")
            fr_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "wheel_front_right_act")
            rl_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "wheel_rear_left_act")
            rr_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "wheel_rear_right_act")
        except ValueError:
            return

        vx = float(d.ctrl[vx_id])
        vy = float(d.ctrl[vy_id])
        wz = float(d.ctrl[wz_id])
        if abs(vx) < BASE_CMD_EPS and abs(vy) < BASE_CMD_EPS and abs(wz) < BASE_CMD_EPS:
            omega_fl_filt = 0.0
            omega_fr_filt = 0.0
            omega_rl_filt = 0.0
            omega_rr_filt = 0.0
        else:
            # Omnidirectional (mecanum-like) mapping.
            # Wheel layout: FL, FR, RL, RR.
            # Even though the wheel contact geometry is simplified (cylinders), we still
            # spin the wheels with the standard mecanum kinematics so the visual motion
            # matches the commanded (vx, vy, wz). The planar wrench assist below (plus
            # yaw hold when wz==0) prevents the non-holonomic contact from turning this
            # into unintended yaw.
            omega_fl = (vx - vy - k_geom * wz) / wheel_radius_m
            omega_fr = (vx + vy + k_geom * wz) / wheel_radius_m
            omega_rl = (vx + vy - k_geom * wz) / wheel_radius_m
            omega_rr = (vx - vy + k_geom * wz) / wheel_radius_m

            dt = float(m.opt.timestep)
            tau = MECANUM_SMOOTH_TAU_S
            alpha = dt / (tau + dt)
            omega_fl_filt += alpha * (omega_fl - omega_fl_filt)
            omega_fr_filt += alpha * (omega_fr - omega_fr_filt)
            omega_rl_filt += alpha * (omega_rl - omega_rl_filt)
            omega_rr_filt += alpha * (omega_rr - omega_rr_filt)

        d.ctrl[fl_id] = omega_fl_filt
        d.ctrl[fr_id] = omega_fr_filt
        d.ctrl[rl_id] = omega_rl_filt
        d.ctrl[rr_id] = omega_rr_filt

        # Drive assist: apply an ideal planar wrench consistent with wheel torque limits.
        # This provides true holonomic (vx, vy, wz) motion even if the wheel contact geometry
        # is simplified (e.g., cylindrical collision geoms).
        if root_body_id is not None and base_body_id is not None:
            # Base axes in world frame (columns of xmat).
            xmat = d.xmat[base_body_id].reshape(3, 3)
            x_axis = xmat[:, 0]
            y_axis = xmat[:, 1]
            z_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)

            v_des_world = vx * x_axis + vy * y_axis
            w_des_world = wz * z_axis

            vel6 = np.zeros(6, dtype=np.float64)
            mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, base_body_id, vel6, 0)
            w_world = vel6[0:3]
            v_world = vel6[3:6]

            # Use the available acceleration (up to limits) for responsiveness without hard clamps.
            v_mag = float(max(abs(vx), abs(vy)))
            tau_xy_eff = max(v_mag / max(a_xy_max, 1e-6), 0.05)
            a_world = (v_des_world - v_world) / max(tau_xy_eff, 1e-6)
            F_world = subtree_mass * a_world
            F_xy = F_world.copy()
            F_xy[2] = 0.0
            F_xy_norm = float(np.linalg.norm(F_xy))
            if F_xy_norm > F_xy_max:
                F_xy *= F_xy_max / F_xy_norm

            if alpha_z_max is not None and Izz is not None:
                # If the user isn't commanding yaw, hold the current heading. This prevents
                # vy from turning the robot due to non-holonomic wheel contact.
                if abs(wz) < BASE_CMD_EPS and free_qpos_adr is not None and free_dof_wz is not None:
                    if yaw_hold is None and (abs(vx) >= BASE_CMD_EPS or abs(vy) >= BASE_CMD_EPS):
                        qw, qx, qy, qz = map(float, d.qpos[free_qpos_adr + 3 : free_qpos_adr + 7])
                        yaw_hold = _quat_to_yaw(qw, qx, qy, qz)
                    elif abs(vx) < BASE_CMD_EPS and abs(vy) < BASE_CMD_EPS:
                        yaw_hold = None

                    if yaw_hold is not None:
                        qw, qx, qy, qz = map(float, d.qpos[free_qpos_adr + 3 : free_qpos_adr + 7])
                        yaw = _quat_to_yaw(qw, qx, qy, qz)
                        yaw_err = _wrap_pi(yaw - yaw_hold)
                        omega_z = float(d.qvel[free_dof_wz])

                        yaw_err_sat = float(np.deg2rad(10.0))
                        kp = Tz_max / max(yaw_err_sat, 1e-6)  # torque per rad
                        kd = 2.0 * np.sqrt(max(kp * Izz, 1e-12))  # critical damping
                        T_world = np.array([0.0, 0.0, -kp * yaw_err - kd * omega_z], dtype=np.float64)
                    else:
                        T_world = np.array([0.0, 0.0, 0.0], dtype=np.float64)
                else:
                    yaw_hold = None
                    tau_wz_eff = max(abs(wz) / max(alpha_z_max, 1e-6), 0.05)
                    alpha_world = (w_des_world - w_world) / max(tau_wz_eff, 1e-6)
                    T_world = np.array([0.0, 0.0, Izz * float(alpha_world[2])], dtype=np.float64)
            else:
                # Fallback: proportional torque on yaw rate error.
                T_world = np.array([0.0, 0.0, Tz_max * (wz - float(w_world[2])) / max(wz_max, 1e-6)], dtype=np.float64)

            T_world[2] = float(np.clip(T_world[2], -Tz_max, Tz_max))

            # Apply at the root body (the one that carries the free joint).
            d.xfrc_applied[root_body_id][0:3] = F_xy
            d.xfrc_applied[root_body_id][3:6] = T_world

        # Mutual exclusion for arm servos:
        # - If the user moves servo0, servo1 holds its current position (even if its slider is at a limit).
        # - If the user moves servo1, servo0 holds its current position.
        nonlocal prev_servo0_ctrl, prev_servo1_ctrl, lock_servo0, lock_servo1, active_servo, last_active_servo, idle_steps
        if (
            servo0_act_id is None
            or servo1_act_id is None
            or servo0_qpos_adr is None
            or servo1_qpos_adr is None
            or servo0_dof_adr is None
            or servo1_dof_adr is None
        ):
            return

        # Read the user-set controls first (viewer/UI writes into d.ctrl).
        user_servo0_ctrl = float(d.ctrl[servo0_act_id])
        user_servo1_ctrl = float(d.ctrl[servo1_act_id])
        if prev_servo0_ctrl != prev_servo0_ctrl:  # NaN check (first step)
            prev_servo0_ctrl = user_servo0_ctrl
        if prev_servo1_ctrl != prev_servo1_ctrl:  # NaN check (first step)
            prev_servo1_ctrl = user_servo1_ctrl

        d0 = abs(user_servo0_ctrl - prev_servo0_ctrl)
        d1 = abs(user_servo1_ctrl - prev_servo1_ctrl)
        prev_servo0_ctrl = user_servo0_ctrl
        prev_servo1_ctrl = user_servo1_ctrl

        user_change0 = d0 > CMD_EPS
        user_change1 = d1 > CMD_EPS

        if user_change0 or user_change1:
            idle_steps = 0
            if user_change0 and not user_change1:
                if active_servo != 0:
                    lock_servo1 = None  # recapture hold point once, when switching to servo0
                active_servo = 0
                last_active_servo = 0
            elif user_change1 and not user_change0:
                if active_servo != 1:
                    lock_servo0 = None  # recapture hold point once, when switching to servo1
                active_servo = 1
                last_active_servo = 1
            else:
                # Both changed in this step: pick the larger move.
                if d0 >= d1:
                    if active_servo != 0:
                        lock_servo1 = None
                    active_servo = 0
                    last_active_servo = 0
                else:
                    if active_servo != 1:
                        lock_servo0 = None
                    active_servo = 1
                    last_active_servo = 1
        else:
            idle_steps += 1
            # Keep the last active servo latched until both joints settle; otherwise the closed
            # chain can back-drive the held joint and it will look like the other servo "moves".
            if active_servo is None and last_active_servo is not None:
                active_servo = last_active_servo
            if idle_steps >= RELEASE_IDLE_STEPS:
                v0 = abs(float(d.qvel[servo0_dof_adr]))
                v1 = abs(float(d.qvel[servo1_dof_adr]))
                if v0 < SERVO_QVEL_EPS and v1 < SERVO_QVEL_EPS:
                    active_servo = None
                    last_active_servo = None
                    lock_servo0 = None
                    lock_servo1 = None

        servo0_qpos = float(d.qpos[servo0_qpos_adr])
        servo1_qpos = float(d.qpos[servo1_qpos_adr])

        def _set_eq(eq_id: int | None, enabled: bool) -> None:
            if eq_id is None:
                return
            d.eq_active[eq_id] = 1 if enabled else 0

        # Always keep the triangle linkage connected (as requested).
        _set_eq(eq_triangle_rod1_a, True)
        _set_eq(eq_triangle_rod1_b, True)
        _set_eq(eq_triangle_rod2_a, True)
        _set_eq(eq_triangle_rod2_b, True)

        if active_servo == 0:
            # Hold servo1 at its last commanded target (not current qpos). This prevents the
            # UI slider from snapping to the current joint angle if the joint hasn't reached
            # the requested target yet.
            if lock_servo1 is None:
                lock_servo1 = user_servo1_ctrl
            lo, hi = map(float, m.actuator_ctrlrange[servo1_act_id])
            d.ctrl[servo1_act_id] = min(max(lock_servo1, lo), hi)
            lock_servo0 = None
        elif active_servo == 1:
            # Hold servo0 at its last commanded target (not current qpos).
            if lock_servo0 is None:
                lock_servo0 = user_servo0_ctrl
            lo, hi = map(float, m.actuator_ctrlrange[servo0_act_id])
            d.ctrl[servo0_act_id] = min(max(lock_servo0, lo), hi)
            lock_servo1 = None
        else:
            lock_servo0 = None
            lock_servo1 = None

    mujoco.set_mjcb_control(control_callback)

    # Pre-roll so the initial pose (especially gripper closure) settles before the first rendered frame.
    for _ in range(800):
        mujoco.mj_step(model, data)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        print(f"MuJoCo viewer running ({DEFAULT_MODEL_PATH}). Close the window or press ESC to exit.")
        if list_cameras:
            print("Available cameras:")
            for i in range(model.ncam):
                name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, i)
                print(f"  {i}: {name}")
        if view_cam_id is not None:
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            viewer.cam.fixedcamid = int(view_cam_id)
        # Hide marker visuals (the colored spheres are `site`s in this model).
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = 0
        if hasattr(viewer.opt, "sitegroup"):
            viewer.opt.sitegroup[:] = 0
        # Hide collision geoms (group=3) like the wheel collision cylinders.
        if hasattr(viewer.opt, "geomgroup"):
            viewer.opt.geomgroup[3] = 0
        step = 0
        frame_idx = 0
        while viewer.is_running():
            # The viewer UI can overwrite options; force them off each frame.
            if view_cam_id is not None:
                viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                viewer.cam.fixedcamid = int(view_cam_id)
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = 0
            if hasattr(viewer.opt, "sitegroup"):
                viewer.opt.sitegroup[:] = 0
            if hasattr(viewer.opt, "geomgroup"):
                viewer.opt.geomgroup[3] = 0
            mujoco.mj_step(model, data)
            if dump_cam_pose and cam_id is not None and (step % max(dump_cam_every, 1) == 0):
                pos = data.cam_xpos[cam_id].copy()
                xmat = data.cam_xmat[cam_id].copy()
                print(f"{cam_name}: pos={pos.tolist()} xmat={xmat.tolist()}")
            if dump_cam_img and renderer is not None and (step % max(cam_img_every, 1) == 0):
                renderer.update_scene(data, camera=cam_name)
                rgb = renderer.render()
                if cam_img_format == "ppm":
                    out = cam_img_dir / f"{cam_name}_{frame_idx:06d}.ppm"
                    _write_ppm(out, rgb)
                else:
                    raise SystemExit(f"Unsupported CAM_IMG_FORMAT={cam_img_format!r}; use 'ppm'")
                frame_idx += 1
            if dump_cam_latest and renderer is not None and (step % max(cam_latest_every, 1) == 0):
                renderer.update_scene(data, camera=cam_name)
                rgb = renderer.render()
                _write_ppm_atomic(cam_latest_path, rgb)
            viewer.sync()
            time.sleep(0.002)
            step += 1


if __name__ == "__main__":
    main()

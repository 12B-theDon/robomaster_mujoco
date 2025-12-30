from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import mujoco
import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

from .servo_config import ServoMapping, load_servo_mappings


@dataclass
class BaseCommand:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


@dataclass
class ArmCommand:
    x: float = 0.0  # forward (arm_base_link frame) [m]
    z: float = 0.0  # up (arm_base_link frame) [m]


@dataclass
class ArmDelta:
    dx: float = 0.0
    dz: float = 0.0


def _safe_id(model: mujoco.MjModel, obj_type: mujoco.mjtObj, name: str) -> Optional[int]:
    try:
        return int(mujoco.mj_name2id(model, obj_type, name))
    except ValueError:
        return None


def _clamp(val: float, lo: float, hi: float) -> float:
    return float(min(max(val, lo), hi))


class MujocoSimNode(Node):
    def __init__(self) -> None:
        super().__init__("mujoco_sim")

        self.declare_parameter("model_path", "/mujoco_rm_ws/models/walker/walker_soft_ball.xml")
        self.declare_parameter("servo_config", "")
        self.declare_parameter("show_viewer", False)
        self.declare_parameter("joy_topic", "joy")
        self.declare_parameter("cmd_vel_topic", "cmd_vel_joy")
        self.declare_parameter("cmd_joint_states_topic", "cmd_joint_states")
        self.declare_parameter("cmd_arm_topic", "cmd_arm")
        self.declare_parameter("cmd_arm_vel_topic", "cmd_arm_vel")
        self.declare_parameter("cmd_gripper_topic", "cmd_gripper")
        self.declare_parameter("wheel_tau_smooth", 0.0)  # s, 0 disables smoothing
        self.declare_parameter("drive_assist_enabled", True)
        self.declare_parameter("drive_assist_tau_min", 0.02)  # s
        # cmd_vel filtering (teleop_twist_joy can briefly output zeros between events).
        self.declare_parameter("cmd_vel_zero_hold_s", 0.06)
        self.declare_parameter("cmd_vel_zero_epsilon", 1e-3)
        self.declare_parameter("base_deadman_button", 5)  # RB (teleop_twist_joy enable_button)
        self.declare_parameter("base_requires_deadman", True)
        self.declare_parameter("arm_deadman_button", 4)  # LB (joy_servo_teleop deadman)
        self.declare_parameter("arm_requires_deadman", True)
        # RoboMaster-style arm command: geometry_msgs/Vector3 on /cmd_arm (x,z) mapped to the 2 arm servos.
        self.declare_parameter("cmd_arm_enabled", False)
        # Joystick-friendly arm velocity command: geometry_msgs/Vector3 on cmd_arm_vel (x/z in m/s).
        # Similar to RoboMaster cmd_arm_vel: integrated at a fixed rate into small dx/dz steps to avoid backlog.
        self.declare_parameter("cmd_arm_vel_enabled", False)
        self.declare_parameter("cmd_arm_frame", "arm_base_link")  # frame used for cmd_arm x/z
        self.declare_parameter("cmd_arm_ee_body", "gripper_link")  # end-effector body used for Jacobian
        # IK tuning: maps x/z deltas (meters) into joint targets each sim tick.
        self.declare_parameter("cmd_arm_damping", 0.01)  # damped least squares lambda
        self.declare_parameter("cmd_arm_gain", 1.0)  # 0..1 blend toward IK solution
        self.declare_parameter("cmd_arm_max_joint_step", 0.12)  # rad per sim tick
        self.declare_parameter("cmd_arm_deadband", 0.002)  # m per tick
        self.declare_parameter("cmd_arm_tolerance", 0.003)  # m, stop correcting when within tolerance
        self.declare_parameter("cmd_arm_full_speed_error", 0.03)  # m, error at which we apply full dq
        # Closed-chain calibration (servo0 affects EE via equality constraints, so analytic Jacobian is 0).
        self.declare_parameter("cmd_arm_calibrate", True)
        self.declare_parameter("cmd_arm_calib_eps", 0.08)  # rad
        self.declare_parameter("cmd_arm_calib_steps", 800)  # simulation steps for settling per sample
        self.declare_parameter("cmd_arm_vel_rate_hz", 20.0)
        self.declare_parameter("cmd_arm_vel_timeout_s", 0.35)
        self.declare_parameter("cmd_arm_vel_deadband_m_s", 0.01)
        self.declare_parameter("cmd_arm_vel_max_step_m", 0.010)
        self.declare_parameter("cmd_arm_vel_requires_deadman", False)
        self.declare_parameter("cmd_gripper_rate", 1.5)  # rad/s (applied to gripper actuator ctrl)
        self.declare_parameter("cmd_gripper_topic_enabled", False)
        # Gripper teleop from joystick (deadman + button), matching RoboMaster teleop_linux behavior.
        self.declare_parameter("gripper_deadman_button", 4)  # LB
        self.declare_parameter("gripper_open_button", 0)  # A
        self.declare_parameter("gripper_close_button", 2)  # X
        self.declare_parameter("gripper_from_joy", True)
        self.declare_parameter("camera_name", "arm_camera")
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        # Allow launch files to pass either int or float (dynamic typing).
        self.declare_parameter(
            "camera_fps",
            None,
            ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter("sim_rate_hz", 500.0)
        self.declare_parameter("arm_telemetry_hz", 5.0)

        model_path = Path(str(self.get_parameter("model_path").value))
        servo_config = str(self.get_parameter("servo_config").value)
        show_viewer = bool(self.get_parameter("show_viewer").value)
        joy_topic = str(self.get_parameter("joy_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        cmd_joint_states_topic = str(self.get_parameter("cmd_joint_states_topic").value)
        cmd_arm_topic = str(self.get_parameter("cmd_arm_topic").value)
        cmd_arm_vel_topic = str(self.get_parameter("cmd_arm_vel_topic").value)
        cmd_gripper_topic = str(self.get_parameter("cmd_gripper_topic").value)
        self._wheel_tau_smooth = float(self.get_parameter("wheel_tau_smooth").value)
        self._drive_assist_enabled = bool(self.get_parameter("drive_assist_enabled").value)
        self._drive_assist_tau_min = float(self.get_parameter("drive_assist_tau_min").value)
        self.cmd_arm_enabled = bool(self.get_parameter("cmd_arm_enabled").value)
        self.cmd_arm_vel_enabled = bool(self.get_parameter("cmd_arm_vel_enabled").value)
        self.cmd_arm_frame = str(self.get_parameter("cmd_arm_frame").value)
        self.cmd_arm_ee_body = str(self.get_parameter("cmd_arm_ee_body").value)
        camera_name = str(self.get_parameter("camera_name").value)
        camera_width = int(self.get_parameter("camera_width").value)
        camera_height = int(self.get_parameter("camera_height").value)
        camera_fps_raw = self.get_parameter("camera_fps").value
        camera_fps = float(camera_fps_raw) if camera_fps_raw is not None else 15.0
        sim_rate_hz = float(self.get_parameter("sim_rate_hz").value)
        arm_telemetry_hz = float(self.get_parameter("arm_telemetry_hz").value)

        if not model_path.exists():
            raise RuntimeError(f"Model path not found: {model_path}")

        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)

        # Optional interactive viewer (third-person). This is independent from the camera publisher.
        self._viewer = None
        if show_viewer:
            try:
                import mujoco.viewer as mjviewer  # type: ignore

                self._viewer = mjviewer.launch_passive(self.model, self.data)
                self.get_logger().info("MuJoCo viewer enabled (close the window to stop updating it).")
            except Exception as exc:  # pragma: no cover
                self.get_logger().warning(f"Failed to start MuJoCo viewer, running headless: {exc}")

        # Servo calibration mapping (joint angle -> actuator ctrl).
        self.servo_mappings: dict[str, ServoMapping] = {}
        if servo_config:
            self.servo_mappings = load_servo_mappings(self.model, Path(servo_config))

        self.base_cmd = BaseCommand()
        self._last_cmd_vel_nonzero = BaseCommand()
        self._last_cmd_vel_nonzero_ns: Optional[int] = None
        self._base_deadman_pressed = False
        self._arm_deadman_pressed = False
        self._gripper_deadman_pressed = False
        self.arm_cmd = ArmCommand()
        self._arm_target_initialized = False
        self._arm_delta = ArmDelta()
        self._cmd_arm_vel_x_m_s = 0.0
        self._cmd_arm_vel_z_m_s = 0.0
        self._cmd_arm_vel_last_msg_ns: Optional[int] = None
        self._cmd_arm_vel_last_tick_ns: Optional[int] = None
        self._cmd_gripper_target: Optional[float] = None
        self._prev_joy_buttons: list[int] = []

        # Command actuator IDs (read by the mecanum control callback).
        self.act_base_vx = _safe_id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "base_vx")
        self.act_base_vy = _safe_id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "base_vy")
        self.act_base_wz = _safe_id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "base_wz")
        if self.act_base_vx is None or self.act_base_vy is None or self.act_base_wz is None:
            raise RuntimeError("Missing base command actuators: base_vx/base_vy/base_wz")

        self.vx_lo, self.vx_hi = map(float, self.model.actuator_ctrlrange[self.act_base_vx])
        self.vy_lo, self.vy_hi = map(float, self.model.actuator_ctrlrange[self.act_base_vy])
        self.wz_lo, self.wz_hi = map(float, self.model.actuator_ctrlrange[self.act_base_wz])

        # Camera setup.
        self.camera_name = camera_name
        self.camera_id = _safe_id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
        if self.camera_id is None:
            raise RuntimeError(f"Camera not found in model: {camera_name}")
        self.renderer = mujoco.Renderer(self.model, width=camera_width, height=camera_height)

        # Publishers.
        # RViz2 Image display defaults to RELIABLE QoS; publish RELIABLE to avoid QoS mismatch.
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        # Use relative topic names so launch namespaces can scope them.
        self.pub_image = self.create_publisher(Image, f"{camera_name}/image_raw", qos)
        self.pub_info = self.create_publisher(CameraInfo, f"{camera_name}/camera_info", qos)

        self.camera_width = camera_width
        self.camera_height = camera_height
        self._camera_info_msg = self._make_camera_info(camera_name, camera_width, camera_height)

        # Subscribers.
        self.sub_cmd = self.create_subscription(Twist, cmd_vel_topic, self._on_cmd_vel, 10)
        self.sub_joint_pos = self.create_subscription(JointState, cmd_joint_states_topic, self._on_joint_states, 10)
        # RoboMaster-style arm/gripper auxiliary commands (for joy_teleop compatibility).
        self.sub_cmd_arm = self.create_subscription(Vector3, cmd_arm_topic, self._on_cmd_arm, 10)
        self.sub_cmd_arm_vel = self.create_subscription(Vector3, cmd_arm_vel_topic, self._on_cmd_arm_vel, 10)
        self.sub_cmd_gripper = self.create_subscription(Float32, cmd_gripper_topic, self._on_cmd_gripper, 10)
        # joy_node publishes with SensorData QoS (BEST_EFFORT). Use matching QoS or we won't receive /joy.
        self.sub_joy = self.create_subscription(Joy, joy_topic, self._on_joy, qos_profile_sensor_data)

        # Arm/gripper actuator IDs (position control).
        self.act_servo0 = _safe_id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "servo_motor_0_act")
        self.act_servo1 = _safe_id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "servo_motor_1_act")
        self.act_gripper = _safe_id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "gripper_pos")

        self.servo0_lo, self.servo0_hi = (0.0, 0.0)
        self.servo1_lo, self.servo1_hi = (0.0, 0.0)
        self.grip_lo, self.grip_hi = (0.0, 0.0)
        if self.act_servo0 is not None:
            self.servo0_lo, self.servo0_hi = map(float, self.model.actuator_ctrlrange[self.act_servo0])
        if self.act_servo1 is not None:
            self.servo1_lo, self.servo1_hi = map(float, self.model.actuator_ctrlrange[self.act_servo1])
        if self.act_gripper is not None:
            self.grip_lo, self.grip_hi = map(float, self.model.actuator_ctrlrange[self.act_gripper])

        # Initialize position actuators to the current joint positions to avoid impulses at startup.
        def _init_actuator_to_joint(act_name: str, joint_name: str) -> None:
            act_id = _safe_id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_name)
            j_id = _safe_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if act_id is None or j_id is None:
                return
            qadr = int(self.model.jnt_qposadr[j_id])
            lo, hi = map(float, self.model.actuator_ctrlrange[act_id])
            self.data.ctrl[act_id] = _clamp(float(self.data.qpos[qadr]), lo, hi)

        _init_actuator_to_joint("servo_motor_0_act", "servo_motor_0")
        _init_actuator_to_joint("servo_motor_1_act", "servo_motor_1")
        _init_actuator_to_joint("gripper_pos", "gripper_hinge_left")

        # Keep explicit joint targets for IK to reduce jitter (don't chase qpos noise).
        self._servo0_qadr: Optional[int] = None
        self._servo1_qadr: Optional[int] = None
        self._gripper_qadr: Optional[int] = None
        self._arm_q_target0: Optional[float] = None
        self._arm_q_target1: Optional[float] = None
        j0 = _safe_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "servo_motor_0")
        j1 = _safe_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "servo_motor_1")
        if j0 is not None:
            self._servo0_qadr = int(self.model.jnt_qposadr[j0])
            self._arm_q_target0 = float(self.data.qpos[self._servo0_qadr])
        if j1 is not None:
            self._servo1_qadr = int(self.model.jnt_qposadr[j1])
            self._arm_q_target1 = float(self.data.qpos[self._servo1_qadr])
        jg = _safe_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "gripper_hinge_left")
        if jg is not None:
            self._gripper_qadr = int(self.model.jnt_qposadr[jg])

        # Joint DOF indices for Jacobian-based arm control (optional).
        self._arm_frame_body_id = _safe_id(self.model, mujoco.mjtObj.mjOBJ_BODY, self.cmd_arm_frame)
        self._arm_ee_body_id = _safe_id(self.model, mujoco.mjtObj.mjOBJ_BODY, self.cmd_arm_ee_body)
        self._servo0_dof = None
        self._servo1_dof = None
        if self.cmd_arm_enabled:
            j0 = _safe_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "servo_motor_0")
            j1 = _safe_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "servo_motor_1")
            if j0 is not None:
                self._servo0_dof = int(self.model.jnt_dofadr[j0])
            if j1 is not None:
                self._servo1_dof = int(self.model.jnt_dofadr[j1])
            if self._arm_frame_body_id is None or self._arm_ee_body_id is None or self._servo0_dof is None or self._servo1_dof is None:
                self.get_logger().warning(
                    "cmd_arm_enabled=true but required bodies/joints are missing; disabling cmd_arm control."
                )
                self.cmd_arm_enabled = False

        # Calibrated mapping: [dx, dz] (meters in cmd_arm_frame) -> [dtheta0, dtheta1] (rad).
        self._arm_delta_to_dtheta: Optional[np.ndarray] = None

        self._pending_joint_targets: dict[int, float] = {}

        # Mecanum control callback (sets wheel actuator controls based on base_vx/vy/wz).
        self._install_mecanum_control_callback()

        # Timers.
        sim_period = 1.0 / max(sim_rate_hz, 1.0)
        self._sim_period = float(sim_period)
        self._sim_timer = self.create_timer(self._sim_period, self._sim_step)

        cam_period = 1.0 / max(camera_fps, 0.1)
        self._cam_timer = self.create_timer(cam_period, self._publish_camera)

        # Optional cmd_arm_vel integrator (RoboMaster-like joystick control).
        if self.cmd_arm_vel_enabled:
            arm_vel_rate_hz = float(self.get_parameter("cmd_arm_vel_rate_hz").value)
            arm_vel_period = 1.0 / max(arm_vel_rate_hz, 1.0)
            self._arm_vel_timer = self.create_timer(arm_vel_period, self._cmd_arm_vel_tick)

        # Optional arm telemetry logs (pretty, periodic).
        self._arm_telem_last_ns: Optional[int] = None
        if arm_telemetry_hz and arm_telemetry_hz > 0.0:
            self._arm_telem_timer = self.create_timer(1.0 / max(arm_telemetry_hz, 0.1), self._log_arm_telemetry)

        # Optional gripper telemetry logs (independent of arm telemetry).
        self.declare_parameter("gripper_telemetry_hz", 0.0)
        self._grip_telem_last_ns: Optional[int] = None
        grip_hz = float(self.get_parameter("gripper_telemetry_hz").value)
        if grip_hz and grip_hz > 0.0:
            self._grip_telem_timer = self.create_timer(1.0 / max(grip_hz, 0.1), self._log_gripper_telemetry)

        self.get_logger().info(
            f"Loaded {model_path} | cmd_vel -> base_vx/vy/wz | publishing /{camera_name}/image_raw ({camera_width}x{camera_height}@{camera_fps}Hz)"
        )

    def _make_camera_info(self, frame_id: str, width: int, height: int) -> CameraInfo:
        msg = CameraInfo()
        msg.header.frame_id = frame_id
        msg.width = width
        msg.height = height

        # MuJoCo camera fovy is in degrees.
        fovy_deg = float(self.model.cam_fovy[self.camera_id])  # type: ignore[index]
        fovy = math.radians(fovy_deg)
        fy = (0.5 * height) / max(math.tan(0.5 * fovy), 1e-9)
        fx = fy
        cx = width * 0.5
        cy = height * 0.5

        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        return msg

    def _log_arm_telemetry(self) -> None:
        if self.act_servo0 is None or self.act_servo1 is None or self._servo0_qadr is None or self._servo1_qadr is None:
            return

        now_ns = int(self.get_clock().now().nanoseconds)
        if self._arm_telem_last_ns is not None and (now_ns - int(self._arm_telem_last_ns)) < int(0.15 * 1e9):
            return
        self._arm_telem_last_ns = now_ns

        q0 = float(self.data.qpos[self._servo0_qadr])
        q1 = float(self.data.qpos[self._servo1_qadr])
        t0 = float(self.data.ctrl[self.act_servo0])
        t1 = float(self.data.ctrl[self.act_servo1])
        lo0, hi0 = (self.servo0_lo, self.servo0_hi)
        lo1, hi1 = (self.servo1_lo, self.servo1_hi)

        def _fmt_row(name: str, q: float, t: float, lo: float, hi: float) -> str:
            err = t - q
            return f"{name:<6} | q={q:+7.3f} rad | tgt={t:+7.3f} rad | err={err:+7.3f} | lim=[{lo:+.3f},{hi:+.3f}]"

        deadman = "1" if self._arm_deadman_pressed else "0"
        gdeadman = "1" if self._gripper_deadman_pressed else "0"
        mode = "cmd_arm_vel" if self.cmd_arm_vel_enabled else ("cmd_arm" if self.cmd_arm_enabled else "joint")
        gmode = "aux" if (self._cmd_gripper_target is not None and self._gripper_deadman_pressed) else "setpoint"
        header = f"[arm] deadman={deadman} mode={mode} | [gripper] deadman={gdeadman} mode={gmode}"
        line0 = _fmt_row("servo0", q0, t0, lo0, hi0)
        line1 = _fmt_row("servo1", q1, t1, lo1, hi1)
        extra = ""
        if self.act_gripper is not None and self._gripper_qadr is not None:
            qg = float(self.data.qpos[self._gripper_qadr])
            tg = float(self.data.ctrl[self.act_gripper])
            extra = "\n  " + _fmt_row("grip", qg, tg, self.grip_lo, self.grip_hi)
            if self._cmd_gripper_target is not None:
                extra += f" | aux_tgt={float(self._cmd_gripper_target):+.3f}"
        self.get_logger().info(f"{header}\n  {line0}\n  {line1}{extra}")

    def _log_gripper_telemetry(self) -> None:
        if self.act_gripper is None or self._gripper_qadr is None:
            return

        now_ns = int(self.get_clock().now().nanoseconds)
        if self._grip_telem_last_ns is not None and (now_ns - int(self._grip_telem_last_ns)) < int(0.15 * 1e9):
            return
        self._grip_telem_last_ns = now_ns

        qg = float(self.data.qpos[self._gripper_qadr])
        tg = float(self.data.ctrl[self.act_gripper])
        err = tg - qg
        deadman = "1" if self._gripper_deadman_pressed else "0"
        mode = "aux" if (self._cmd_gripper_target is not None and self._gripper_deadman_pressed) else "setpoint"
        msg = (
            f"[gripper] deadman={deadman} mode={mode}\n"
            f"  grip | q={qg:+7.3f} rad | tgt={tg:+7.3f} rad | err={err:+7.3f} | lim=[{self.grip_lo:+.3f},{self.grip_hi:+.3f}]"
        )
        if self._cmd_gripper_target is not None:
            msg += f" | aux_tgt={float(self._cmd_gripper_target):+.3f}"
        self.get_logger().info(msg)

    def _on_cmd_vel(self, msg: Twist) -> None:
        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        wz = float(msg.angular.z)

        # If deadman isn't pressed, always accept stop commands immediately.
        if bool(self.get_parameter("base_requires_deadman").value) and not self._base_deadman_pressed:
            self.base_cmd = BaseCommand()
            self._last_cmd_vel_nonzero_ns = None
            return

        eps = float(self.get_parameter("cmd_vel_zero_epsilon").value)
        is_zero = abs(vx) < eps and abs(vy) < eps and abs(wz) < eps
        if is_zero and self._last_cmd_vel_nonzero_ns is not None:
            hold_s = float(self.get_parameter("cmd_vel_zero_hold_s").value)
            now_ns = int(self.get_clock().now().nanoseconds)
            if (now_ns - int(self._last_cmd_vel_nonzero_ns)) < int(hold_s * 1e9):
                # Ignore spurious zero; keep last nonzero command.
                return

        self.base_cmd.vx = vx
        self.base_cmd.vy = vy
        self.base_cmd.wz = wz
        if not is_zero:
            self._last_cmd_vel_nonzero = BaseCommand(vx=vx, vy=vy, wz=wz)
            self._last_cmd_vel_nonzero_ns = int(self.get_clock().now().nanoseconds)

    def _on_cmd_arm(self, msg: Vector3) -> None:
        # RoboMaster semantics: cmd_arm is a delta in meters per message (arm.py multiplies by 1000 to mm).
        # We accumulate deltas and apply them from the sim loop (keeps callback lightweight).
        if self.cmd_arm_vel_enabled:
            return
        self._arm_delta.dx += float(msg.x)
        self._arm_delta.dz += float(msg.z)

    def _on_cmd_arm_vel(self, msg: Vector3) -> None:
        if not self.cmd_arm_vel_enabled:
            return
        # Velocities in m/s on x/z; integrated by _cmd_arm_vel_tick.
        self._cmd_arm_vel_x_m_s = float(msg.x)
        self._cmd_arm_vel_z_m_s = float(msg.z)
        self._cmd_arm_vel_last_msg_ns = int(self.get_clock().now().nanoseconds)

    def _cmd_arm_vel_tick(self) -> None:
        if not self.cmd_arm_vel_enabled:
            return
        # If desired, enforce a local deadman (requires this node to receive Joy messages too).
        if bool(self.get_parameter("cmd_arm_vel_requires_deadman").value) and not self._arm_deadman_pressed:
            self._cmd_arm_vel_last_tick_ns = int(self.get_clock().now().nanoseconds)
            return

        now_ns = int(self.get_clock().now().nanoseconds)
        if self._cmd_arm_vel_last_tick_ns is None:
            self._cmd_arm_vel_last_tick_ns = now_ns
            return
        dt_s = max(0.0, (now_ns - int(self._cmd_arm_vel_last_tick_ns)) * 1e-9)
        self._cmd_arm_vel_last_tick_ns = now_ns

        timeout_s = float(self.get_parameter("cmd_arm_vel_timeout_s").value)
        stale = self._cmd_arm_vel_last_msg_ns is None or (now_ns - int(self._cmd_arm_vel_last_msg_ns)) > int(timeout_s * 1e9)
        vx = 0.0 if stale else float(self._cmd_arm_vel_x_m_s)
        vz = 0.0 if stale else float(self._cmd_arm_vel_z_m_s)

        deadband = float(self.get_parameter("cmd_arm_vel_deadband_m_s").value)
        if abs(vx) < deadband:
            vx = 0.0
        if abs(vz) < deadband:
            vz = 0.0
        if vx == 0.0 and vz == 0.0:
            return

        dx = vx * dt_s
        dz = vz * dt_s
        max_step = float(self.get_parameter("cmd_arm_vel_max_step_m").value)
        if max_step > 0.0:
            dx = float(max(-max_step, min(max_step, dx)))
            dz = float(max(-max_step, min(max_step, dz)))
        if dx == 0.0 and dz == 0.0:
            return

        # Reuse the existing delta pipeline (calibration + step limiting happens in _sim_step).
        self._arm_delta.dx += float(dx)
        self._arm_delta.dz += float(dz)

    def _on_cmd_gripper(self, msg: Float32) -> None:
        if not bool(self.get_parameter("cmd_gripper_topic_enabled").value):
            return
        val = float(msg.data)
        if self.act_gripper is None:
            return
        # RoboMaster teleop uses open/close actions; emulate that behavior with a latched target.
        if val > 0.0:
            self._cmd_gripper_target = float(self.grip_hi)
        elif val < 0.0:
            self._cmd_gripper_target = float(self.grip_lo)

    def _on_joy(self, msg: Joy) -> None:
        # Track base deadman state even if gripper teleop is disabled.
        base_deadman = int(self.get_parameter("base_deadman_button").value)
        if 0 <= base_deadman < len(msg.buttons):
            self._base_deadman_pressed = bool(msg.buttons[base_deadman])
        else:
            self._base_deadman_pressed = False

        arm_deadman = int(self.get_parameter("arm_deadman_button").value)
        if 0 <= arm_deadman < len(msg.buttons):
            self._arm_deadman_pressed = bool(msg.buttons[arm_deadman])
        else:
            self._arm_deadman_pressed = False

        gripper_deadman = int(self.get_parameter("gripper_deadman_button").value)
        if 0 <= gripper_deadman < len(msg.buttons):
            self._gripper_deadman_pressed = bool(msg.buttons[gripper_deadman])
        else:
            self._gripper_deadman_pressed = False

        # Deadman-gated gripper command (deadman + button combination).
        if not bool(self.get_parameter("gripper_from_joy").value):
            return
        if self.act_gripper is None:
            return

        open_b = int(self.get_parameter("gripper_open_button").value)
        close_b = int(self.get_parameter("gripper_close_button").value)

        buttons = [int(b) for b in msg.buttons]
        if not self._prev_joy_buttons:
            self._prev_joy_buttons = [0] * len(buttons)

        def pressed(idx: int) -> bool:
            return 0 <= idx < len(buttons) and buttons[idx] == 1

        def rising(idx: int) -> bool:
            return 0 <= idx < len(buttons) and buttons[idx] == 1 and self._prev_joy_buttons[idx] == 0

        if self._gripper_deadman_pressed:
            if pressed(open_b):
                self._cmd_gripper_target = float(self.grip_hi)
            elif pressed(close_b):
                self._cmd_gripper_target = float(self.grip_lo)

        self._prev_joy_buttons = buttons

    def _on_joint_states(self, msg: JointState) -> None:
        # Position setpoints in joint-angle space by default.
        # - If a name matches a servo mapping `name`, treat position as joint angle [rad].
        # - If a name matches an actuator name, treat position as actuator ctrl [rad].
        for name, pos in zip(msg.name, msg.position):
            if name in self.servo_mappings:
                mapping = self.servo_mappings[name]
                self._pending_joint_targets[mapping.actuator_id] = mapping.angle_to_ctrl(float(pos))
                continue

            # Fallback: accept actuator names directly.
            act_id = _safe_id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            if act_id is not None:
                lo, hi = map(float, self.model.actuator_ctrlrange[act_id])
                self._pending_joint_targets[act_id] = _clamp(float(pos), lo, hi)
                if self.act_gripper is not None and int(act_id) == int(self.act_gripper):
                    # If gripper is being driven by explicit setpoints (cmd_joint_states),
                    # don't let the auxiliary gripper latch path override it.
                    self._cmd_gripper_target = None

    def _install_mecanum_control_callback(self) -> None:
        model = self.model
        data = self.data

        base_body_id = _safe_id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        root_body_id = _safe_id(model, mujoco.mjtObj.mjOBJ_BODY, "builder_root")

        drive_assist_enabled = bool(self._drive_assist_enabled)
        drive_assist_tau_min = float(max(self._drive_assist_tau_min, 1e-4))

        wheel_radius_m = 0.05
        wheel_geom_id = _safe_id(model, mujoco.mjtObj.mjOBJ_GEOM, "wheel_front_left_collision")
        if wheel_geom_id is not None and int(model.geom_type[wheel_geom_id]) == int(mujoco.mjtGeom.mjGEOM_CYLINDER):
            wheel_radius_m = float(model.geom_size[wheel_geom_id][0])

        # Wheelbase geometry from mount positions.
        lx = 0.10
        ly = 0.10
        wheel_body_names = ("wheel_front_left", "wheel_front_right", "wheel_rear_left", "wheel_rear_right")
        wheel_positions = []
        for wheel_name in wheel_body_names:
            wheel_id = _safe_id(model, mujoco.mjtObj.mjOBJ_BODY, wheel_name)
            if wheel_id is not None:
                wheel_positions.append(model.body_pos[wheel_id])
        if wheel_positions:
            lx = float(max(abs(p[0]) for p in wheel_positions))
            ly = float(max(abs(p[1]) for p in wheel_positions))
        k_geom = lx + ly

        # Limits implied by wheel torque (uses model actuator forcerange, not hardcoded).
        wheel_tau_max = 0.0
        wheel_act_id = _safe_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "wheel_front_left_act")
        if wheel_act_id is not None and int(model.actuator_forcerange.shape[0]) > int(wheel_act_id):
            lo, hi = model.actuator_forcerange[int(wheel_act_id)]
            wheel_tau_max = float(min(abs(float(lo)), abs(float(hi))))
        if wheel_tau_max <= 0.0:
            wheel_tau_max = 0.25

        subtree_mass = float(model.body_subtreemass[int(root_body_id)]) if root_body_id is not None else float(model.body_mass.sum())
        if subtree_mass <= 0.0:
            subtree_mass = float(model.body_mass.sum())

        F_xy_max = 4.0 * wheel_tau_max / max(wheel_radius_m, 1e-6)
        Tz_max = F_xy_max * k_geom
        a_xy_max = F_xy_max / max(subtree_mass, 1e-6)

        # Effective yaw inertia about free joint, if present.
        Izz = None
        free_qpos_adr = None
        free_dof_wz = None
        free_joint_id = _safe_id(model, mujoco.mjtObj.mjOBJ_JOINT, "builder_free")
        if free_joint_id is not None:
            mujoco.mj_forward(model, data)
            free_qpos_adr = int(model.jnt_qposadr[int(free_joint_id)])
            dof0 = int(model.jnt_dofadr[int(free_joint_id)])
            free_dof_wz = dof0 + 5
            fullM = np.zeros((model.nv, model.nv), dtype=np.float64)
            mujoco.mj_fullM(model, fullM, data.qM)
            Izz = float(fullM[int(free_dof_wz), int(free_dof_wz)])

        alpha_z_max = (Tz_max / max(Izz, 1e-6)) if Izz is not None else None

        # Needed IDs.
        act_fl = _safe_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "wheel_front_left_act")
        act_fr = _safe_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "wheel_front_right_act")
        act_rl = _safe_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "wheel_rear_left_act")
        act_rr = _safe_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "wheel_rear_right_act")
        if None in (act_fl, act_fr, act_rl, act_rr):
            raise RuntimeError("Missing wheel velocity actuators wheel_*_act")

        # Initialize.
        try:
            data.ctrl[self.act_base_vx] = 0.0
            data.ctrl[self.act_base_vy] = 0.0
            data.ctrl[self.act_base_wz] = 0.0
        except Exception:
            pass

        omega = np.zeros(4, dtype=np.float64)
        omega_filt = np.zeros(4, dtype=np.float64)
        tau_smooth = float(self._wheel_tau_smooth)
        yaw_hold: Optional[float] = None

        def _quat_to_yaw(qw: float, qx: float, qy: float, qz: float) -> float:
            return float(np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)))

        def _wrap_pi(angle: float) -> float:
            return float((angle + np.pi) % (2.0 * np.pi) - np.pi)

        def control_callback(m: mujoco.MjModel, d: mujoco.MjData) -> None:
            nonlocal omega, omega_filt, yaw_hold

            # Clear drive assist forces (we keep only wheel drive here).
            if base_body_id is not None:
                d.xfrc_applied[base_body_id][:] = 0.0
            if root_body_id is not None:
                d.xfrc_applied[root_body_id][:] = 0.0

            vx = float(d.ctrl[self.act_base_vx])
            vy = float(d.ctrl[self.act_base_vy])
            wz = float(d.ctrl[self.act_base_wz])

            omega[0] = (vx - vy - k_geom * wz) / wheel_radius_m  # FL
            omega[1] = (vx + vy + k_geom * wz) / wheel_radius_m  # FR
            omega[2] = (vx + vy - k_geom * wz) / wheel_radius_m  # RL
            omega[3] = (vx - vy + k_geom * wz) / wheel_radius_m  # RR

            if tau_smooth <= 0.0:
                omega_filt[:] = omega
            else:
                dt = float(m.opt.timestep)
                alpha = dt / (tau_smooth + dt)
                omega_filt += alpha * (omega - omega_filt)

            d.ctrl[int(act_fl)] = float(omega_filt[0])
            d.ctrl[int(act_fr)] = float(omega_filt[1])
            d.ctrl[int(act_rl)] = float(omega_filt[2])
            d.ctrl[int(act_rr)] = float(omega_filt[3])

            # Drive assist: apply an ideal planar wrench consistent with wheel torque limits.
            # This makes vx/vy follow commands even with simplified wheel contact.
            if not drive_assist_enabled:
                return
            if root_body_id is None or base_body_id is None:
                return

            xmat = d.xmat[int(base_body_id)].reshape(3, 3)
            x_axis = xmat[:, 0]
            y_axis = xmat[:, 1]
            z_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)

            v_des_world = vx * x_axis + vy * y_axis
            w_des_world = wz * z_axis

            vel6 = np.zeros(6, dtype=np.float64)
            mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, int(base_body_id), vel6, 0)
            w_world = vel6[0:3]
            v_world = vel6[3:6]

            # Use available acceleration up to limits (similar to load_env.py) with a low minimum time constant.
            v_mag = float(max(abs(vx), abs(vy)))
            tau_xy_eff = max(v_mag / max(a_xy_max, 1e-6), drive_assist_tau_min)
            a_world = (v_des_world - v_world) / max(tau_xy_eff, 1e-6)
            F_world = subtree_mass * a_world
            F_xy = F_world.copy()
            F_xy[2] = 0.0
            F_xy_norm = float(np.linalg.norm(F_xy))
            if F_xy_norm > F_xy_max:
                F_xy *= F_xy_max / F_xy_norm

            if alpha_z_max is not None and Izz is not None and free_qpos_adr is not None and free_dof_wz is not None:
                # If wz isn't commanded, hold current heading while translating.
                if abs(wz) < 1e-3:
                    if yaw_hold is None and (abs(vx) >= 1e-3 or abs(vy) >= 1e-3):
                        qw, qx, qy, qz = map(float, d.qpos[int(free_qpos_adr) + 3 : int(free_qpos_adr) + 7])
                        yaw_hold = _quat_to_yaw(qw, qx, qy, qz)
                    elif abs(vx) < 1e-3 and abs(vy) < 1e-3:
                        yaw_hold = None

                    if yaw_hold is not None:
                        qw, qx, qy, qz = map(float, d.qpos[int(free_qpos_adr) + 3 : int(free_qpos_adr) + 7])
                        yaw = _quat_to_yaw(qw, qx, qy, qz)
                        yaw_err = _wrap_pi(yaw - yaw_hold)
                        omega_z = float(d.qvel[int(free_dof_wz)])
                        yaw_err_sat = float(np.deg2rad(10.0))
                        kp = Tz_max / max(yaw_err_sat, 1e-6)
                        kd = 2.0 * np.sqrt(max(kp * Izz, 1e-12))
                        T_world = np.array([0.0, 0.0, -kp * yaw_err - kd * omega_z], dtype=np.float64)
                    else:
                        T_world = np.array([0.0, 0.0, 0.0], dtype=np.float64)
                else:
                    yaw_hold = None
                    tau_wz_eff = max(abs(wz) / max(alpha_z_max, 1e-6), drive_assist_tau_min)
                    alpha_world = (w_des_world - w_world) / max(tau_wz_eff, 1e-6)
                    T_world = np.array([0.0, 0.0, Izz * float(alpha_world[2])], dtype=np.float64)
            else:
                T_world = np.array([0.0, 0.0, Tz_max * (wz - float(w_world[2])) / max(abs(self.wz_hi), 1e-6)], dtype=np.float64)

            T_world[2] = float(np.clip(T_world[2], -Tz_max, Tz_max))
            d.xfrc_applied[int(root_body_id)][0:3] = F_xy
            d.xfrc_applied[int(root_body_id)][3:6] = T_world

        mujoco.set_mjcb_control(control_callback)

    def _calibrate_arm_delta_mapping(self) -> Optional[np.ndarray]:
        """Compute a local mapping from [dx, dz] to [dtheta0, dtheta1].

        The arm linkage is a closed chain (equality constraints), so the standard kinematic Jacobian does
        not capture servo_motor_0 influence on the end-effector. We estimate the 2x2 Jacobian numerically
        by perturbing each servo target and letting the sim settle.
        Returns a 2x2 matrix M such that dtheta ≈ M @ [dx, dz].
        """
        if self.act_servo0 is None or self.act_servo1 is None:
            self.get_logger().warning("Arm calibration skipped: missing servo actuators.")
            return None
        if self._arm_frame_body_id is None or self._arm_ee_body_id is None:
            self.get_logger().warning("Arm calibration skipped: missing arm frame/ee body.")
            return None

        eps = float(self.get_parameter("cmd_arm_calib_eps").value)
        settle_steps = int(self.get_parameter("cmd_arm_calib_steps").value)
        if eps <= 0.0 or settle_steps <= 0:
            return None

        tmp = mujoco.MjData(self.model)
        mujoco.mj_copyState(self.model, self.data, tmp, mujoco.mjtState.mjSTATE_FULLPHYSICS)
        tmp.ctrl[:] = self.data.ctrl

        def settle(d: mujoco.MjData, n: int) -> None:
            for _ in range(n):
                mujoco.mj_step(self.model, d)

        def ee_local(d: mujoco.MjData) -> np.ndarray:
            R = d.xmat[self._arm_frame_body_id].reshape(3, 3)
            p_frame = d.xpos[self._arm_frame_body_id]
            p_ee = d.xpos[self._arm_ee_body_id]
            p_local = R.T @ (p_ee - p_frame)
            return np.array([float(p_local[0]), float(p_local[2])], dtype=np.float64)

        # Baseline settle.
        settle(tmp, settle_steps)
        p0 = ee_local(tmp)

        def sample(act_id: int) -> np.ndarray:
            d2 = mujoco.MjData(self.model)
            mujoco.mj_copyState(self.model, tmp, d2, mujoco.mjtState.mjSTATE_FULLPHYSICS)
            d2.ctrl[:] = tmp.ctrl
            d2.ctrl[act_id] = float(d2.ctrl[act_id]) + eps
            settle(d2, settle_steps)
            p = ee_local(d2)
            return (p - p0) / eps

        # Columns are d[x,z]/dtheta_i
        j0 = sample(self.act_servo0)
        j1 = sample(self.act_servo1)
        J = np.stack([j0, j1], axis=1)  # 2x2

        # Invert (with damping) to get dtheta = J^{-1} dxz.
        lam = float(self.get_parameter("cmd_arm_damping").value)
        try:
            inv = np.linalg.inv(J.T @ J + (lam * lam) * np.eye(2)) @ J.T  # (2x2)
        except np.linalg.LinAlgError:
            self.get_logger().warning(f"Arm calibration failed: singular Jacobian\n{J}")
            return None

        self.get_logger().info(f"Arm calibration OK. J=\n{J}\ninv=\n{inv}")
        return inv

    def _sim_step(self) -> None:
        # Enforce deadman stop at the sim side (prevents lingering motion).
        if bool(self.get_parameter("base_requires_deadman").value) and not self._base_deadman_pressed:
            self.base_cmd = BaseCommand()
            self._last_cmd_vel_nonzero_ns = None

        # Freeze arm when arm deadman isn't pressed (prevents continuing toward an old target).
        # Note: gripper is controlled independently via cmd_joint_states or cmd_gripper; don't drop its targets here.
        if bool(self.get_parameter("arm_requires_deadman").value) and not self._arm_deadman_pressed:
            if self.act_servo0 is not None and self._servo0_qadr is not None:
                q0 = float(self.data.qpos[self._servo0_qadr])
                self.data.ctrl[self.act_servo0] = _clamp(q0, self.servo0_lo, self.servo0_hi)
                self._arm_q_target0 = float(self.data.ctrl[self.act_servo0])
            if self.act_servo1 is not None and self._servo1_qadr is not None:
                q1 = float(self.data.qpos[self._servo1_qadr])
                self.data.ctrl[self.act_servo1] = _clamp(q1, self.servo1_lo, self.servo1_hi)
                self._arm_q_target1 = float(self.data.ctrl[self.act_servo1])
            # Drop any pending targets for these actuators.
            for act_id in [self.act_servo0, self.act_servo1]:
                if act_id is not None and act_id in self._pending_joint_targets:
                    self._pending_joint_targets.pop(act_id, None)

        # Apply latest cmd_vel into the command actuators (clamped to ctrlrange).
        vx = _clamp(self.base_cmd.vx, self.vx_lo, self.vx_hi)
        vy = _clamp(self.base_cmd.vy, self.vy_lo, self.vy_hi)
        wz = _clamp(self.base_cmd.wz, self.wz_lo, self.wz_hi)
        self.data.ctrl[self.act_base_vx] = vx
        self.data.ctrl[self.act_base_vy] = vy
        self.data.ctrl[self.act_base_wz] = wz

        # Apply pending position setpoints (position control, like RoboMaster MoveServo).
        if self._pending_joint_targets:
            for act_id, target in list(self._pending_joint_targets.items()):
                lo, hi = map(float, self.model.actuator_ctrlrange[act_id])
                self.data.ctrl[act_id] = _clamp(target, lo, hi)
            self._pending_joint_targets.clear()

        # Step the sim multiple times to match the ROS timer period.
        dt = float(self.model.opt.timestep)
        steps = max(int(round(self._sim_period / max(dt, 1e-6))), 1)
        dt_total = steps * dt

        # Optional RoboMaster-style arm command: cmd_arm is a delta in meters.
        # The arm linkage is a closed chain via equality constraints, so the analytic Jacobian for servo_motor_0
        # is zero; use a one-time calibrated delta->joint mapping computed from finite differences.
        if (self.cmd_arm_enabled or self.cmd_arm_vel_enabled) and self._arm_frame_body_id is not None and self._arm_ee_body_id is not None:
            deadband = float(self.get_parameter("cmd_arm_deadband").value)
            dx = float(self._arm_delta.dx)
            dz = float(self._arm_delta.dz)
            self._arm_delta.dx = 0.0
            self._arm_delta.dz = 0.0
            if abs(dx) < deadband and abs(dz) < deadband:
                dx = 0.0
                dz = 0.0

            if (dx != 0.0 or dz != 0.0) and self._arm_delta_to_dtheta is None and bool(self.get_parameter("cmd_arm_calibrate").value):
                self._arm_delta_to_dtheta = self._calibrate_arm_delta_mapping()

            if (dx != 0.0 or dz != 0.0) and self._arm_delta_to_dtheta is not None:
                delta = np.array([dx, dz], dtype=np.float64)
                dtheta = self._arm_delta_to_dtheta @ delta

                max_step = float(self.get_parameter("cmd_arm_max_joint_step").value)
                dtheta = np.clip(dtheta, -max_step, max_step)

                if self._arm_q_target0 is None and self._servo0_qadr is not None:
                    self._arm_q_target0 = float(self.data.qpos[self._servo0_qadr])
                if self._arm_q_target1 is None and self._servo1_qadr is not None:
                    self._arm_q_target1 = float(self.data.qpos[self._servo1_qadr])

                if self._arm_q_target0 is not None:
                    self._arm_q_target0 = _clamp(self._arm_q_target0 + float(dtheta[0]), self.servo0_lo, self.servo0_hi)
                    if self.act_servo0 is not None:
                        self.data.ctrl[self.act_servo0] = float(self._arm_q_target0)

                if self._arm_q_target1 is not None:
                    self._arm_q_target1 = _clamp(self._arm_q_target1 + float(dtheta[1]), self.servo1_lo, self.servo1_hi)
                    if self.act_servo1 is not None:
                        self.data.ctrl[self.act_servo1] = float(self._arm_q_target1)

        # Optional cmd_gripper (integrated position target).
        # Only drive this path while its deadman is held (safety: releasing deadman stops motion).
        if (
            self._cmd_gripper_target is not None
            and self.act_gripper is not None
            and (bool(self.get_parameter("cmd_gripper_topic_enabled").value) or bool(self.get_parameter("gripper_from_joy").value))
            and self._gripper_deadman_pressed
        ):
            rate = float(self.get_parameter("cmd_gripper_rate").value)
            cur = float(self.data.ctrl[self.act_gripper])
            tgt = float(self._cmd_gripper_target)
            step = rate * dt_total
            if abs(tgt - cur) <= step:
                cur = tgt
            else:
                cur = cur + step if tgt > cur else cur - step
            self.data.ctrl[self.act_gripper] = _clamp(cur, self.grip_lo, self.grip_hi)

        for _ in range(steps):
            mujoco.mj_step(self.model, self.data)

        if self._viewer is not None:
            try:
                if self._viewer.is_running():
                    self._viewer.sync()
                else:
                    self._viewer.close()
                    self._viewer = None
            except Exception:
                # If the viewer has been closed, keep sim running.
                self._viewer = None

    def _publish_camera(self) -> None:
        now = self.get_clock().now().to_msg()

        self.renderer.update_scene(self.data, camera=self.camera_name)
        rgb = self.renderer.render()

        msg = Image()
        msg.header.stamp = now
        msg.header.frame_id = self.camera_name
        msg.height = self.camera_height
        msg.width = self.camera_width
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = self.camera_width * 3
        msg.data = rgb.tobytes()
        self.pub_image.publish(msg)

        info = self._camera_info_msg
        info.header.stamp = now
        self.pub_info.publish(info)


def main() -> None:
    rclpy.init()
    node = MujocoSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

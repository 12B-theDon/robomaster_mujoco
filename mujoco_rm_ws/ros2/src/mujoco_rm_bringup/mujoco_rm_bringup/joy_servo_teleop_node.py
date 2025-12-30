from __future__ import annotations

from pathlib import Path
from typing import Literal
from typing import Optional

import mujoco
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy, JointState

from .servo_config import ServoMapping, load_servo_mappings


def _safe_id(model: mujoco.MjModel, obj_type: mujoco.mjtObj, name: str) -> Optional[int]:
    try:
        return int(mujoco.mj_name2id(model, obj_type, name))
    except ValueError:
        return None


def _clamp(val: float, lo: float, hi: float) -> float:
    return float(min(max(val, lo), hi))


def _safe_joint_range(model: mujoco.MjModel, joint_name: str) -> Optional[tuple[float, float]]:
    jid = _safe_id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if jid is None:
        return None
    lo, hi = map(float, model.jnt_range[int(jid)])
    return (lo, hi)


class JoyServoTeleop(Node):
    """Position-setpoint teleop for MuJoCo arm/gripper.

    Publishes absolute position setpoints on `/cmd_joint_states`, which the sim node applies
    as actuator target positions (position control), similar to RoboMaster MoveServo usage.
    """

    def __init__(self) -> None:
        super().__init__("joy_servo_teleop")

        self.declare_parameter("model_path", "/mujoco_rm_ws/models/walker/walker_soft_ball.xml")
        self.declare_parameter("servo_config", "")
        self.declare_parameter("joy_topic", "joy")
        self.declare_parameter("cmd_joint_states_topic", "cmd_joint_states")
        self.declare_parameter("deadman_button", 4)  # LB
        self.declare_parameter("rate_hz", 60.0)

        # Axis mapping (joystick axes are typically [-1, 1]).
        self.declare_parameter("servo0_axis", 0)  # left stick left/right
        self.declare_parameter("servo1_axis", 1)  # left stick up/down
        # If servo_mode == 'rate': rad/s at full deflection.
        # If servo_mode == 'absolute': axis scale before mapping to full joint range.
        self.declare_parameter("servo0_rate", 1.0)
        self.declare_parameter("servo1_rate", 1.0)
        self.declare_parameter("servo_mode", "absolute")  # 'absolute' or 'rate'
        self.declare_parameter("axis_deadzone", 0.02)
        self.declare_parameter("absolute_use_offset", True)
        self.declare_parameter("absolute_require_center_on_deadman", True)
        self.declare_parameter("absolute_center_deadband", 0.10)
        self.declare_parameter("debug_print_hz", 0.0)
        # Optional axis auto-detect (for varying joy_node axis orderings).
        # While deadman is held, we measure per-axis min/max and (optionally) auto-bind servo1_axis.
        self.declare_parameter("auto_detect_axes", False)
        self.declare_parameter("auto_detect_window_s", 1.0)
        self.declare_parameter("auto_bind_servo1_axis", True)

        # Gripper buttons (hold deadman too).
        self.declare_parameter("gripper_open_button", 0)   # A
        self.declare_parameter("gripper_close_button", 2)  # X
        # gripper_mode:
        # - 'latch': button press drives to fully open/close and holds (RoboMaster-like)
        # - 'hold': button hold integrates position at gripper_rate
        self.declare_parameter("gripper_mode", "latch")
        self.declare_parameter("gripper_rate", 1.5)  # rad/s (hold mode)
        self.declare_parameter("gripper_latch_rate", 0.6)  # rad/s (latch mode travel speed)

        model_path = Path(str(self.get_parameter("model_path").value))
        servo_config = str(self.get_parameter("servo_config").value)
        if not model_path.exists():
            raise RuntimeError(f"Model path not found: {model_path}")

        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, data)

        if not servo_config:
            raise RuntimeError("servo_config parameter must point to a servos.yaml (robomaster-style)")

        self.mappings: dict[str, ServoMapping] = load_servo_mappings(self.model, Path(servo_config))
        if not self.mappings:
            raise RuntimeError(f"No valid servo mappings loaded from {servo_config}")

        # Required mappings.
        if "servo_motor_0" not in self.mappings or "servo_motor_1" not in self.mappings:
            raise RuntimeError("servos.yaml must include entries named servo_motor_0 and servo_motor_1")

        self.servo0 = self.mappings["servo_motor_0"]
        self.servo1 = self.mappings["servo_motor_1"]
        self.gripper = self.mappings.get("gripper")

        # Use MuJoCo joint limits for mapping joystick -> target angle.
        # (RoboMaster direction/reference mapping is for converting raw servo values; MuJoCo position actuators
        # expect joint angles directly.)
        self.servo0_range = _safe_joint_range(self.model, "servo_motor_0") or (self.servo0.ctrl_lo, self.servo0.ctrl_hi)
        self.servo1_range = _safe_joint_range(self.model, "servo_motor_1") or (self.servo1.ctrl_lo, self.servo1.ctrl_hi)
        self.grip_range = _safe_joint_range(self.model, "gripper_hinge_left") or (
            (self.gripper.ctrl_lo, self.gripper.ctrl_hi) if self.gripper is not None else (0.0, 0.0)
        )

        # Start at the model's initial joint position so the first deadman press doesn't jump the arm.
        def _joint_qpos(joint_name: str, fallback: float) -> float:
            jid = _safe_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if jid is None:
                return float(fallback)
            qadr = int(self.model.jnt_qposadr[jid])
            return float(data.qpos[qadr])

        s0_lo, s0_hi = self.servo0_range
        s1_lo, s1_hi = self.servo1_range
        self.target_servo0 = _joint_qpos("servo_motor_0", 0.5 * (float(s0_lo) + float(s0_hi)))
        self.target_servo1 = _joint_qpos("servo_motor_1", 0.5 * (float(s1_lo) + float(s1_hi)))
        # Gripper: start at the model's current joint position too (otherwise pressing deadman would
        # immediately drive the gripper toward an arbitrary mid-point).
        g_lo, g_hi = self.grip_range
        self.target_grip = _joint_qpos(
            "gripper_hinge_left",
            0.5 * (float(g_lo) + float(g_hi)) if self.gripper is not None else 0.0,
        )

        self._last_joy: Optional[Joy] = None
        self._deadman_prev = False
        self._abs_offset0 = 0.0
        self._abs_offset1 = 0.0
        self._grip_open_prev = False
        self._grip_close_prev = False
        self._grip_latch_target: Optional[float] = None
        self._grip_action_active = False
        self._grip_action_name: Optional[str] = None
        self._last_debug_ns: Optional[int] = None
        self._first_joy_logged = False
        self._axis_min: list[float] = []
        self._axis_max: list[float] = []
        self._axis_first_ns: Optional[int] = None
        self._servo1_axis_effective = int(self.get_parameter("servo1_axis").value)
        self._autobound_servo1 = False

        cmd_joint_states_topic = str(self.get_parameter("cmd_joint_states_topic").value)
        joy_topic = str(self.get_parameter("joy_topic").value)
        self.pub = self.create_publisher(JointState, cmd_joint_states_topic, 10)
        # joy_node publishes with SensorData QoS (BEST_EFFORT). Use matching QoS or we won't receive /joy.
        self.sub = self.create_subscription(Joy, joy_topic, self._on_joy, qos_profile_sensor_data)

        rate_hz = float(self.get_parameter("rate_hz").value)
        self._dt = 1.0 / max(rate_hz, 1.0)
        self._timer = self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f"JoyServoTeleop using {servo_config} publishing {cmd_joint_states_topic} at {rate_hz}Hz (deadman button {int(self.get_parameter('deadman_button').value)})"
        )

    def _on_joy(self, msg: Joy) -> None:
        self._last_joy = msg
        if not self._first_joy_logged:
            self.get_logger().info(
                f"[arm_joy] first msg on {str(self.get_parameter('joy_topic').value)}: "
                f"axes={len(msg.axes)} buttons={len(msg.buttons)} axes_sample={list(msg.axes)[:10]} buttons_sample={list(msg.buttons)[:12]}"
            )
            self._first_joy_logged = True

        if not bool(self.get_parameter("auto_detect_axes").value):
            return

        deadman = int(self.get_parameter("deadman_button").value)
        if not self._button(msg, deadman):
            self._axis_first_ns = None
            self._axis_min = []
            self._axis_max = []
            self._autobound_servo1 = False
            self._servo1_axis_effective = int(self.get_parameter("servo1_axis").value)
            return

        if not self._axis_min or len(self._axis_min) != len(msg.axes):
            self._axis_min = [float(a) for a in msg.axes]
            self._axis_max = [float(a) for a in msg.axes]
            self._axis_first_ns = int(self.get_clock().now().nanoseconds)
            return

        for i, a in enumerate(msg.axes):
            v = float(a)
            if v < self._axis_min[i]:
                self._axis_min[i] = v
            if v > self._axis_max[i]:
                self._axis_max[i] = v

    def _axis(self, msg: Joy, idx: int) -> float:
        if idx < 0 or idx >= len(msg.axes):
            return 0.0
        return float(msg.axes[idx])

    def _button(self, msg: Joy, idx: int) -> bool:
        if idx < 0 or idx >= len(msg.buttons):
            return False
        return bool(msg.buttons[idx])

    def _tick(self) -> None:
        msg = self._last_joy
        if msg is None:
            return

        deadman = int(self.get_parameter("deadman_button").value)
        deadman_pressed = self._button(msg, deadman)
        if not deadman_pressed:
            self._deadman_prev = False
            # Still allow a gripper "action" (latched open/close) to continue while deadman is released,
            # so a single press behaves like an action server: move to end-stop then hold.
            if self.gripper is not None:
                open_b = int(self.get_parameter("gripper_open_button").value)
                close_b = int(self.get_parameter("gripper_close_button").value)
                self._grip_open_prev = self._button(msg, open_b)
                self._grip_close_prev = self._button(msg, close_b)

            # Don't update arm targets without deadman, but still publish the last targets so the sim can
            # keep applying the gripper setpoint.
            self._last_debug_ns = None
            self._autobound_servo1 = False
        else:
            # Print a full snapshot once on deadman press to debug axis mapping quickly.
            if not self._deadman_prev:
                self.get_logger().info(
                    f"[arm_joy] deadman pressed: servo0_axis={int(self.get_parameter('servo0_axis').value)} "
                    f"servo1_axis={int(self.get_parameter('servo1_axis').value)} "
                    f"axes={list(msg.axes)} buttons={list(msg.buttons)}"
                )

        if deadman_pressed:
            servo0_axis = int(self.get_parameter("servo0_axis").value)
            servo1_axis = int(self.get_parameter("servo1_axis").value)
            if bool(self.get_parameter("auto_detect_axes").value) and bool(self.get_parameter("auto_bind_servo1_axis").value):
                # Auto-bind servo1_axis to the most-varying axis excluding servo0_axis.
                if not self._autobound_servo1 and self._axis_first_ns is not None and self._axis_min:
                    now_ns = int(self.get_clock().now().nanoseconds)
                    window_s = float(self.get_parameter("auto_detect_window_s").value)
                    if (now_ns - int(self._axis_first_ns)) >= int(window_s * 1e9):
                        variations = []
                        for i in range(len(self._axis_min)):
                            if i == servo0_axis:
                                continue
                            span = float(self._axis_max[i] - self._axis_min[i])
                            variations.append((span, i))
                        variations.sort(reverse=True)
                        if variations and variations[0][0] > 0.2:
                            self._servo1_axis_effective = int(variations[0][1])
                            self._autobound_servo1 = True
                            self.get_logger().warning(
                                f"[arm_joy] auto-bound servo1_axis={self._servo1_axis_effective} (spans: {variations[:5]})"
                            )
                servo1_axis = int(self._servo1_axis_effective)
            servo0_rate = float(self.get_parameter("servo0_rate").value)
            servo1_rate = float(self.get_parameter("servo1_rate").value)
            servo_mode = str(self.get_parameter("servo_mode").value).strip().lower()
            axis_deadzone = float(self.get_parameter("axis_deadzone").value)
            absolute_use_offset = bool(self.get_parameter("absolute_use_offset").value)
            absolute_require_center = bool(self.get_parameter("absolute_require_center_on_deadman").value)
            absolute_center_deadband = float(self.get_parameter("absolute_center_deadband").value)

            a0_raw = self._axis(msg, servo0_axis)
            a1_raw = self._axis(msg, servo1_axis)

            # Prevent "range shrink" in absolute mode: if deadman is pressed while sticks are deflected,
            # the continuity offset would be computed off-center and can reduce reachable joint span.
            # Require both sticks centered before we start applying absolute mapping.
            if servo_mode == "absolute" and absolute_require_center and not self._deadman_prev:
                if abs(a0_raw) > absolute_center_deadband or abs(a1_raw) > absolute_center_deadband:
                    # Still allow gripper buttons while waiting for centering.
                    a0 = 0.0
                    a1 = 0.0
                else:
                    a0 = a0_raw
                    a1 = a1_raw
            else:
                a0 = a0_raw
                a1 = a1_raw

            if abs(a0) < axis_deadzone:
                a0 = 0.0
            if abs(a1) < axis_deadzone:
                a1 = 0.0

            s0_lo, s0_hi = self.servo0_range
            s1_lo, s1_hi = self.servo1_range
            if servo_mode == "absolute":
                # Map axis [-1,1] to the full joint angle range (matches MuJoCo viewer sliders / ctrlrange).
                n0 = _clamp(a0 * servo0_rate, -1.0, 1.0)
                n1 = _clamp(a1 * servo1_rate, -1.0, 1.0)
                mapped0 = float(s0_lo + 0.5 * (n0 + 1.0) * (s0_hi - s0_lo))
                mapped1 = float(s1_lo + 0.5 * (n1 + 1.0) * (s1_hi - s1_lo))

                # On deadman press, compute an offset so the mapping is continuous (no jump).
                if absolute_use_offset and not self._deadman_prev:
                    self._abs_offset0 = float(self.target_servo0 - mapped0)
                    self._abs_offset1 = float(self.target_servo1 - mapped1)
                if absolute_use_offset:
                    mapped0 = mapped0 + self._abs_offset0
                    mapped1 = mapped1 + self._abs_offset1

                self.target_servo0 = _clamp(mapped0, s0_lo, s0_hi)
                self.target_servo1 = _clamp(mapped1, s1_lo, s1_hi)
            else:
                # Rate mode: integrate axis into position targets.
                self.target_servo0 = _clamp(self.target_servo0 + servo0_rate * a0 * self._dt, s0_lo, s0_hi)
                self.target_servo1 = _clamp(self.target_servo1 + servo1_rate * a1 * self._dt, s1_lo, s1_hi)

            self._deadman_prev = True

        dbg_hz = float(self.get_parameter("debug_print_hz").value)
        if dbg_hz > 0.0 and deadman_pressed:
            now_ns = int(self.get_clock().now().nanoseconds)
            period_ns = int((1.0 / dbg_hz) * 1e9)
            if self._last_debug_ns is None or (now_ns - int(self._last_debug_ns)) >= period_ns:
                self.get_logger().info(
                    f"[arm_joy] mode={servo_mode} axes(servo0={servo0_axis}, servo1={servo1_axis}) "
                    f"raw=({float(a0):+.3f},{float(a1):+.3f}) tgt=({self.target_servo0:+.3f},{self.target_servo1:+.3f})"
                )
                self._last_debug_ns = now_ns

        if self.gripper is not None:
            open_b = int(self.get_parameter("gripper_open_button").value)
            close_b = int(self.get_parameter("gripper_close_button").value)
            lo_ang, hi_ang = self.grip_range
            open_now = self._button(msg, open_b)
            close_now = self._button(msg, close_b)
            grip_mode = str(self.get_parameter("gripper_mode").value).strip().lower()

            if grip_mode == "hold":
                grip_rate = float(self.get_parameter("gripper_rate").value)
                direction = 0.0
                if open_now:
                    direction -= 1.0
                if close_now:
                    direction += 1.0
                if abs(direction) > 0.0:
                    self.target_grip = _clamp(self.target_grip + grip_rate * direction * self._dt, lo_ang, hi_ang)
                self._grip_latch_target = None
                self._grip_action_active = False
            else:
                # Latch mode as a simple action:
                # - Start only when deadman is pressed.
                # - Once started, continue toward end-stop even if deadman is released.
                # - Ignore new commands until reaching the end-stop (no mid-stop).
                if deadman_pressed and not self._grip_action_active:
                    open_edge = open_now and not self._grip_open_prev
                    close_edge = close_now and not self._grip_close_prev
                    if open_edge and not close_edge:
                        # Desired semantics: A=open to the OPEN end-stop.
                        open_target = float(lo_ang)
                        if abs(self.target_grip - open_target) < 1e-3:
                            self.get_logger().info("[gripper_action] already OPEN")
                        else:
                            self._grip_latch_target = open_target
                            self._grip_action_active = True
                            self._grip_action_name = "OPEN"
                            self.get_logger().info("[gripper_action] OPEN started")
                    elif close_edge and not open_edge:
                        # Desired semantics: X=close to the CLOSE end-stop.
                        close_target = float(hi_ang)
                        if abs(self.target_grip - close_target) < 1e-3:
                            self.get_logger().info("[gripper_action] already CLOSED")
                        else:
                            self._grip_latch_target = close_target
                            self._grip_action_active = True
                            self._grip_action_name = "CLOSE"
                            self.get_logger().info("[gripper_action] CLOSE started")

                # Rate-limit travel toward the latched end-stop.
                if self._grip_latch_target is not None:
                    latch_rate = float(self.get_parameter("gripper_latch_rate").value)
                    step = max(latch_rate, 0.0) * self._dt
                    err = float(self._grip_latch_target - self.target_grip)
                    if abs(err) <= step:
                        self.target_grip = float(self._grip_latch_target)
                        self._grip_action_active = False
                        if self._grip_action_name is not None:
                            self.get_logger().info(f"[gripper_action] {self._grip_action_name} done")
                        self._grip_action_name = None
                    else:
                        self.target_grip = float(self.target_grip + step if err > 0.0 else self.target_grip - step)
                    self.target_grip = _clamp(self.target_grip, lo_ang, hi_ang)

            self._grip_open_prev = open_now
            self._grip_close_prev = close_now

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        # Publish actuator target positions directly (MuJoCo position actuators take joint angles as ctrl).
        out.name = [self.servo0.spec.actuator, self.servo1.spec.actuator]
        out.position = [float(self.target_servo0), float(self.target_servo1)]
        if self.gripper is not None:
            out.name.append(self.gripper.spec.actuator)
            out.position.append(float(self.target_grip))
        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = JoyServoTeleop()
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

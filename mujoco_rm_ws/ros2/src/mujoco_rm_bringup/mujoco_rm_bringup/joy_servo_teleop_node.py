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


def _angle_bounds(mapping: ServoMapping) -> tuple[float, float]:
    a0 = mapping.ctrl_to_angle(mapping.ctrl_lo)
    a1 = mapping.ctrl_to_angle(mapping.ctrl_hi)
    return (min(a0, a1), max(a0, a1))


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

        # Start at the model's initial joint position so the first deadman press doesn't jump the arm.
        def _joint_qpos(joint_name: str, fallback: float) -> float:
            jid = _safe_id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if jid is None:
                return float(fallback)
            qadr = int(self.model.jnt_qposadr[jid])
            return float(data.qpos[qadr])

        self.target_servo0 = _joint_qpos("servo_motor_0", self.servo0.ctrl_to_angle(0.5 * (self.servo0.ctrl_lo + self.servo0.ctrl_hi)))
        self.target_servo1 = _joint_qpos("servo_motor_1", self.servo1.ctrl_to_angle(0.5 * (self.servo1.ctrl_lo + self.servo1.ctrl_hi)))
        self.target_grip = (
            self.gripper.ctrl_to_angle(0.5 * (self.gripper.ctrl_lo + self.gripper.ctrl_hi))
            if self.gripper is not None
            else 0.0
        )

        self._last_joy: Optional[Joy] = None
        self._deadman_prev = False
        self._abs_offset0 = 0.0
        self._abs_offset1 = 0.0
        self._grip_open_prev = False
        self._grip_close_prev = False
        self._grip_latch_target: Optional[float] = None

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
            self._grip_open_prev = False
            self._grip_close_prev = False
            self._grip_latch_target = None
            return

        servo0_axis = int(self.get_parameter("servo0_axis").value)
        servo1_axis = int(self.get_parameter("servo1_axis").value)
        servo0_rate = float(self.get_parameter("servo0_rate").value)
        servo1_rate = float(self.get_parameter("servo1_rate").value)
        servo_mode = str(self.get_parameter("servo_mode").value).strip().lower()
        axis_deadzone = float(self.get_parameter("axis_deadzone").value)
        absolute_use_offset = bool(self.get_parameter("absolute_use_offset").value)

        a0 = self._axis(msg, servo0_axis)
        a1 = self._axis(msg, servo1_axis)
        if abs(a0) < axis_deadzone:
            a0 = 0.0
        if abs(a1) < axis_deadzone:
            a1 = 0.0

        s0_lo, s0_hi = _angle_bounds(self.servo0)
        s1_lo, s1_hi = _angle_bounds(self.servo1)
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

        if self.gripper is not None:
            open_b = int(self.get_parameter("gripper_open_button").value)
            close_b = int(self.get_parameter("gripper_close_button").value)
            lo_ang, hi_ang = _angle_bounds(self.gripper)
            open_now = self._button(msg, open_b)
            close_now = self._button(msg, close_b)
            grip_mode = str(self.get_parameter("gripper_mode").value).strip().lower()

            if grip_mode == "hold":
                grip_rate = float(self.get_parameter("gripper_rate").value)
                direction = 0.0
                if open_now:
                    direction += 1.0
                if close_now:
                    direction -= 1.0
                if abs(direction) > 0.0:
                    self.target_grip = _clamp(self.target_grip + grip_rate * direction * self._dt, lo_ang, hi_ang)
                self._grip_latch_target = None
            else:
                # Latch mode: rising edge drives to an end-stop and holds.
                open_edge = open_now and not self._grip_open_prev
                close_edge = close_now and not self._grip_close_prev
                if open_edge and not close_edge:
                    self._grip_latch_target = float(hi_ang)
                elif close_edge and not open_edge:
                    self._grip_latch_target = float(lo_ang)

                # Rate-limit travel toward the latched end-stop.
                if self._grip_latch_target is not None:
                    latch_rate = float(self.get_parameter("gripper_latch_rate").value)
                    step = max(latch_rate, 0.0) * self._dt
                    err = float(self._grip_latch_target - self.target_grip)
                    if abs(err) <= step:
                        self.target_grip = float(self._grip_latch_target)
                    else:
                        self.target_grip = float(self.target_grip + step if err > 0.0 else self.target_grip - step)
                    self.target_grip = _clamp(self.target_grip, lo_ang, hi_ang)

            self._grip_open_prev = open_now
            self._grip_close_prev = close_now

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = ["servo_motor_0", "servo_motor_1"]
        out.position = [float(self.target_servo0), float(self.target_servo1)]
        if self.gripper is not None:
            out.name.append("gripper")
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
        rclpy.shutdown()


if __name__ == "__main__":
    main()

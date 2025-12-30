from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import mujoco
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy


def _safe_id(model: mujoco.MjModel, obj_type: mujoco.mjtObj, name: str) -> Optional[int]:
    try:
        return int(mujoco.mj_name2id(model, obj_type, name))
    except ValueError:
        return None


def _clamp(val: float, lo: float, hi: float) -> float:
    return float(min(max(val, lo), hi))


@dataclass(frozen=True)
class AxisBounds:
    neg: float  # magnitude for negative direction
    pos: float  # magnitude for positive direction


class JoyBaseTeleop(Node):
    """Joystick -> /cmd_vel with continuous output (no zero gaps).

    teleop_twist_joy can emit intermittent zeros depending on joystick event timing.
    This node latches the last joystick state and publishes cmd_vel at a fixed rate,
    mapping stick deflection either:
      - to the MuJoCo model's actuator ctrlrange (default), or
      - with teleop_twist_joy-style fixed scales (mode='scaled').
    """

    def __init__(self) -> None:
        super().__init__("joy_base_teleop")

        self.declare_parameter("model_path", "/mujoco_rm_ws/models/walker/walker_soft_ball.xml")
        self.declare_parameter("joy_topic", "joy")
        self.declare_parameter("cmd_vel_topic", "cmd_vel_joy")
        self.declare_parameter("rate_hz", 60.0)
        self.declare_parameter("deadman_button", 5)  # RB
        self.declare_parameter("mode", "model_ctrlrange")  # 'model_ctrlrange' or 'scaled'
        self.declare_parameter("axis_x", 1)  # left stick up/down
        self.declare_parameter("axis_y", 0)  # left stick left/right
        self.declare_parameter("axis_yaw", 3)  # right stick left/right
        self.declare_parameter("axis_deadzone", 0.02)
        # teleop_twist_joy-style scales (used when mode='scaled')
        self.declare_parameter("scale_linear_x", 2.8)
        self.declare_parameter("scale_linear_y", 2.8)
        self.declare_parameter("scale_angular_yaw", 10.472)
        self.declare_parameter("turbo_button", -1)
        self.declare_parameter("scale_linear_x_turbo", 2.8)
        self.declare_parameter("scale_linear_y_turbo", 2.8)
        self.declare_parameter("scale_angular_yaw_turbo", 10.472)
        self.declare_parameter("debug_print_hz", 0.0)
        self.declare_parameter("diagnose_vxvy_zero", True)
        # If vx/vy stays zero, try to infer which /joy axes correspond to translation.
        self.declare_parameter("auto_detect_axes", True)
        self.declare_parameter("auto_detect_window_s", 2.0)
        self.declare_parameter("auto_bind_axes", True)

        model_path = Path(str(self.get_parameter("model_path").value))
        if not model_path.exists():
            raise RuntimeError(f"Model path not found: {model_path}")

        model = mujoco.MjModel.from_xml_path(str(model_path))

        def _bounds(act_name: str, default: tuple[float, float]) -> AxisBounds:
            act_id = _safe_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_name)
            if act_id is None:
                lo, hi = default
            else:
                lo, hi = map(float, model.actuator_ctrlrange[act_id])
            # Allow asymmetric ranges, e.g. [-2.5, 2.8]
            return AxisBounds(neg=abs(lo), pos=abs(hi))

        self.vx_bounds = _bounds("base_vx", (-2.5, 2.8))
        self.vy_bounds = _bounds("base_vy", (-2.8, 2.8))
        self.wz_bounds = _bounds("base_wz", (-10.472, 10.472))

        self._last_joy: Optional[Joy] = None
        self._last_joy_ns: Optional[int] = None
        self._vxvy_zero_ticks = 0
        self._axis_min: list[float] = []
        self._axis_max: list[float] = []
        self._axis_first_ns: Optional[int] = None
        self._axis_x_effective = int(self.get_parameter("axis_x").value)
        self._axis_y_effective = int(self.get_parameter("axis_y").value)
        self._axis_logged = False
        self._autobound_once = False
        self._last_debug_ns: Optional[int] = None
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        joy_topic = str(self.get_parameter("joy_topic").value)
        # joy_node publishes with SensorData QoS (BEST_EFFORT). Use matching QoS or we won't receive /joy.
        self.sub = self.create_subscription(Joy, joy_topic, self._on_joy, qos_profile_sensor_data)

        rate_hz = float(self.get_parameter("rate_hz").value)
        self._dt = 1.0 / max(rate_hz, 1.0)
        self._timer = self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f"JoyBaseTeleop publishing {cmd_vel_topic} at {rate_hz}Hz (deadman button {int(self.get_parameter('deadman_button').value)}) "
            f"vx=[-{self.vx_bounds.neg},{self.vx_bounds.pos}] vy=[-{self.vy_bounds.neg},{self.vy_bounds.pos}] wz=[-{self.wz_bounds.neg},{self.wz_bounds.pos}]"
        )

    def _on_joy(self, msg: Joy) -> None:
        self._last_joy = msg
        self._last_joy_ns = int(self.get_clock().now().nanoseconds)

        if not self._axis_logged:
            self.get_logger().info(
                f"First Joy msg on {str(self.get_parameter('joy_topic').value)}: "
                f"axes={len(msg.axes)} buttons={len(msg.buttons)} axes_sample={list(msg.axes)[:6]} buttons_sample={list(msg.buttons)[:10]}"
            )
            self._axis_logged = True

        if not bool(self.get_parameter("auto_detect_axes").value):
            return

        # Track per-axis min/max while deadman is pressed, then suggest a mapping if vx/vy appear stuck.
        deadman = int(self.get_parameter("deadman_button").value)
        if not self._button(msg, deadman):
            self._axis_first_ns = None
            self._axis_min = []
            self._axis_max = []
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

    def _scale(self, val: float, bounds: AxisBounds) -> float:
        val = _clamp(val, -1.0, 1.0)
        if val >= 0.0:
            return val * bounds.pos
        return val * bounds.neg

    def _scaled_cmd(self, msg: Joy) -> Twist:
        axis_deadzone = float(self.get_parameter("axis_deadzone").value)
        axis_yaw = int(self.get_parameter("axis_yaw").value)

        ax = self._axis(msg, self._axis_x_effective)
        ay = self._axis(msg, self._axis_y_effective)
        aw = self._axis(msg, axis_yaw)
        if abs(ax) < axis_deadzone:
            ax = 0.0
        if abs(ay) < axis_deadzone:
            ay = 0.0
        if abs(aw) < axis_deadzone:
            aw = 0.0

        turbo_button = int(self.get_parameter("turbo_button").value)
        turbo = turbo_button >= 0 and self._button(msg, turbo_button)
        if turbo:
            sx = float(self.get_parameter("scale_linear_x_turbo").value)
            sy = float(self.get_parameter("scale_linear_y_turbo").value)
            sw = float(self.get_parameter("scale_angular_yaw_turbo").value)
        else:
            sx = float(self.get_parameter("scale_linear_x").value)
            sy = float(self.get_parameter("scale_linear_y").value)
            sw = float(self.get_parameter("scale_angular_yaw").value)

        out = Twist()
        out.linear.x = float(ax * sx)
        out.linear.y = float(ay * sy)
        out.angular.z = float(aw * sw)
        return out

    def _tick(self) -> None:
        msg = self._last_joy
        if msg is None:
            return

        deadman = int(self.get_parameter("deadman_button").value)
        axis_deadzone = float(self.get_parameter("axis_deadzone").value)
        diagnose = bool(self.get_parameter("diagnose_vxvy_zero").value)
        auto_detect = bool(self.get_parameter("auto_detect_axes").value)
        auto_bind = bool(self.get_parameter("auto_bind_axes").value)
        mode = str(self.get_parameter("mode").value).strip().lower()

        out = Twist()
        if self._button(msg, deadman):
            axis_yaw = int(self.get_parameter("axis_yaw").value)

            # Optionally auto-bind x/y to the most varying axes while deadman is held.
            if auto_detect and auto_bind and not self._autobound_once and self._axis_first_ns is not None and self._axis_min:
                now_ns = int(self.get_clock().now().nanoseconds)
                window_s = float(self.get_parameter("auto_detect_window_s").value)
                if (now_ns - int(self._axis_first_ns)) >= int(window_s * 1e9):
                    variations = []
                    for i in range(len(self._axis_min)):
                        if i == axis_yaw:
                            continue
                        span = float(self._axis_max[i] - self._axis_min[i])
                        variations.append((span, i))
                    variations.sort(reverse=True)
                    if len(variations) >= 2 and variations[0][0] > 0.2 and variations[1][0] > 0.2:
                        self._axis_x_effective = variations[0][1]
                        self._axis_y_effective = variations[1][1]
                        self._autobound_once = True
                        self.get_logger().warning(
                            f"Auto-bound base axes: axis_x={self._axis_x_effective} axis_y={self._axis_y_effective} axis_yaw={axis_yaw} "
                            f"(spans: {variations[:5]})"
                        )

            if mode == "scaled":
                out = self._scaled_cmd(msg)
            else:
                ax = self._axis(msg, self._axis_x_effective)
                ay = self._axis(msg, self._axis_y_effective)
                aw = self._axis(msg, axis_yaw)
                if abs(ax) < axis_deadzone:
                    ax = 0.0
                if abs(ay) < axis_deadzone:
                    ay = 0.0
                if abs(aw) < axis_deadzone:
                    aw = 0.0
                out.linear.x = self._scale(ax, self.vx_bounds)
                out.linear.y = self._scale(ay, self.vy_bounds)
                out.angular.z = self._scale(aw, self.wz_bounds)

            if diagnose:
                if abs(out.linear.x) < 1e-6 and abs(out.linear.y) < 1e-6 and abs(out.angular.z) > 0.2:
                    self._vxvy_zero_ticks += 1
                    # ~2 seconds at 60Hz.
                    if self._vxvy_zero_ticks == 120:
                        self.get_logger().warning(
                            "Base vx/vy staying at 0 while wz changes. Likely joystick axis mapping mismatch. "
                            "Check /joy axes and adjust launch args base_axis_x/base_axis_y."
                        )
                        if auto_detect and self._axis_first_ns is not None and self._axis_min:
                            now_ns = int(self.get_clock().now().nanoseconds)
                            window_s = float(self.get_parameter("auto_detect_window_s").value)
                            if (now_ns - int(self._axis_first_ns)) >= int(window_s * 1e9):
                                yaw_idx = int(self.get_parameter("axis_yaw").value)
                                variations = []
                                for i in range(len(self._axis_min)):
                                    if i == yaw_idx:
                                        continue
                                    span = float(self._axis_max[i] - self._axis_min[i])
                                    variations.append((span, i))
                                variations.sort(reverse=True)
                                if len(variations) >= 2 and variations[0][0] > 0.2 and variations[1][0] > 0.2:
                                    guess_x = variations[0][1]
                                    guess_y = variations[1][1]
                                    self.get_logger().warning(
                                        f"Auto-detect suggestion: base_axis_x:={guess_x} base_axis_y:={guess_y} "
                                        f"(measured axis spans: {variations[:5]})"
                                    )
                else:
                    self._vxvy_zero_ticks = 0
        else:
            # Always publish an explicit stop when deadman is released.
            out.linear.x = 0.0
            out.linear.y = 0.0
            out.angular.z = 0.0
            self._vxvy_zero_ticks = 0
            self._axis_first_ns = None
            self._axis_min = []
            self._axis_max = []
            self._autobound_once = False

        self.pub.publish(out)

        dbg_hz = float(self.get_parameter("debug_print_hz").value)
        if dbg_hz > 0.0 and self._button(msg, deadman):
            now_ns = int(self.get_clock().now().nanoseconds)
            period_ns = int((1.0 / dbg_hz) * 1e9)
            if self._last_debug_ns is None or (now_ns - int(self._last_debug_ns)) >= period_ns:
                axis_yaw = int(self.get_parameter("axis_yaw").value)
                ax_raw = self._axis(msg, self._axis_x_effective)
                ay_raw = self._axis(msg, self._axis_y_effective)
                aw_raw = self._axis(msg, axis_yaw)
                mode = str(self.get_parameter("mode").value).strip().lower()
                self.get_logger().info(
                    f"deadman=1 mode={mode} axes(x={self._axis_x_effective},y={self._axis_y_effective},yaw={axis_yaw}) "
                    f"raw=({ax_raw:+.3f},{ay_raw:+.3f},{aw_raw:+.3f}) cmd=({out.linear.x:+.3f},{out.linear.y:+.3f},{out.angular.z:+.3f})"
                )
                self._last_debug_ns = now_ns

        # Helpful warning if /joy isn't arriving (common when QoS mismatches or another process holds the device).
        if self._last_joy_ns is not None:
            now_ns = int(self.get_clock().now().nanoseconds)
            if (now_ns - int(self._last_joy_ns)) > int(2.0 * 1e9):
                self.get_logger().warning(f"No /joy messages for >2s on {str(self.get_parameter('joy_topic').value)}")
                self._last_joy_ns = now_ns


def main() -> None:
    rclpy.init()
    node = JoyBaseTeleop()
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

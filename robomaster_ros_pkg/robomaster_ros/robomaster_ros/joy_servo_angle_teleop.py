#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from sensor_msgs.msg import Joy

from robomaster_msgs.action import MoveServo
from robomaster_msgs.msg import ServoRawState


@dataclass
class ServoTarget:
    index: int
    axis: int
    rate_rad_s: float
    min_rad: float
    max_rad: float
    target_rad: float = 0.0
    last_sent_rad: float = 0.0


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class JoyServoAngleTeleop(Node):
    """
    Joystick -> servo angle setpoints (rad) using the `move_servo` action.

    Notes:
    - The RoboMaster ROS servo action server only accepts one goal at a time, so this node
      rate-limits goals and alternates between servo0/servo1 as needed.
    - This is intended for coarse manual positioning, not high-rate control.
    """

    def __init__(self) -> None:
        super().__init__("joy_servo_angle_teleop")

        self.declare_parameter("deadman_button", 6)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("send_threshold_rad", 0.01)
        self.declare_parameter("print_rate_hz", 5.0)

        self.declare_parameter("servo0_index", 0)
        self.declare_parameter("servo1_index", 1)
        self.declare_parameter("servo0_axis", 3)
        self.declare_parameter("servo1_axis", 1)
        self.declare_parameter("servo0_rate_rad_s", 1.0)
        self.declare_parameter("servo1_rate_rad_s", 1.0)

        self.declare_parameter("servo0_min_rad", -math.pi)
        self.declare_parameter("servo0_max_rad", math.pi)
        self.declare_parameter("servo1_min_rad", -math.pi)
        self.declare_parameter("servo1_max_rad", math.pi)

        deadman_button = int(self.get_parameter("deadman_button").value)

        self._send_threshold_rad = float(self.get_parameter("send_threshold_rad").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        publish_rate_hz = max(1.0, publish_rate_hz)

        self._servo0 = ServoTarget(
            index=int(self.get_parameter("servo0_index").value),
            axis=int(self.get_parameter("servo0_axis").value),
            rate_rad_s=float(self.get_parameter("servo0_rate_rad_s").value),
            min_rad=float(self.get_parameter("servo0_min_rad").value),
            max_rad=float(self.get_parameter("servo0_max_rad").value),
        )
        self._servo1 = ServoTarget(
            index=int(self.get_parameter("servo1_index").value),
            axis=int(self.get_parameter("servo1_axis").value),
            rate_rad_s=float(self.get_parameter("servo1_rate_rad_s").value),
            min_rad=float(self.get_parameter("servo1_min_rad").value),
            max_rad=float(self.get_parameter("servo1_max_rad").value),
        )

        self._deadman_button = deadman_button
        self._joy: Optional[Joy] = None
        self._last_tick_ns: Optional[int] = None

        self._busy = False
        self._last_sent_servo: Optional[int] = None

        self._ac = ActionClient(self, MoveServo, "move_servo")
        self.create_subscription(Joy, "joy", self._on_joy, 10)
        self.create_subscription(ServoRawState, "servo_raw_state", self._on_servo_raw_state, 10)
        self.create_timer(1.0 / publish_rate_hz, self._tick)

        self.get_logger().info(
            f"JoyServoAngleTeleop enabled: holding deadman button {self._deadman_button} and moving axes "
            f"(servo0 axis={self._servo0.axis}, servo1 axis={self._servo1.axis}) sends MoveServo goals in radians."
        )

        self._last_action_warn_ns: Optional[int] = None
        self._move_servo_available = False

        self._servo_raw: Optional[ServoRawState] = None
        print_rate_hz = max(0.1, float(self.get_parameter("print_rate_hz").value))
        self.create_timer(1.0 / print_rate_hz, self._print_servo_state)

    def _on_joy(self, msg: Joy) -> None:
        self._joy = msg

    def _on_servo_raw_state(self, msg: ServoRawState) -> None:
        self._servo_raw = msg

    def _axis(self, joy: Joy, axis_index: int) -> float:
        if axis_index < 0:
            return 0.0
        if axis_index >= len(joy.axes):
            return 0.0
        return float(joy.axes[axis_index])

    def _deadman_pressed(self, joy: Joy) -> bool:
        b = self._deadman_button
        return 0 <= b < len(joy.buttons) and int(joy.buttons[b]) == 1

    def _integrate_targets(self, joy: Joy, dt_s: float) -> None:
        for servo in (self._servo0, self._servo1):
            cmd = self._axis(joy, servo.axis)
            if abs(cmd) < 1e-3:
                continue
            servo.target_rad += cmd * servo.rate_rad_s * dt_s
            servo.target_rad = _clamp(servo.target_rad, servo.min_rad, servo.max_rad)

    def _pick_next_servo(self) -> Optional[ServoTarget]:
        d0 = abs(self._servo0.target_rad - self._servo0.last_sent_rad)
        d1 = abs(self._servo1.target_rad - self._servo1.last_sent_rad)
        needs0 = d0 >= self._send_threshold_rad
        needs1 = d1 >= self._send_threshold_rad
        if not (needs0 or needs1):
            return None
        if needs0 and needs1 and self._last_sent_servo is not None:
            if self._last_sent_servo == 0:
                return self._servo1
            return self._servo0
        if needs0 and (not needs1 or d0 >= d1):
            return self._servo0
        return self._servo1

    def _send_goal(self, servo: ServoTarget) -> None:
        if self._busy:
            return
        if not self._ac.wait_for_server(timeout_sec=0.0):
            now_ns = int(self.get_clock().now().nanoseconds)
            if self._last_action_warn_ns is None or (now_ns - self._last_action_warn_ns) > int(2e9):
                self.get_logger().warn(
                    "move_servo action server not available (arm:=false servo:=true in main.launch? and same namespace?)"
                )
                self._last_action_warn_ns = now_ns
            return
        self._move_servo_available = True
        goal = MoveServo.Goal()
        goal.index = int(servo.index)
        goal.angle = float(servo.target_rad)
        self._busy = True
        self._last_sent_servo = 0 if servo is self._servo0 else 1

        future = self._ac.send_goal_async(goal)

        def _on_goal_done(fut) -> None:  # noqa: ANN001
            gh = fut.result()
            if gh is None or not gh.accepted:
                self.get_logger().warn(f"MoveServo goal rejected (index={servo.index})")
                self._busy = False
                return

            result_future = gh.get_result_async()

            def _on_result_done(_rf) -> None:  # noqa: ANN001
                servo.last_sent_rad = servo.target_rad
                self._busy = False

            result_future.add_done_callback(_on_result_done)

        future.add_done_callback(_on_goal_done)

    def _tick(self) -> None:
        joy = self._joy
        if joy is None:
            return

        now_ns = int(self.get_clock().now().nanoseconds)
        if self._last_tick_ns is None:
            self._last_tick_ns = now_ns
            return
        dt_s = max(0.0, (now_ns - self._last_tick_ns) * 1e-9)
        self._last_tick_ns = now_ns

        if self._deadman_pressed(joy):
            self._integrate_targets(joy, dt_s)

        if self._busy:
            return
        servo = self._pick_next_servo()
        if servo is None:
            return
        self._send_goal(servo)

    def _print_servo_state(self) -> None:
        msg = self._servo_raw
        if msg is None:
            return
        i0 = int(self._servo0.index)
        i1 = int(self._servo1.index)
        if i0 >= len(msg.value) or i1 >= len(msg.value):
            return
        v0 = int(msg.value[i0])
        v1 = int(msg.value[i1])
        ok0 = bool(msg.valid[i0]) if i0 < len(msg.valid) else False
        ok1 = bool(msg.valid[i1]) if i1 < len(msg.valid) else False
        status = "move_servo=OK" if self._move_servo_available else "move_servo=N/A"
        self.get_logger().info(f"[servo_raw_state] {status} | servo0(idx={i0}) value={v0} valid={ok0} | servo1(idx={i1}) value={v1} valid={ok1}")


def main() -> None:
    rclpy.init()
    node = JoyServoAngleTeleop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

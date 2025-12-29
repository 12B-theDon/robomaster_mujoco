#!/usr/bin/env python3
import sys, termios, tty, select, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from rclpy.action import ActionClient

from robomaster_msgs.action import GripperControl
from robomaster_msgs.msg import CameraConfig

HELP = """
[RM KB Extra]
Arm (cmd_arm: Vector3, 상대 이동):
  W/S : end-effector +Z / -Z
  A/D : end-effector -X / +X
  R/F : end-effector +Y / -Y
  Space: Arm 명령 정지(0,0,0)

Gripper (action: gripper):
  O : Open
  P : Pause(정지)
  C : Close

Camera (topic: camera/config):
  + / - : Digital Zoom 증/감 (0.0 ~ 2.0)
  Z     : Zoom 1.0으로 리셋

H : 도움말, Q : 종료
"""

def kbhit(timeout=0.01):
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    return dr

# 인터페이스 상수 (Goal 스코프에 정의됨)
PAUSE_CODE = getattr(GripperControl.Goal, 'PAUSE', 0)
OPEN_CODE  = getattr(GripperControl.Goal, 'OPEN',  1)
CLOSE_CODE = getattr(GripperControl.Goal, 'CLOSE', 2)

class RMKbExtra(Node):
    def __init__(self):
        super().__init__('rm_kb_extra')
        ns = self.get_namespace().strip('/')
        base = '' if ns == '' else f'/{ns}'

        # Arm: /<ns>/cmd_arm (Vector3)
        self.arm_pub = self.create_publisher(Vector3, f'{base}/cmd_arm', 10)

        # Gripper: action /<ns>/gripper (GripperControl)
        self.grip_ac = ActionClient(self, GripperControl, f'{base}/gripper')
        self.grip_power = 0.8  # 0~1 권장 파워 (원하면 조절)

        # Camera: /<ns>/camera/config (CameraConfig)
        self.cam_pub = self.create_publisher(CameraConfig, f'{base}/camera/config', 1)
        self.zoom = 1.0

        self.delta = 0.01  # m
        self.timer = self.create_timer(0.02, lambda: None)
        self.get_logger().info(HELP)
        self._keyboard_loop()

    def _send_arm(self, dx=0.0, dy=0.0, dz=0.0):
        msg = Vector3()
        msg.x, msg.y, msg.z = float(dx), float(dy), float(dz)
        self.arm_pub.publish(msg)

    def _grip_goal(self, target_state: int):
        if not self.grip_ac.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Gripper action server not available")
            return
        goal = GripperControl.Goal()
        # 인터페이스 정의: uint8 target_state, float32 power
        goal.target_state = int(target_state)
        goal.power = float(self.grip_power)
        self.grip_ac.send_goal_async(goal)

    def _send_zoom(self, zoom: float):
        cfg = CameraConfig()
        z = float(max(0.0, min(2.0, zoom)))
        cfg.zoom = z
        self.cam_pub.publish(cfg)

    def _keyboard_loop(self):
        old_attr = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while rclpy.ok():
                if kbhit():
                    ch = sys.stdin.read(1)
                    if ch in ('q','Q'):
                        break
                    elif ch in ('h','H'):
                        self.get_logger().info(HELP)

                    # Arm
                    elif ch in ('w','W'): self._send_arm(0, 0, +self.delta)
                    elif ch in ('s','S'): self._send_arm(0, 0, -self.delta)
                    elif ch in ('a','A'): self._send_arm(-self.delta, 0, 0)
                    elif ch in ('d','D'): self._send_arm(+self.delta, 0, 0)
                    elif ch in ('r','R'): self._send_arm(0, +self.delta, 0)
                    elif ch in ('f','F'): self._send_arm(0, -self.delta, 0)
                    elif ch == ' ':        self._send_arm(0, 0, 0)

                    # Gripper
                    elif ch in ('o','O'): self._grip_goal(OPEN_CODE)
                    elif ch in ('c','C'): self._grip_goal(CLOSE_CODE)
                    elif ch in ('p','P'): self._grip_goal(PAUSE_CODE)

                    # Camera
                    elif ch == '+':
                        self.zoom = min(2.0, self.zoom + 0.1)
                        self._send_zoom(self.zoom)
                        self.get_logger().info(f"Zoom: {self.zoom:.1f}")
                    elif ch == '-':
                        self.zoom = max(0.0, self.zoom - 0.1)
                        self._send_zoom(self.zoom)
                        self.get_logger().info(f"Zoom: {self.zoom:.1f}")
                    elif ch in ('z','Z'):
                        self.zoom = 1.0
                        self._send_zoom(self.zoom)
                        self.get_logger().info("Zoom reset to 1.0")

                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(0.01)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)

def main():
    rclpy.init()
    node = RMKbExtra()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
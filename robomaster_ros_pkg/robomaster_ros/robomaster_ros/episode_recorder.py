#!/usr/bin/env python3
import json
from datetime import datetime
from pathlib import Path
from collections import deque
from typing import Deque, Dict, IO, List, Optional, Sequence

import cv2
import rclpy
import yaml
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, Vector3
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import Image, JointState, Joy
from robomaster_msgs.msg import GripperState
from nav_msgs.msg import Odometry
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile


def stamp_to_float(stamp: object) -> Optional[float]:
    if stamp is None:
        return None
    if hasattr(stamp, 'nanoseconds'):
        return float(getattr(stamp, 'nanoseconds')) * 1e-9
    sec = getattr(stamp, 'sec', None)
    nanosec = getattr(stamp, 'nanosec', 0)
    if sec is None:
        return None
    return float(sec) + float(nanosec) * 1e-9


def normalize_sequence(raw: Sequence) -> List[str]:
    if isinstance(raw, str):
        return [raw]
    return [str(item) for item in raw] if isinstance(raw, Sequence) else [str(raw)]


class EpisodeRecorder(Node):
    def __init__(self) -> None:
        super().__init__('episode_recorder')
        self.bridge = CvBridge()

        self.session_root = Path(
            self.declare_parameter('session_root', '/tmp/CtrlWorldSessions').value
        ).expanduser()
        self.session_root.mkdir(parents=True, exist_ok=True)

        self.episode_prefix = self.declare_parameter('episode_prefix', 'episode').value
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.episode_dir = self.session_root / f'{self.episode_prefix}_{timestamp}'
        self.episode_dir.mkdir(parents=True, exist_ok=True)

        self.video_fps = float(self.declare_parameter('video_fps', 10.0).value)
        self.video_codec = str(self.declare_parameter('video_codec', 'mp4v').value or 'mp4v')
        self.write_config = bool(self.declare_parameter('write_config', True).value)
        declared_sample_hz = float(self.declare_parameter('sample_hz', 0.0).value)
        if declared_sample_hz <= 0.0:
            declared_sample_hz = self.video_fps if self.video_fps > 0.0 else 10.0
        self.sample_hz = declared_sample_hz
        self.sample_period = 1.0 / self.sample_hz if self.sample_hz > 0.0 else 0.0
        self.image_rate_log_interval = float(
            self.declare_parameter('image_rate_log_interval', 5.0).value
        )
        self.command_activity_epsilon = float(
            self.declare_parameter('command_activity_epsilon', 1e-3).value
        )
        self._image_rate_counter = 0
        self._image_rate_last_time: Optional[float] = None
        self._last_sample_time: Optional[float] = None

        self.image_topic = str(self.declare_parameter('image_topic', '/camera/image_color').value)
        self.cmd_vel_topic = str(self.declare_parameter('cmd_vel_topic', '/cmd_vel').value)
        self.cmd_arm_topic = str(self.declare_parameter('cmd_arm_topic', '/cmd_arm').value)
        self.joint_state_topic = str(self.declare_parameter('joint_state_topic', '/joint_states').value)
        self.command_topic = str(self.declare_parameter('command_topic', '/joy').value)
        self.arm_position_topic = str(self.declare_parameter('arm_position_topic', '/arm_position').value)
        self.arm_position_history_limit = int(self.declare_parameter('arm_position_history_limit', 500).value)
        self.gripper_topic = str(self.declare_parameter('gripper_topic', '/gripper').value)
        self.gripper_history_limit = int(self.declare_parameter('gripper_history_limit', 200).value)
        self.odom_topic = str(self.declare_parameter('odom_topic', '/odom').value)
        self.odom_history_limit = int(self.declare_parameter('odom_history_limit', 500).value)
        self.sample_history_limit = int(self.declare_parameter('sample_history_limit', 200).value)
        self.sync_tolerance = float(self.declare_parameter('sync_tolerance', 0.15).value)
        self.gripper_qos_profile = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.gripper_qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.latest_image_frame: Optional[object] = None
        self.latest_image_stamp: Optional[float] = None
        self.arm_position_history: Deque[Dict[str, object]] = deque(maxlen=self.arm_position_history_limit)
        self.gripper_history: Deque[Dict[str, object]] = deque(maxlen=self.gripper_history_limit)
        self.odom_history: Deque[Dict[str, object]] = deque(maxlen=self.odom_history_limit)
        self.cmd_vel_history: Deque[Dict[str, object]] = deque(maxlen=self.sample_history_limit)
        self.cmd_arm_history: Deque[Dict[str, object]] = deque(maxlen=self.sample_history_limit)
        self.command_history: Deque[Dict[str, object]] = deque(maxlen=self.sample_history_limit)
        self.joint_state_history: Deque[Dict[str, object]] = deque(maxlen=self.sample_history_limit)

        raw_filters = self.declare_parameter(
            'joint_name_filters', ['left', 'right', 'gripper']
        ).value
        self.joint_name_filters = normalize_sequence(raw_filters)

        self.video_writer: Optional[cv2.VideoWriter] = None
        self.video_path = self.episode_dir / 'rgb.mp4'
        self.frame_count = 0
        self.log_streams: Dict[str, IO[str]] = {}
        self.metadata = {
            'started_at': datetime.now().isoformat(),
            'episode_dir': str(self.episode_dir),
            'sample_file': str(self.episode_dir / 'samples.jsonl'),
            'topics': {
                'image': self.image_topic,
                'cmd_vel': self.cmd_vel_topic,
                'cmd_arm': self.cmd_arm_topic,
                'joint_state': self.joint_state_topic,
                'command': self.command_topic,
                'arm_position': self.arm_position_topic,
                'gripper': self.gripper_topic,
                'odom': self.odom_topic,
            },
            'video_fps': self.video_fps,
            'video_codec': self.video_codec,
            'sample_hz': self.sample_hz,
            'command_activity_epsilon': self.command_activity_epsilon,
        }

        if self.write_config:
            self._write_config_snapshot()

        self.create_subscription(Image, self.image_topic, self._on_image, 10)
        if self.cmd_vel_topic:
            self.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd_vel, 10)
        if self.cmd_arm_topic:
            self.create_subscription(Vector3, self.cmd_arm_topic, self._on_cmd_arm, 10)
        if self.joint_state_topic:
            self.create_subscription(JointState, self.joint_state_topic, self._on_joint_state, 10)
        if self.command_topic:
            self.create_subscription(Joy, self.command_topic, self._on_command, 10)
        if self.arm_position_topic:
            self.create_subscription(PointStamped, self.arm_position_topic, self._on_arm_position, 10)
        if self.gripper_topic:
            self.create_subscription(
                GripperState, self.gripper_topic, self._on_gripper_state, self.gripper_qos_profile
            )
        if self.odom_topic:
            self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)

        self.get_logger().info(f'Episode recorder writing to {self.episode_dir}')

    def _write_config_snapshot(self) -> None:
        config_snapshot = {
            'session_root': str(self.session_root),
            'episode_prefix': self.episode_prefix,
            'video_fps': self.video_fps,
            'video_codec': self.video_codec,
            'sample_hz': self.sample_hz,
            'sample_file': str(self.episode_dir / 'samples.jsonl'),
            'command_activity_epsilon': self.command_activity_epsilon,
            'image_topic': self.image_topic,
            'cmd_vel_topic': self.cmd_vel_topic,
            'cmd_arm_topic': self.cmd_arm_topic,
            'joint_state_topic': self.joint_state_topic,
            'joint_name_filters': self.joint_name_filters,
            'command_topic': self.command_topic,
            'arm_position_topic': self.arm_position_topic,
            'arm_position_history_limit': self.arm_position_history_limit,
            'gripper_topic': self.gripper_topic,
            'gripper_history_limit': self.gripper_history_limit,
            'odom_topic': self.odom_topic,
            'odom_history_limit': self.odom_history_limit,
        }
        with (self.episode_dir / 'config_used.yaml').open('w', encoding='utf-8') as file:
            yaml.safe_dump(config_snapshot, file)

    def _ensure_video_writer(self, frame) -> bool:
        if self.video_writer is not None:
            return True
        height, width = frame.shape[:2]
        fps = self.video_fps if self.video_fps > 0.0 else 1.0
        fourcc = cv2.VideoWriter_fourcc(*self.video_codec.upper())
        writer = cv2.VideoWriter(str(self.video_path), fourcc, fps, (width, height))
        if not writer.isOpened():
            self.get_logger().error(f'Failed to open video writer at {self.video_path}')
            return False
        self.video_writer = writer
        self.get_logger().info(f'Started RGB video recording -> {self.video_path}')
        return True

    def _on_image(self, msg: Image) -> None:
        now_time = self.get_clock().now().nanoseconds * 1e-9
        stamp = None
        header = getattr(msg, 'header', None)
        if header is not None:
            stamp = stamp_to_float(header.stamp)
        if stamp is None:
            stamp = now_time
        self._update_image_rate(now_time)
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'Failed to convert image: {exc}')
            return
        self.latest_image_frame = frame
        self.latest_image_stamp = stamp
        target_time = stamp if stamp is not None else now_time
        if not self._ready_for_sample(target_time):
            return
        self._record_sample(frame, stamp, target_time)

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._append_history_record(self.cmd_vel_history, msg)

    def _on_cmd_arm(self, msg: Vector3) -> None:
        self._append_history_record(self.cmd_arm_history, msg)

    def _on_joint_state(self, msg: JointState) -> None:
        filtered = self._filter_joint_state(msg)
        self._append_history_record(self.joint_state_history, msg, extra={'filtered': filtered})

    def _on_command(self, msg: Joy) -> None:
        self._append_history_record(self.command_history, msg)

    def _on_arm_position(self, msg: PointStamped) -> None:
        record = self._message_payload(msg)
        record['received_at'] = self.get_clock().now().nanoseconds * 1e-9
        self.arm_position_history.append(record)
        stream = self._get_log_stream('arm_position')
        stream.write(json.dumps(record, default=float))
        stream.write('\n')
        stream.flush()

    def _on_gripper_state(self, msg: GripperState) -> None:
        record = self._message_payload(msg)
        record['received_at'] = self.get_clock().now().nanoseconds * 1e-9
        self.gripper_history.append(record)
        stream = self._get_log_stream('gripper')
        stream.write(json.dumps(record, default=float))
        stream.write('\n')
        stream.flush()

    def _on_odom(self, msg: Odometry) -> None:
        record = self._message_payload(msg)
        record['received_at'] = self.get_clock().now().nanoseconds * 1e-9
        self.odom_history.append(record)
        stream = self._get_log_stream('odom')
        stream.write(json.dumps(record, default=float))
        stream.write('\n')
        stream.flush()

    def _update_image_rate(self, now_time: float) -> None:
        self._image_rate_counter += 1
        if self.image_rate_log_interval <= 0.0:
            return
        if self._image_rate_last_time is None:
            self._image_rate_last_time = now_time
            self._image_rate_counter = 0
            return
        elapsed = now_time - self._image_rate_last_time
        if elapsed >= self.image_rate_log_interval and elapsed > 0.0:
            hz = self._image_rate_counter / elapsed
            self.get_logger().info(f'Received {hz:.2f} Hz on {self.image_topic}')
            self._image_rate_last_time = now_time
            self._image_rate_counter = 0

    def _ready_for_sample(self, target_time: Optional[float]) -> bool:
        if target_time is None:
            return False
        if self.sample_period <= 0.0:
            return True
        if self._last_sample_time is None:
            return True
        return (target_time - self._last_sample_time) >= max(0.0, self.sample_period)

    def _record_sample(
        self,
        frame,
        image_stamp: Optional[float],
        sample_time: float,
    ) -> None:
        if not self._ensure_video_writer(frame):
            return
        self.video_writer.write(frame)
        self.frame_count += 1
        self._last_sample_time = sample_time
        frame_index = self.frame_count
        aligned_for_bundle: Dict[str, Dict[str, object]] = {}
        if self.cmd_vel_topic:
            record = self._align_history_record(self.cmd_vel_history, sample_time)
            if record is not None:
                aligned_for_bundle['cmd_vel'] = dict(record)
                self._log_topic_record('cmd_vel', self.cmd_vel_topic, record, sample_time)
        if self.cmd_arm_topic:
            record = self._align_history_record(self.cmd_arm_history, sample_time)
            if record is not None:
                aligned_for_bundle['cmd_arm'] = dict(record)
                self._log_topic_record('cmd_arm', self.cmd_arm_topic, record, sample_time)
        if self.joint_state_topic:
            record = self._align_history_record(self.joint_state_history, sample_time)
            if record is not None:
                self._log_topic_record('joint_states', self.joint_state_topic, record, sample_time)
        if self.command_topic:
            record = self._align_history_record(self.command_history, sample_time)
            if record is not None:
                aligned_for_bundle['commands'] = dict(record)
                self._log_topic_record('commands', self.command_topic, record, sample_time)
        arm_record = self._align_history_record(self.arm_position_history, sample_time)
        if arm_record is not None:
            aligned_for_bundle['arm_position'] = dict(arm_record)
        gripper_record = self._align_history_record(self.gripper_history, sample_time)
        if gripper_record is not None:
            aligned_for_bundle['gripper'] = dict(gripper_record)
        odom_record = self._align_history_record(self.odom_history, sample_time)
        if odom_record is not None:
            aligned_for_bundle['odom'] = dict(odom_record)
        self._log_sample_bundle(sample_time, frame_index, image_stamp, aligned_for_bundle)

    def _log_sample_bundle(
        self,
        sample_time: float,
        frame_index: int,
        image_stamp: Optional[float],
        aligned_records: Dict[str, Dict[str, object]],
    ) -> None:
        record: Dict[str, object] = {
            'sampled_at': sample_time,
            'frame_index': frame_index,
            'image_stamp': image_stamp,
        }
        for key in ('cmd_vel', 'cmd_arm', 'commands', 'arm_position', 'gripper', 'odom'):
            payload = aligned_records.get(key)
            if payload is not None:
                record[key] = payload
        stream = self._get_log_stream('samples')
        stream.write(json.dumps(record, default=float))
        stream.write('\n')
        stream.flush()

    def _filter_joint_state(self, msg: JointState) -> List[Dict[str, float]]:
        if not self.joint_name_filters:
            return []
        filtered: List[Dict[str, float]] = []
        for idx, name in enumerate(msg.name):
            if any(fragment in name for fragment in self.joint_name_filters) and 'wheel' not in name:
                entry: Dict[str, float] = {'name': name}
                if idx < len(msg.position):
                    entry['position'] = float(msg.position[idx])
                if idx < len(msg.velocity):
                    entry['velocity'] = float(msg.velocity[idx])
                if idx < len(msg.effort):
                    entry['effort'] = float(msg.effort[idx])
                filtered.append(entry)
        return filtered

    def _append_history_record(
        self,
        history: Deque[Dict[str, object]],
        msg,
        extra: Optional[Dict[str, object]] = None,
    ) -> Dict[str, object]:
        record = self._message_payload(msg)
        record['received_at'] = self.get_clock().now().nanoseconds * 1e-9
        if extra:
            record.update(extra)
        history.append(record)
        return record

    def _align_history_record(
        self,
        history: Deque[Dict[str, object]],
        target_stamp: Optional[float],
    ) -> Optional[Dict[str, object]]:
        if target_stamp is None or not history:
            return None
        best_record: Optional[Dict[str, object]] = None
        best_delta = float('inf')
        for entry in history:
            stamp = entry.get('stamp') or entry.get('received_at')
            if stamp is None:
                continue
            delta = abs(stamp - target_stamp)
            if delta < best_delta:
                best_delta = delta
                best_record = entry
        if best_record is None:
            return None
        if self.sync_tolerance > 0.0 and best_delta > self.sync_tolerance:
            return None
        return best_record

    def _log_topic_record(
        self,
        stream_name: str,
        topic: str,
        record: Dict[str, object],
        sample_time: Optional[float],
    ) -> None:
        log_entry = {'topic': topic}
        log_entry.update(record)
        if sample_time is not None:
            log_entry['sampled_at'] = sample_time
        stream = self._get_log_stream(stream_name)
        stream.write(json.dumps(log_entry, default=float))
        stream.write('\n')
        stream.flush()

    def _message_payload(self, msg) -> Dict[str, object]:
        header = getattr(msg, 'header', None)
        stamp_obj = None
        if header is not None and hasattr(header, 'stamp'):
            stamp_obj = header.stamp
        else:
            stamp_obj = getattr(msg, 'stamp', None)
        data = message_to_ordereddict(msg)
        if isinstance(msg, JointState):
            data = self._strip_wheel_joints(data)
        return {
            'stamp': stamp_to_float(stamp_obj),
            'data': data,
        }

    def _strip_wheel_joints(self, payload: Dict[str, object]) -> Dict[str, object]:
        names = payload.get('name')
        if not isinstance(names, list):
            return payload
        keep_indices = [
            idx for idx, name in enumerate(names)
            if 'wheel_joint' not in str(name) and 'wheel' not in str(name)
        ]
        if len(keep_indices) == len(names):
            return payload
        filtered = dict(payload)
        filtered['name'] = [names[idx] for idx in keep_indices]
        for field in ('position', 'velocity', 'effort'):
            values = payload.get(field)
            if isinstance(values, list):
                filtered[field] = [
                    values[idx] for idx in keep_indices
                    if idx < len(values)
                ]
        return filtered

    def _get_log_stream(self, name: str) -> IO[str]:
        if name in self.log_streams:
            return self.log_streams[name]
        path = self.episode_dir / f'{name}.jsonl'
        stream = path.open('a', encoding='utf-8')
        self.log_streams[name] = stream
        return stream

    def _close_resources(self) -> None:
        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None
            length = 0.0
            if self.video_fps > 0.0:
                length = self.frame_count / self.video_fps
            message = (
                f'RGB video saved -> name: {self.video_path.name}, '
                f'type: {self.video_codec}, frames: {self.frame_count}, '
                f'length: {length:.2f}s'
            )
            self.get_logger().info(message)
        for stream in self.log_streams.values():
            stream.flush()
            stream.close()
        self.log_streams.clear()
        self.metadata.update(
            {
                'ended_at': datetime.now().isoformat(),
                'frame_count': self.frame_count,
                'video_path': str(self.video_path),
            }
        )
        with (self.episode_dir / 'metadata.json').open('w', encoding='utf-8') as file:
            json.dump(self.metadata, file, indent=2)

    def destroy_node(self) -> None:
        self._close_resources()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EpisodeRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

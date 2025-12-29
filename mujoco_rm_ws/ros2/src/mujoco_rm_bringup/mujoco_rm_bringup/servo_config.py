from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import mujoco
import yaml


def _safe_id(model: mujoco.MjModel, obj_type: mujoco.mjtObj, name: str) -> Optional[int]:
    try:
        return int(mujoco.mj_name2id(model, obj_type, name))
    except ValueError:
        return None


def _clamp(val: float, lo: float, hi: float) -> float:
    return float(min(max(val, lo), hi))


@dataclass(frozen=True)
class ServoSpec:
    name: str
    actuator: str
    direction: int = 1
    reference_angle: float = 0.0  # joint angle at reference_ctrl [rad]
    reference_ctrl: float = 0.0  # actuator ctrl at reference_angle [rad]


@dataclass(frozen=True)
class ServoMapping:
    spec: ServoSpec
    actuator_id: int
    ctrl_lo: float
    ctrl_hi: float

    def angle_to_ctrl(self, angle: float) -> float:
        ctrl = self.spec.reference_ctrl + self.spec.direction * (angle - self.spec.reference_angle)
        return _clamp(ctrl, self.ctrl_lo, self.ctrl_hi)

    def ctrl_to_angle(self, ctrl: float) -> float:
        # Inverse of angle_to_ctrl (without clamping).
        return self.spec.reference_angle + self.spec.direction * (ctrl - self.spec.reference_ctrl)


def load_servo_mappings(model: mujoco.MjModel, yaml_path: Path) -> dict[str, ServoMapping]:
    """Load servo mappings keyed by `name` from a robomaster-style YAML list."""
    if not yaml_path.exists():
        raise FileNotFoundError(str(yaml_path))

    raw = yaml.safe_load(yaml_path.read_text())
    if raw is None:
        return {}
    if not isinstance(raw, list):
        raise ValueError("servos.yaml must be a YAML list of servo entries")

    mappings: dict[str, ServoMapping] = {}
    for entry in raw:
        if not isinstance(entry, dict):
            continue
        name = str(entry.get("name", "")).strip()
        actuator = str(entry.get("actuator", "")).strip()
        if not name or not actuator:
            continue
        direction = 1 if int(entry.get("direction", 1)) >= 0 else -1
        reference_angle = float(entry.get("reference_angle", entry.get("angle", 0.0)))
        reference_ctrl = float(entry.get("reference_ctrl", entry.get("ctrl", 0.0)))
        spec = ServoSpec(
            name=name,
            actuator=actuator,
            direction=direction,
            reference_angle=reference_angle,
            reference_ctrl=reference_ctrl,
        )
        act_id = _safe_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, spec.actuator)
        if act_id is None:
            continue
        lo, hi = map(float, model.actuator_ctrlrange[act_id])
        mappings[spec.name] = ServoMapping(spec=spec, actuator_id=act_id, ctrl_lo=lo, ctrl_hi=hi)
    return mappings


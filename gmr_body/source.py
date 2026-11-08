"""Convert explicitly described EVA Body poses to the old GMR skeleton.

This module does not establish calibration or produce robot commands.
"""

import json
import math
from pathlib import Path

import numpy as np

# The mapping is by name, never by slicing the 24-element input.
JOINT_MAP = json.loads((Path(__file__).parent / "resources" / "body_config.json").read_text(encoding="utf-8"))["joint_map"]
BODY_JOINTS = (
    "pelvis", "left_hip", "right_hip", "spine1", "left_knee", "right_knee",
    "spine2", "left_ankle", "right_ankle", "spine3", "left_foot", "right_foot",
    "neck", "left_collar", "right_collar", "head", "left_shoulder",
    "right_shoulder", "left_elbow", "right_elbow", "left_wrist", "right_wrist",
    "left_hand", "right_hand",
)


def _vector(value, length, label):
    if not isinstance(value, (tuple, list)) or len(value) != length:
        raise ValueError(f"{label} must have {length} elements")
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value):
        raise ValueError(f"{label} must contain finite numbers")
    return np.asarray(value, dtype=float)


def normalize_observation(observation, *, axis_map, units, joint_space, joint_map=None):
    """Normalize an EVA BodyObservation for offline inspection.

    Caller must establish the source conventions independently. axis_map is a
    proper rotation from source to GMR axes, not an inferred calibration.
    Returns global metres and wxyz quaternions; receive time remains receive time.
    """
    from scipy.spatial.transform import Rotation

    if units != "m" or joint_space != "global":
        raise ValueError("only explicitly declared global metre poses are supported")
    try:
        axis = np.asarray(axis_map, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError("axis_map must be a 3x3 proper rotation") from exc
    if (axis.shape != (3, 3) or not np.isfinite(axis).all()
            or not np.allclose(axis.T @ axis, np.eye(3), atol=1e-8, rtol=0)
            or not np.isclose(np.linalg.det(axis), 1.0, atol=1e-8, rtol=0)):
        raise ValueError("axis_map must be a 3x3 proper rotation")
    joint_map = JOINT_MAP if joint_map is None else joint_map
    if set(joint_map) != set(JOINT_MAP) or set(joint_map.values()) != set(JOINT_MAP.values()):
        raise ValueError("joint_map must map the 17 expected joints one-to-one")
    joints = observation.joints
    if tuple(j.name for j in joints) != BODY_JOINTS:
        raise ValueError("expected the exact 24 named EVA Body joints")
    poses = {}
    for joint in joints:
        if (isinstance(joint.location_flags, bool) or not isinstance(joint.location_flags, int)
                or joint.location_flags & 3 != 3):
            raise ValueError(f"{joint.name}: position and orientation must be valid")
        position = _vector(joint.position, 3, f"{joint.name}.position")
        quaternion = _vector(joint.orientation_xyzw, 4, f"{joint.name}.orientation_xyzw")
        norm = float(np.linalg.norm(quaternion))
        if norm < 1e-8:
            raise ValueError(f"{joint.name}: zero quaternion")
        if joint.name in joint_map:
            rotation = Rotation.from_quat(quaternion / norm).as_matrix()
            q = Rotation.from_matrix(axis @ rotation @ axis.T).as_quat()
            poses[joint_map[joint.name]] = ((axis @ position).tolist(), [q[3], q[0], q[1], q[2]])
    return {
        "poses": poses,
        "session_id": observation.session_id,
        "frame_seq": observation.frame_seq,
        "sample_sequence": observation.sample_sequence,
        "received_monotonic_ns": observation.received_monotonic_ns,
        "xr_predicted_display_time": observation.xr_predicted_display_time,
        "reference_space": observation.reference_space,
        "metadata": dict(observation.metadata),
    }

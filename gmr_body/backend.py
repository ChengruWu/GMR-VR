"""Experimental EVA Body to Luna solver using the upstream GMR engine."""

import json
import math
import os
import time
from pathlib import Path

import mink
import numpy as np
from mink.limits import Constraint, Limit
from scipy.spatial.transform import Rotation

from .source import normalize_observation

RESOURCES = Path(__file__).parent / "resources"


class _FreezeDofs(Limit):
    def __init__(self, model, names):
        indices = [int(model.joint(name).dofadr[0]) for name in names]
        eye = np.eye(model.nv)[indices]
        self.matrix = np.vstack((eye, -eye))

    def compute_qp_inequalities(self, configuration, dt):
        return Constraint(self.matrix, np.zeros(self.matrix.shape[0]))


class BodyBackend:
    def __init__(self, config_path=None):
        try:
            from ._gmr.motion_retarget import GeneralMotionRetargeting
        except ModuleNotFoundError as exc:
            if exc.name != "gmr_body._gmr":
                raise
            from general_motion_retargeting.motion_retarget import (
                GeneralMotionRetargeting,
            )

        path = Path(config_path or os.environ.get("GMR_BODY_CONFIG", RESOURCES / "body_config.json"))
        self.config = json.loads(path.read_text(encoding="utf-8"))
        for key in ("feedback_gripper_limit_tolerance_m", "feedback_joint_limit_tolerance_rad"):
            tolerance = self.config.get(key, 0.0)
            if (isinstance(tolerance, bool) or not isinstance(tolerance, (int, float))
                    or not math.isfinite(tolerance) or not 0 <= tolerance <= 0.001):
                raise ValueError(f"{key} must be between 0 and 0.001")
        enabled = self.config.get("relative_task_alignment", False)
        if not isinstance(enabled, bool):
            raise ValueError("relative_task_alignment must be boolean")
        scale = self.config.get("relative_position_scale", 0.5)
        if (isinstance(scale, bool) or not isinstance(scale, (int, float))
                or not math.isfinite(scale) or not 0 < scale <= 1):
            raise ValueError("relative_position_scale must be greater than 0 and at most 1")
        self.contract = json.loads((RESOURCES / "gento_luna" / "joint_limits.json").read_text(encoding="utf-8"))
        self.joints = sorted((r for r in self.contract if r["ctrl_index_24d"] is not None),
                             key=lambda r: r["ctrl_index_24d"])
        if [r["ctrl_index_24d"] for r in self.joints] != list(range(24)):
            raise ValueError("invalid EVA joint contract")
        self.engine = GeneralMotionRetargeting(
            "eva_body", "gento_luna", model_path=RESOURCES / "gento_luna" / "luna_full.xml",
            config_path=RESOURCES / "luna_ik.json", solver=self.config["solver"], verbose=False,
        )
        self.engine.max_iter = 1
        self.addresses = [int(self.engine.model.joint(r["vendor_name"]).qposadr[0]) for r in self.joints]
        fixed = [r["vendor_name"] for r in self.contract if r["ctrl_index_24d"] is None]
        fixed += ["base_x", "base_y", "base_yaw"]
        fixed += [self.joints[i]["vendor_name"] for i in (7, 15)]
        self.engine.ik_limits.append(_FreezeDofs(self.engine.model, fixed))
        self.reset()

    def reset(self):
        self._session = None
        self._sequence = None
        self._frame_seq = None
        self._reference_space = None
        self._transform = None
        self._relative_reference = None
        self._locked = False

    def close(self):
        self.reset()

    def _mapped_poses(self, poses, rotation, translation):
        return {name: [rotation.apply(poses[name][0]) + translation,
                       (rotation * Rotation.from_quat(poses[name][1], scalar_first=True)).as_quat(scalar_first=True)]
                for name in self.engine.human_scale_table}

    def _capture_relative_reference(self, mapped):
        if self.engine.use_ik_match_table2:
            raise ValueError("relative task alignment does not support a second task table")
        self.engine.update_targets(mapped)
        reference = {}
        for robot_name, row in self.engine.ik_match_table1.items():
            name = row[0]
            if name not in self.engine.human_body_to_task1:
                continue
            pos, quat = self.engine.scaled_human_data[name]
            body_id = self.engine.model.body(robot_name).id
            reference[name] = (
                np.asarray(pos).copy(), Rotation.from_quat(quat, scalar_first=True),
                self.engine.configuration.data.xpos[body_id].copy(),
                Rotation.from_quat(self.engine.configuration.data.xquat[body_id], scalar_first=True),
            )
        self._relative_reference = reference

    def _adjust_relative_targets(self):
        if self._relative_reference is None:
            raise ValueError("relative task reference is unavailable")
        scale = self.config.get("relative_position_scale", 0.5)
        for name, (source_pos, source_rot, robot_pos, robot_rot) in self._relative_reference.items():
            pos, quat = self.engine.scaled_human_data[name]
            adjusted_pos = robot_pos + scale * (np.asarray(pos) - source_pos)
            adjusted_quat = (Rotation.from_quat(quat, scalar_first=True) * source_rot.inv() * robot_rot).as_quat(scalar_first=True)
            self.engine.human_body_to_task1[name].set_target(
                mink.SE3.from_rotation_and_translation(mink.SO3(adjusted_quat), adjusted_pos)
            )
            self.engine.scaled_human_data[name] = [adjusted_pos, adjusted_quat]

    def step(self, request):
        from eva_body_contract import GmrResult, validate_feedback

        def fail(reason):
            self._locked = True
            return GmrResult("reset_required", reason=reason)

        if self._locked:
            return GmrResult("reset_required", reason="backend requires reset")
        try:
            obs, feedback = request.observation, request.feedback
            validate_feedback(feedback)
            if self.config.get("allow_unvalidated_output") is not True:
                return GmrResult("rejected", reason="experimental output disabled in Body configuration")
            if self.config.get("output_base_target") is True and feedback.base_pose_xyyaw is None:
                return fail("measured base pose is required for base target output")
            if obs.metadata.get("runtime_all_joint_locations_valid") is False:
                return fail("Body runtime reports invalid joint locations")
            for key, expected in (("units", self.config["source_units"]), ("joint_space", self.config["source_joint_space"]), ("orientation_order", "xyzw")):
                if obs.metadata.get(key) not in (None, expected):
                    return fail(f"Body metadata {key} conflicts with configuration")
            now = time.monotonic_ns()
            for stamp, maximum, label in (
                (obs.received_monotonic_ns, self.config["max_input_age_s"], "Body"),
                (feedback.received_monotonic_ns, self.config["max_feedback_age_s"], "feedback"),
            ):
                age = (now - stamp) / 1e9
                if age < 0 or age > maximum:
                    return fail(f"{label} reception time invalid or stale")
            if not obs.session_id or obs.sample_sequence is None or obs.reference_space is None:
                return fail("Body session, sequence and reference space are required")
            if self._session is not None and (obs.session_id != self._session or obs.reference_space != self._reference_space):
                return fail("Body session or reference space changed")
            if self._sequence is not None and (obs.sample_sequence < self._sequence or obs.frame_seq < self._frame_seq):
                return fail("Body sequence went backwards")
            if self._sequence == obs.sample_sequence:
                return GmrResult("idle")
            controls = request.controllers
            if not all(controls.get(side, {}).get("valid") is True and controls.get(side, {}).get("grip_engaged") is True for side in ("left", "right")):
                return fail("both controllers must be valid and grip-authorized")
            frame = normalize_observation(obs, axis_map=self.config["axis_map"], units=self.config["units"],
                                          joint_space=self.config["joint_space"], joint_map=self.config["joint_map"])
            q = self.engine.model.qpos0.copy()
            values = np.asarray(feedback.joint_positions, dtype=float)
            effective_limits = []
            model_values = values.copy()
            for i, row in enumerate(self.joints):
                jid = self.engine.model.joint(row["vendor_name"]).id
                lower, upper = row["lower"], row["upper"]
                if self.engine.model.jnt_limited[jid]:
                    lo, hi = self.engine.model.jnt_range[jid]
                    lower, upper = max(lower, lo), min(upper, hi)
                key = "feedback_gripper_limit_tolerance_m" if i in (7, 15) else "feedback_joint_limit_tolerance_rad"
                tolerance = max(1e-7, self.config.get(key, 0.0))
                if not lower - tolerance <= values[i] <= upper + tolerance:
                    return fail(f"feedback outside model limits: {row['clean_name']}")
                model_values[i] = np.clip(values[i], lower, upper)
                effective_limits.append((lower, upper))
            q[self.addresses] = model_values
            for side, index in (("left", 7), ("right", 15)):
                q[self.engine.model.joint(f"{side}_gripper_right_finger_joint").qposadr[0]] = model_values[index]
            if feedback.base_pose_xyyaw is not None:
                for name, value in zip(("base_x", "base_y", "base_yaw"), feedback.base_pose_xyyaw, strict=True):
                    q[self.engine.model.joint(name).qposadr[0]] = value
            self.engine.configuration.update(q=q)
            poses = frame["poses"]
            if self._transform is None:
                hips_pos, hips_quat = poses["Hips"]
                hips_rot = Rotation.from_quat(hips_quat, scalar_first=True)
                m = hips_rot.as_matrix()
                yaw = math.atan2(m[1, 0], m[0, 0])
                base_yaw = q[self.engine.model.joint("base_yaw").qposadr[0]]
                rotation = Rotation.from_euler("z", base_yaw - yaw)
                shoulders = np.mean([rotation.apply(poses[n][0]) for n in ("LeftShoulder", "RightShoulder")], axis=0)
                robot_z = np.mean([self.engine.configuration.data.xpos[self.engine.model.body(n).id, 2] for n in ("Link2_L", "Link2_R")])
                target = np.array([q[self.engine.model.joint("base_x").qposadr[0]], q[self.engine.model.joint("base_y").qposadr[0]], 0.0])
                target[2] = rotation.apply(hips_pos)[2] + robot_z - shoulders[2]
                root_offset = self.engine.pos_offsets1["Hips"]
                translation = target - (rotation * hips_rot).apply(root_offset) - rotation.apply(hips_pos)
                self._transform = (rotation, translation)
                if self.config.get("relative_task_alignment", False):
                    self._capture_relative_reference(self._mapped_poses(poses, rotation, translation))
                self._session, self._reference_space = obs.session_id, obs.reference_space
                self._sequence, self._frame_seq = obs.sample_sequence, obs.frame_seq
                return GmrResult("idle")
            rotation, translation = self._transform
            mapped = self._mapped_poses(poses, rotation, translation)
            start = time.monotonic()
            adjuster = self._adjust_relative_targets if self.config.get("relative_task_alignment", False) else None
            solved = self.engine.retarget(mapped, target_adjuster=adjuster)
            if time.monotonic() - start > self.config["max_solve_s"]:
                return fail("solver exceeded time budget")
            now = time.monotonic_ns()
            if ((now - obs.received_monotonic_ns) / 1e9 > self.config["max_input_age_s"] or
                    (now - feedback.received_monotonic_ns) / 1e9 > self.config["max_feedback_age_s"]):
                return fail("input or feedback expired during solve")
            if not np.isfinite(solved).all():
                return fail("solver produced non-finite qpos")
            output = np.asarray(solved[self.addresses], dtype=float)
            for side, index in (("left", 7), ("right", 15)):
                trigger = controls[side].get("trigger")
                if isinstance(trigger, bool) or not isinstance(trigger, (float, int)) or not math.isfinite(trigger) or not 0 <= trigger <= 1:
                    return fail(f"invalid {side} trigger")
                output[index] = self.config["gripper_trigger_zero_m"] + trigger * (self.config["gripper_trigger_one_m"] - self.config["gripper_trigger_zero_m"])
            for i, row in enumerate(self.joints):
                delta = self.config["max_gripper_delta_m" if i in (7, 15) else "max_joint_delta_rad"]
                lower, upper = effective_limits[i]
                lower, upper = max(lower, values[i] - delta), min(upper, values[i] + delta)
                if lower > upper:
                    return fail(f"no valid target within step limit: {row['clean_name']}")
                output[i] = np.clip(output[i], lower, upper)
            self._sequence, self._frame_seq = obs.sample_sequence, obs.frame_seq
            base_target = None
            if self.config.get("output_base_target") is True:
                position, quaternion = self.engine.scaled_human_data["Hips"]
                matrix = Rotation.from_quat(quaternion, scalar_first=True).as_matrix()
                base_target = (float(position[0]), float(position[1]),
                               math.atan2(matrix[1, 0], matrix[0, 0]))
            return GmrResult("ok", {row["clean_name"]: float(output[i]) for i, row in enumerate(self.joints)},
                             base_target_xyyaw=base_target)
        except Exception as error:
            return fail(f"invalid input or solve failure: {error}")

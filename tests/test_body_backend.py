import time
from dataclasses import replace

import numpy as np
import pytest

from gmr_body import create_backend

contract = pytest.importorskip("eva_body_contract")


def make_request(backend, sequence):
    # Synthetic poses are only for exercising the interface and solver.
    positions = {"pelvis": (0, 1, 0), "spine3": (0, 1.4, 0), "head": (0, 1.7, 0),
                 "left_shoulder": (-0.2, 1.4, 0), "right_shoulder": (0.2, 1.4, 0),
                 "left_elbow": (-0.4, 1.2, 0), "right_elbow": (0.4, 1.2, 0),
                 "left_wrist": (-0.5, 1, 0), "right_wrist": (0.5, 1, 0)}
    joints = tuple(contract.BodyJoint(n, 3, positions.get(n, (0, 1, 0)), (0, 0, 0, 1))
                   for n in contract.BODY_JOINTS)
    stamp = time.monotonic_ns()
    obs = contract.BodyObservation("test", sequence, stamp, sequence, None, "local", "tracking", joints,
                                   {"units": "metres", "joint_space": "base_space", "orientation_order": "xyzw", "runtime_all_joint_locations_valid": True})
    q = backend.engine.model.qpos0[backend.addresses]
    feedback = contract.RobotFeedback(tuple(float(x) for x in q), stamp)
    controllers = {s: {"valid": True, "grip_engaged": True, "trigger": 0.0} for s in ("left", "right")}
    return contract.GmrRequest(obs, feedback, controllers)


def test_body_backend_uses_feedback_and_rejects_reset_conditions():
    backend = create_backend()
    backend.config["max_solve_s"] = 10
    backend.config["max_input_age_s"] = backend.config["max_feedback_age_s"] = 10
    assert backend.step(make_request(backend, 0)).status == "rejected"
    backend.config["allow_unvalidated_output"] = True
    first = make_request(backend, 1)
    assert backend.step(first).status == "idle"
    second = make_request(backend, 2)
    result = backend.step(second)
    assert result.status == "ok", result.reason
    ordered = contract.ordered_targets(result)
    assert len(ordered) == 24
    assert np.isfinite(ordered).all()
    assert np.max(np.abs(np.asarray(ordered) - second.feedback.joint_positions)) <= 0.050001
    for i, row in enumerate(backend.joints):
        jid = backend.engine.model.joint(row["vendor_name"]).id
        if backend.engine.model.jnt_limited[jid]:
            low, high = backend.engine.model.jnt_range[jid]
            assert low <= ordered[i] <= high
    next_request = make_request(backend, 3)
    next_request = replace(next_request, feedback=replace(next_request.feedback, joint_positions=ordered))
    assert backend.step(next_request).status == "ok"
    assert backend.step(next_request).status == "idle"
    changed = replace(next_request.observation, session_id="other", sample_sequence=4)
    assert backend.step(replace(next_request, observation=changed)).status == "reset_required"
    assert backend.step(make_request(backend, 4)).status == "reset_required"
    backend.reset()
    assert backend.step(make_request(backend, 5)).status == "idle"


def test_body_backend_rejects_stale_feedback_and_lost_grip():
    backend = create_backend()
    backend.config["allow_unvalidated_output"] = True
    request = make_request(backend, 1)
    old_feedback = replace(request.feedback, received_monotonic_ns=time.monotonic_ns() - 2_000_000_000)
    assert backend.step(replace(request, feedback=old_feedback)).status == "reset_required"
    backend.reset()
    controllers = dict(request.controllers)
    controllers["left"] = {"valid": True, "grip_engaged": False, "trigger": 0.0}
    assert backend.step(replace(make_request(backend, 2), controllers=controllers)).status == "reset_required"


def test_gripper_soft_limit_tolerance_is_opt_in_and_bounded():
    backend = create_backend()
    backend.config.update(allow_unvalidated_output=True, max_input_age_s=10, max_feedback_age_s=10)

    def with_gripper(value, sequence):
        request = make_request(backend, sequence)
        qpos = list(request.feedback.joint_positions)
        qpos[7] = value
        return replace(request, feedback=replace(request.feedback, joint_positions=tuple(qpos)))

    assert backend.step(with_gripper(-0.0005, 1)).status == "reset_required"
    backend.reset()
    backend.config["feedback_gripper_limit_tolerance_m"] = 0.001
    assert backend.step(with_gripper(-0.0005, 2)).status == "idle"
    assert backend.engine.configuration.q[backend.addresses[7]] == 0.0
    assert backend.step(with_gripper(-0.0011, 3)).status == "reset_required"


def test_simulation_torso_feedback_is_accepted():
    backend = create_backend()
    backend.config.update(allow_unvalidated_output=True, max_input_age_s=10, max_feedback_age_s=10)
    request = make_request(backend, 1)
    qpos = list(request.feedback.joint_positions)
    qpos[16] = 0.0054  # Observed simulation feedback, inside the EVA model range.
    qpos[20] = 0.15  # Inside EVA's range but outside the old experimental window.
    request = replace(request, feedback=replace(request.feedback, joint_positions=tuple(qpos)))
    assert backend.step(request).status == "idle"


def test_feedback_tolerance_cannot_override_step_limit():
    backend = create_backend()
    backend.config.update(allow_unvalidated_output=True, max_input_age_s=10, max_feedback_age_s=10,
                          max_solve_s=10, feedback_gripper_limit_tolerance_m=0.001,
                          max_gripper_delta_m=0.0001)

    def request_with_gripper(sequence):
        request = make_request(backend, sequence)
        qpos = list(request.feedback.joint_positions)
        qpos[7] = -0.0005
        return replace(request, feedback=replace(request.feedback, joint_positions=tuple(qpos)))

    assert backend.step(request_with_gripper(1)).status == "idle"
    result = backend.step(request_with_gripper(2))
    assert result.status == "reset_required"
    assert "no valid target within step limit" in result.reason


def test_joint_soft_limit_tolerance_is_opt_in_and_bounded():
    backend = create_backend()
    backend.config.update(allow_unvalidated_output=True, max_input_age_s=10, max_feedback_age_s=10)
    index = 13  # joint6_r
    joint = backend.engine.model.joint(backend.joints[index]["vendor_name"])
    lower = max(backend.joints[index]["lower"], backend.engine.model.jnt_range[joint.id][0])

    def with_value(value, sequence):
        sample = make_request(backend, sequence)
        qpos = list(sample.feedback.joint_positions)
        qpos[index] = value
        return replace(sample, feedback=replace(sample.feedback, joint_positions=tuple(qpos)))

    assert backend.step(with_value(lower - 0.0005, 1)).status == "reset_required"
    backend.reset()
    backend.config["feedback_joint_limit_tolerance_rad"] = 0.001
    assert backend.step(with_value(lower - 0.0005, 2)).status == "idle"
    assert backend.engine.configuration.q[backend.addresses[index]] == lower
    assert backend.step(with_value(lower - 0.0011, 3)).status == "reset_required"


def test_relative_task_alignment_anchors_neutral_and_responds_to_motion():
    backend = create_backend()
    backend.config.update(allow_unvalidated_output=True, relative_task_alignment=True,
                          relative_position_scale=0.5, max_solve_s=10,
                          max_input_age_s=10, max_feedback_age_s=10)
    first = make_request(backend, 1)
    assert backend.step(first).status == "idle"
    still = make_request(backend, 2)
    result = backend.step(still)
    assert result.status == "ok", result.reason
    np.testing.assert_allclose(contract.ordered_targets(result), still.feedback.joint_positions, atol=1e-6)
    moved = make_request(backend, 3)
    joints = tuple(replace(j, position=(j.position[0], j.position[1] + 0.2, j.position[2]))
                   if j.name == "left_wrist" else j for j in moved.observation.joints)
    moved = replace(moved, observation=replace(moved.observation, joints=joints))
    result = backend.step(moved)
    assert result.status == "ok", result.reason
    assert np.linalg.norm(np.asarray(contract.ordered_targets(result))[:7] - moved.feedback.joint_positions[:7]) > 1e-4
    backend.reset()
    assert backend._relative_reference is None
    assert backend.step(make_request(backend, 4)).status == "idle"


def test_base_target_tracks_human_translation_from_measured_reference():
    backend = create_backend()
    backend.config.update(allow_unvalidated_output=True, output_base_target=True,
                          max_solve_s=10, max_input_age_s=10, max_feedback_age_s=10)
    first = make_request(backend, 1)
    assert backend.step(first).status == "reset_required"
    backend.reset()
    first = replace(first, feedback=replace(first.feedback, base_pose_xyyaw=(1.0, 2.0, 0.0)))
    assert backend.step(first).status == "idle"
    second = make_request(backend, 2)
    moved = tuple(replace(j, position=(j.position[0], j.position[1], j.position[2] - 0.1))
                  for j in second.observation.joints)
    second = replace(second, observation=replace(second.observation, joints=moved),
                     feedback=replace(second.feedback, base_pose_xyyaw=(1.0, 2.0, 0.0)))
    result = backend.step(second)
    assert result.status == "ok", result.reason
    np.testing.assert_allclose(result.base_target_xyyaw, (1.1, 2.0, 0.0), atol=1e-6)

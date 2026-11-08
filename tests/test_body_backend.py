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
    assert backend.step(second).status == "idle"
    changed = replace(second.observation, session_id="other", sample_sequence=3)
    assert backend.step(replace(second, observation=changed)).status == "reset_required"
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

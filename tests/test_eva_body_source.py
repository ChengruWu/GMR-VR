import unittest
from types import SimpleNamespace

import numpy as np

from gmr_body.source import BODY_JOINTS, normalize_observation


def observation():
    joints = tuple(SimpleNamespace(name=name, location_flags=3, position=(1, 2, 3),
                                   orientation_xyzw=(0, 0, 0, 1)) for name in BODY_JOINTS)
    return SimpleNamespace(joints=joints, session_id="s", frame_seq=2, sample_sequence=1,
                           received_monotonic_ns=10, xr_predicted_display_time=20,
                           reference_space="local", metadata={"source": "test"})


class BodySourceTest(unittest.TestCase):
    def test_mapping_and_axis_conversion(self):
        axis = [[0, 0, -1], [-1, 0, 0], [0, 1, 0]]
        result = normalize_observation(observation(), axis_map=axis, units="m", joint_space="global")
        self.assertEqual(len(result["poses"]), 17)
        np.testing.assert_allclose(result["poses"]["Hips"][0], [-3, -1, 2])
        np.testing.assert_allclose(result["poses"]["Head"][1], [1, 0, 0, 0])
        self.assertEqual(result["received_monotonic_ns"], 10)
        self.assertEqual(result["xr_predicted_display_time"], 20)

    def test_rejects_bad_tracking_and_conventions(self):
        frame = observation()
        frame.joints[0].location_flags = 1
        with self.assertRaises(ValueError):
            normalize_observation(frame, axis_map=np.eye(3), units="m", joint_space="global")
        frame = observation()
        frame.joints[3].orientation_xyzw = (0, 0, 0, 0)
        with self.assertRaises(ValueError):
            normalize_observation(frame, axis_map=np.eye(3), units="m", joint_space="global")
        with self.assertRaises(ValueError):
            normalize_observation(observation(), axis_map=np.diag([-1, 1, 1]), units="m", joint_space="global")
        with self.assertRaises(ValueError):
            normalize_observation(observation(), axis_map=np.eye(3), units="m", joint_space="parent_local")

    def test_rejects_reordered_joints(self):
        frame = observation()
        frame.joints = frame.joints[::-1]
        with self.assertRaises(ValueError):
            normalize_observation(frame, axis_map=np.eye(3), units="m", joint_space="global")


if __name__ == "__main__":
    unittest.main()

"""Sanity checks on the canonical joint ordering."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from skate_ros2 import names  # noqa: E402


def test_counts():
    assert names.N_JOINTS == 26
    assert len(names.JOINT_NAMES) == 26
    assert len(set(names.JOINT_NAMES)) == 26
    assert len(names.CAN_LAYOUT) == 26


def test_urdf_suffix_encodes_protocol_index():
    """skt_v3.urdf joint names end in the global protocol index."""
    for i, name in enumerate(names.JOINT_NAMES):
        suffix = name.rsplit("a", 1)[-1]
        assert int(suffix) == i, (name, i)


def test_can_layout():
    # bus sizes 4,4,8,8,2 in order
    assert names.CAN_LAYOUT[0] == (0, 0)
    assert names.CAN_LAYOUT[7] == (1, 3)
    assert names.CAN_LAYOUT[8] == (2, 0)
    assert names.CAN_LAYOUT[15] == (2, 7)
    assert names.CAN_LAYOUT[16] == (3, 0)
    assert names.CAN_LAYOUT[23] == (3, 7)
    assert names.CAN_LAYOUT[24] == (4, 0)
    assert names.CAN_LAYOUT[25] == (4, 1)


def test_vector_can_dict_roundtrip():
    vec = [float(i) for i in range(26)]
    d = names.vector_to_can_dict(vec)
    assert d[2][3] == 11.0          # left elbow = protocol index 11
    assert d[4] == [24.0, 25.0]
    back = names.can_dict_to_vector(d)
    assert back == vec


def test_default_pose_matches_official_docs():
    import math
    assert names.DEFAULT_POSE[11] == math.radians(90)
    assert names.DEFAULT_POSE[19] == math.radians(90)
    assert names.DEFAULT_POSE[15] == math.radians(20)
    assert names.DEFAULT_POSE[23] == math.radians(20)
    assert sum(abs(x) for i, x in enumerate(names.DEFAULT_POSE)
               if i not in (11, 15, 19, 23)) == 0.0


if __name__ == "__main__":
    for f in [test_counts, test_urdf_suffix_encodes_protocol_index,
              test_can_layout, test_vector_can_dict_roundtrip,
              test_default_pose_matches_official_docs]:
        f()
        print(f"PASS {f.__name__}")

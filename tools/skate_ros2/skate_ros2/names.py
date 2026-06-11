"""Canonical Skate (skt_v3) joint naming and UDP protocol ordering.

The wire protocol orders the 26 DoF as:
    [0:4]   left leg   (CAN bus 0)
    [4:8]   right leg  (CAN bus 1)
    [8:16]  left arm   (CAN bus 2)
    [16:24] right arm  (CAN bus 3)
    [24:26] head       (CAN bus 4)

The official URDF (Rbotic/skate_teleop, skt_v3.urdf) encodes the global
protocol index directly in each joint name suffix (e.g. ``a3_armL_a11`` is
protocol index 11), so the mapping below is exact, not guessed.

No external dependencies — safe to import anywhere.
"""

N_JOINTS = 26

# Exact URDF joint names, in wire-protocol order (index i == protocol index i).
JOINT_NAMES = (
    # lower chain (legs / base linkage), protocol 0..7
    "a0", "a1", "a2", "a3", "a4", "a5", "a6", "a7",
    # left arm, protocol 8..15 (15 = gripper)
    "a0_armL_a8", "a1_armL_a9", "a2_armL_a10", "a3_armL_a11",
    "a4_armL_a12", "a5_armL_a13", "a6_armL_a14", "a7_armL_a15",
    # right arm, protocol 16..23 (23 = gripper)
    "a0_armR_a16", "a1_armR_a17", "a2_armR_a18", "a3_armR_a19",
    "a4_armR_a20", "a5_armR_a21", "a6_armR_a22", "a7_armR_a23",
    # head, protocol 24..25
    "a0_head_a24", "a1_head_a25",
)

INDEX = {name: i for i, name in enumerate(JOINT_NAMES)}

# Convenient slices into the 26-vector.
LEFT_LEG = slice(0, 4)
RIGHT_LEG = slice(4, 8)
LOWER_CHAIN = slice(0, 8)
LEFT_ARM = slice(8, 16)
RIGHT_ARM = slice(16, 24)
HEAD = slice(24, 26)
LEFT_GRIPPER = 15
RIGHT_GRIPPER = 23

# CAN bus layout used by the firmware telemetry dicts:
# protocol index i lives at motor_pos[CAN_LAYOUT[i][0]][CAN_LAYOUT[i][1]].
_BUS_SIZES = (4, 4, 8, 8, 2)
CAN_LAYOUT = []
for _bus, _n in enumerate(_BUS_SIZES):
    for _slot in range(_n):
        CAN_LAYOUT.append((_bus, _slot))
CAN_LAYOUT = tuple(CAN_LAYOUT)

# Safe default pose from the official docs (elbows bent, grippers slightly open).
import math as _math

DEFAULT_POSE = [0.0] * N_JOINTS
DEFAULT_POSE[11] = _math.radians(90)   # left elbow
DEFAULT_POSE[15] = _math.radians(20)   # left gripper
DEFAULT_POSE[19] = _math.radians(90)   # right elbow
DEFAULT_POSE[23] = _math.radians(20)   # right gripper
DEFAULT_POSE = tuple(DEFAULT_POSE)


def vector_to_can_dict(vec, template=None):
    """Scatter a 26-vector into a firmware-style ``{bus: [..]}`` dict."""
    d = {0: [0.0] * 4, 1: [0.0] * 4, 2: [0.0] * 8, 3: [0.0] * 8, 4: [0.0] * 2}
    for i in range(N_JOINTS):
        bus, slot = CAN_LAYOUT[i]
        d[bus][slot] = float(vec[i])
    return d


def can_dict_to_vector(d):
    """Gather a firmware-style ``{bus: [..]}`` dict into a flat 26-list."""
    out = [0.0] * N_JOINTS
    for i in range(N_JOINTS):
        bus, slot = CAN_LAYOUT[i]
        try:
            v = d[bus][slot]
        except (KeyError, IndexError, TypeError):
            v = 0.0
        out[i] = float(v) if v is not None else 0.0
    return out

"""Virtual-obstacle augmentation of the collision guard. The geometry helper
is unit-tested standalone (no mujoco); the guard integration places a box on
the robot and checks it blocks, then clears.

    SKT_DIR=.../skt_v3 python -m pytest test/test_obstacles.py
"""

import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "skate_ros2"))

from skate_commander.server import _obstacle_hit   # noqa: E402

SKT = Path(os.environ.get("SKT_DIR", "/tmp/skate_teleop/skt_v3"))
CXML = SKT / "skt_v3_collision.xml"


def test_obstacle_hit_geometry():
    box = {"type": "box", "p": [0, 0, 0], "s": [0.1, 0.1, 0.1]}
    assert _obstacle_hit([0, 0, 0], 0.02, box)             # geom centre inside the box
    assert _obstacle_hit([0.11, 0, 0], 0.02, box)          # 1 cm past the face, within radius
    assert not _obstacle_hit([0.25, 0, 0], 0.02, box)      # clearly clear
    cyl = {"type": "cyl", "p": [0, 0, 0], "s": [0.1, 0.2]}  # radius 0.1, half-height 0.2
    assert _obstacle_hit([0.06, 0.06, 0.1], 0.02, cyl)     # inside radius + height
    assert not _obstacle_hit([0.4, 0, 0], 0.02, cyl)       # outside radially
    assert not _obstacle_hit([0, 0, 0.4], 0.02, cyl)       # above the cap
    assert not _obstacle_hit([0, 0, 0], 0.02, {"type": "box"})  # missing p/s → no hit


def test_guard_blocks_a_box_on_the_robot():
    try:
        import mujoco  # noqa: F401
    except ImportError:
        print("SKIP: mujoco not installed"); return
    if not CXML.exists():
        print("SKIP: no collision model"); return
    from skate_commander.server import make_collision_guard

    obstacles = []
    guard = make_collision_guard(CXML, get_obstacles=lambda: obstacles)
    neutral = np.zeros(26)
    assert not guard(neutral), "no obstacles → neutral pose allowed"

    geoms = guard.collision_view(neutral)
    assert geoms, "collision_view should return robot geoms"
    p = geoms[len(geoms) // 2]["p"]                        # a mid-chain geom centre
    obstacles.append({"id": 1, "type": "box", "p": list(p), "s": [0.05, 0.05, 0.05]})
    assert guard(neutral), "a box placed on the robot must be detected"

    obstacles.clear()
    assert not guard(neutral), "clearing obstacles unblocks again"
    print("PASS virtual box blocks + clears")

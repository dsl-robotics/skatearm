import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from skate_commander.urdf import joint_limits, parse_urdf  # noqa: E402

URDF = Path(os.environ.get("SKT_DIR", "/tmp/skate_teleop/skt_v3")) / "skt_v3.urdf"



def _skip(msg):
    """Real pytest.skip under pytest; clean print when run as a standalone script."""
    import sys
    if "pytest" in sys.modules:
        import pytest
        pytest.skip(msg)
    print(f"SKIP: {msg}")


def test_parse_tree():
    if not URDF.exists():
        _skip(f"{URDF} missing (run sim/make_control_model.py)"); return
    m = parse_urdf(URDF)
    assert len(m["joint_names"]) == 26
    indexed = [j for j in m["joints"] if j["index"] is not None]
    assert len(indexed) == 26
    assert {j["index"] for j in indexed} == set(range(26))
    fixed = [j for j in m["joints"] if j["type"] == "fixed"]
    assert all(j["index"] is None for j in fixed)
    assert len(m["mesh_files"]) > 10
    assert all(("/" not in f and "\\" not in f) for f in m["mesh_files"])


def test_limits():
    if not URDF.exists():
        _skip(f"{URDF} missing (run sim/make_control_model.py)"); return
    m = parse_urdf(URDF)
    lo, hi = joint_limits(m)
    assert len(lo) == 26 and len(hi) == 26
    assert all(l < h for l, h in zip(lo, hi))
    assert hi[11] > 2.0           # left elbow range from the official URDF


if __name__ == "__main__":
    test_parse_tree(); print("PASS test_parse_tree")
    test_limits(); print("PASS test_limits")

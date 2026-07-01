"""SRDF <-> URDF consistency for the MoveIt config (structural validation, no
ROS 2 runtime). Every link/joint the SRDF references must exist in the skt_v3
URDF, and the arm groups/controllers must match the protocol joint map in
skate_ros2.names. Skips if the URDF isn't present.

    SKT_DIR=.../skt_v3 python test_srdf.py
"""
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
SRDF = PKG / "config" / "skate.srdf"
SKT = Path(os.environ.get("SKT_DIR", "/tmp/skate_teleop/skt_v3"))
sys.path.insert(0, str(PKG.parent / "skate_ros2"))     # for skate_ros2.names


def _skip(msg):
    if "pytest" in sys.modules:
        import pytest
        pytest.skip(msg)
    print(f"SKIP: {msg}")


def _urdf_names():
    root = ET.parse(SKT / "skt_v3.urdf").getroot()
    links = {l.get("name") for l in root.findall("link")}
    joints = {j.get("name") for j in root.findall("joint")}
    return links, joints


def test_srdf_references_resolve_against_urdf():
    if not (SKT / "skt_v3.urdf").exists():
        _skip("no skt_v3.urdf (set SKT_DIR)"); return
    links, joints = _urdf_names()
    srdf = ET.parse(SRDF).getroot()
    urdf_name = ET.parse(SKT / "skt_v3.urdf").getroot().get("name")
    assert srdf.get("name") == urdf_name, \
        f"SRDF robot '{srdf.get('name')}' must match URDF robot '{urdf_name}' (MoveIt requires it)"
    for g in srdf.findall("group"):
        for ch in g.findall("chain"):
            assert ch.get("base_link") in links, ch.get("base_link")
            assert ch.get("tip_link") in links, ch.get("tip_link")
        for j in g.findall("joint"):
            assert j.get("name") in joints, j.get("name")
    for gs in srdf.findall("group_state"):
        for j in gs.findall("joint"):
            assert j.get("name") in joints, j.get("name")
    for ee in srdf.findall("end_effector"):
        assert ee.get("parent_link") in links, ee.get("parent_link")
    for vj in srdf.findall("virtual_joint"):
        assert vj.get("child_link") in links, vj.get("child_link")
    n_acm = 0
    for dc in srdf.findall("disable_collisions"):
        assert dc.get("link1") in links and dc.get("link2") in links, \
            (dc.get("link1"), dc.get("link2"))
        n_acm += 1
    print(f"PASS SRDF<->URDF: chains/joints/states/ee/vj + {n_acm} ACM pairs all resolve")


def test_groups_match_protocol_map():
    if not (SKT / "skt_v3.urdf").exists():
        _skip("no skt_v3.urdf"); return
    from skate_ros2 import names
    left = [names.JOINT_NAMES[i] for i in range(8, 15)]     # protocol 8..14
    right = [names.JOINT_NAMES[i] for i in range(16, 23)]   # protocol 16..22
    ctrl = (PKG / "config" / "moveit_controllers.yaml").read_text()
    for j in left + right:
        assert j in ctrl, f"controller yaml missing {j}"
    assert names.JOINT_NAMES[15] == "a7_armL_a15"
    assert names.JOINT_NAMES[23] == "a7_armR_a23"
    # the SRDF gripper groups use exactly the gripper joints
    srdf = ET.parse(SRDF).getroot()
    grip_joints = {j.get("name") for g in srdf.findall("group")
                   if g.get("name").endswith("gripper") for j in g.findall("joint")}
    assert grip_joints == {"a7_armL_a15", "a7_armR_a23"}, grip_joints
    print("PASS groups/controllers/grippers match names.py protocol map")


if __name__ == "__main__":
    test_srdf_references_resolve_against_urdf()
    test_groups_match_protocol_map()

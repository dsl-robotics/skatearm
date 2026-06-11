"""Parse skt_v3.urdf into a JSON-ready kinematic tree for the browser viewer.

The frontend builds a THREE.Group per link and rotates joint groups around
their URDF axes — no URDF library needed on either side.
"""

from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

_SKATE_ROS2 = Path(__file__).resolve().parents[2] / "skate_ros2"
if str(_SKATE_ROS2) not in sys.path:
    sys.path.insert(0, str(_SKATE_ROS2))

from skate_ros2 import names  # noqa: E402


def _floats(s, default):
    if not s:
        return list(default)
    return [float(x) for x in s.split()]


def parse_urdf(urdf_path):
    """Return {links: {...}, joints: [...], mesh_files: [...]} for /api/model."""
    root = ET.parse(str(urdf_path)).getroot()

    links = {}
    mesh_files = []
    for link in root.findall("link"):
        lname = link.get("name")
        visuals = []
        for vis in link.findall("visual"):
            origin = vis.find("origin")
            xyz = _floats(origin.get("xyz") if origin is not None else None,
                          (0, 0, 0))
            rpy = _floats(origin.get("rpy") if origin is not None else None,
                          (0, 0, 0))
            mesh = vis.find("geometry/mesh")
            if mesh is None:
                continue
            fname = Path(mesh.get("filename")).name   # basename only
            scale = _floats(mesh.get("scale"), (1, 1, 1))
            color = None
            cel = vis.find("material/color")
            if cel is not None:
                color = _floats(cel.get("rgba"), (0.7, 0.7, 0.7, 1.0))
            visuals.append({"mesh": fname, "xyz": xyz, "rpy": rpy,
                            "scale": scale, "color": color})
            mesh_files.append(fname)
        links[lname] = {"visuals": visuals}

    joints = []
    for joint in root.findall("joint"):
        jname = joint.get("name")
        jtype = joint.get("type")
        origin = joint.find("origin")
        limit = joint.find("limit")
        axis = joint.find("axis")
        joints.append({
            "name": jname,
            "type": jtype,
            "parent": joint.find("parent").get("link"),
            "child": joint.find("child").get("link"),
            "xyz": _floats(origin.get("xyz") if origin is not None else None,
                           (0, 0, 0)),
            "rpy": _floats(origin.get("rpy") if origin is not None else None,
                           (0, 0, 0)),
            "axis": _floats(axis.get("xyz") if axis is not None else None,
                            (0, 0, 1)),
            "lower": float(limit.get("lower")) if limit is not None else None,
            "upper": float(limit.get("upper")) if limit is not None else None,
            "index": names.INDEX.get(jname),   # protocol index or None
        })

    return {
        "joint_names": list(names.JOINT_NAMES),
        "links": links,
        "joints": joints,
        "mesh_files": sorted(set(mesh_files)),
    }


def joint_limits(model):
    """(lo[26], hi[26]) arrays in protocol order from a parse_urdf() dict."""
    lo = [-3.14159] * names.N_JOINTS
    hi = [3.14159] * names.N_JOINTS
    for j in model["joints"]:
        i = j.get("index")
        if i is not None and j["lower"] is not None:
            lo[i], hi[i] = j["lower"], j["upper"]
    return lo, hi

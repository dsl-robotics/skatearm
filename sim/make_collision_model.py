#!/usr/bin/env python3
"""Generate skt_v3_collision.xml: control-ready model with primitive box collisions.

Why: the raw converted meshes interpenetrate at the link mounts and jam the
joints (see make_control_model.py, which simply disables contacts). This script
restores contacts with a clean primitive collision layer instead:

- every mesh geom becomes **visual-only** (contype=0, conaffinity=0, group 1)
- each body gets a **box collision geom** (group 3, shown transparent orange)
  computed from the compiled model's per-geom AABB (`m.geom_aabb`) — using the
  compiled values respects MuJoCo's mesh re-centering (computing AABBs from raw
  mesh vertices + XML offsets double-counts the centering and produces giant
  boxes — been there)
- boxes are shrunk by `shrink=0.85` to soften AABB overestimation
- residual home-pose overlaps (link mounts) get `<contact><exclude>` entries,
  generated automatically: anything already touching at qpos=0 is a
  by-construction overlap, not a real collision (11 pairs for skt_v3)
- contacts are ENABLED

Verified (MuJoCo 3.9): RELAXED/WORK poses hold with max err < 0.03 rad and zero
contacts; commanded arm-crossing stops at hip contact instead of passing
through; a staged out→up→together trajectory brings the wrists into a stable
wrist↔wrist contact (the "clap" in demo_selfcollision.py).

Known limitation: AABB boxes overestimate L-shaped links (esp. wrists), so the
hands "touch" earlier than the real hardware would. Capsules / convex
decomposition are the next refinement if manipulation needs tighter clearances.

Usage:
    python make_control_model.py /path/to/skate_teleop/skt_v3    # first
    python make_collision_model.py /path/to/skate_teleop/skt_v3
"""
import os
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


def quat_mat(q):
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, q)
    return R.reshape(3, 3)


def make(model_dir, shrink=0.85):
    ctrl_path = os.path.join(model_dir, "skt_v3_control.xml")
    if not os.path.exists(ctrl_path):
        sys.exit("run make_control_model.py first")
    root = ET.fromstring(open(ctrl_path).read())

    # re-enable contacts (drop the contact-disable flag)
    for opt in root.findall("option"):
        for flag in list(opt):
            if flag.get("contact") == "disable":
                opt.remove(flag)

    m = mujoco.MjModel.from_xml_path(ctrl_path)
    body_el = {b.get("name"): b for b in root.iter("body")}

    # mesh geoms -> visual only
    for b in root.iter("body"):
        for g in b.findall("geom"):
            if g.get("type") == "mesh":
                g.set("contype", "0")
                g.set("conaffinity", "0")
                g.set("group", "1")

    # box per mesh geom, from compiled AABB
    nboxes = 0
    for gid in range(m.ngeom):
        if m.geom_type[gid] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        bname = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[gid])
        aabb = m.geom_aabb[gid]  # [center(3), half(3)] in geom frame
        center, half = aabb[:3], np.maximum(aabb[3:] * shrink, 0.004)
        gpos, gquat = m.geom_pos[gid], m.geom_quat[gid]  # compiled, body frame
        bpos = gpos + quat_mat(gquat) @ center
        ET.SubElement(body_el[bname], "geom", {
            "type": "box", "group": "3",
            "pos": " ".join(f"{x:.5f}" for x in bpos),
            "quat": " ".join(f"{x:.6f}" for x in gquat),
            "size": " ".join(f"{x:.5f}" for x in half),
            "rgba": "0.8 0.4 0.1 0.35",
        })
        nboxes += 1

    out = os.path.join(model_dir, "skt_v3_collision.xml")
    ET.ElementTree(root).write(out)

    # auto-exclude residual home-pose overlaps
    m2 = mujoco.MjModel.from_xml_path(out)
    d2 = mujoco.MjData(m2)
    mujoco.mj_forward(m2, d2)
    pairs = set()
    for c in range(d2.ncon):
        con = d2.contact[c]
        n1 = mujoco.mj_id2name(m2, mujoco.mjtObj.mjOBJ_BODY, m2.geom_bodyid[con.geom1])
        n2 = mujoco.mj_id2name(m2, mujoco.mjtObj.mjOBJ_BODY, m2.geom_bodyid[con.geom2])
        if n1 != n2:
            pairs.add(tuple(sorted((n1, n2))))
    if pairs:
        contact_el = ET.SubElement(root, "contact")
        for n1, n2 in sorted(pairs):
            ET.SubElement(contact_el, "exclude", {"body1": n1, "body2": n2})
        ET.ElementTree(root).write(out)

    print(f"wrote {out}: {nboxes} boxes, {len(pairs)} home-pose excludes")
    for p in sorted(pairs):
        print("  exclude:", p)
    return out


if __name__ == "__main__":
    make(sys.argv[1] if len(sys.argv) > 1 else ".")

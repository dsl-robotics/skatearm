#!/usr/bin/env python3
"""Generate skt_v3_collision.xml: control-ready model with primitive box collisions.

- mesh geoms become visual-only; each body gets a box from the COMPILED
  per-geom AABB (m.geom_aabb — respects MuJoCo's mesh re-centering)
- per-link shrink: L-shaped wrist links get 0.62 (their AABBs are badly
  overestimated and snag on the table edge during normal reaches), others 0.85
- structural excludes: intra-arm pairs (grandparent links like
  wrist_a1<->wrist_a3 overlap during articulation and jam the wrist) and
  arm<->lower-body pairs (hands graze hip boxes on every swing). Inter-arm
  contact, arm<->torso/head, and everything vs world/parts stay ACTIVE.
- residual home-pose overlaps are auto-excluded
- contacts ENABLED (unlike skt_v3_control.xml)

Usage:
    python make_control_model.py /path/to/skate_teleop/skt_v3    # first
    python make_collision_model.py /path/to/skate_teleop/skt_v3
"""
import os
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ARM_LINKS = ["shoulder", "upperArm", "midArm", "lowArm",
             "wrist_a0", "wrist_a1", "wrist_a2", "wrist_a3"]
LOWER_BODY = ["hip", "upperLeg", "lowerLeg", "wheel"]


def quat_mat(q):
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, q)
    return R.reshape(3, 3)


def make(model_dir, shrink=0.85, wrist_shrink=0.62):
    ctrl_path = os.path.join(model_dir, "skt_v3_control.xml")
    if not os.path.exists(ctrl_path):
        sys.exit("run make_control_model.py first")
    root = ET.fromstring(open(ctrl_path).read())

    # re-enable contacts
    for opt in root.findall("option"):
        for flag in list(opt):
            if flag.get("contact") == "disable":
                opt.remove(flag)

    m = mujoco.MjModel.from_xml_path(ctrl_path)
    body_el = {b.get("name"): b for b in root.iter("body")}

    for b in root.iter("body"):
        for g in b.findall("geom"):
            if g.get("type") == "mesh":
                g.set("contype", "0")
                g.set("conaffinity", "0")
                g.set("group", "1")

    nboxes = 0
    for gid in range(m.ngeom):
        if m.geom_type[gid] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        bname = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[gid])
        sh = wrist_shrink if "wrist" in bname.lower() else shrink
        aabb = m.geom_aabb[gid]  # [center(3), half(3)] in geom frame
        center, half = aabb[:3], np.maximum(aabb[3:] * sh, 0.004)
        gpos, gquat = m.geom_pos[gid], m.geom_quat[gid]
        bpos = gpos + quat_mat(gquat) @ center
        ET.SubElement(body_el[bname], "geom", {
            "type": "box", "group": "3",
            "pos": " ".join(f"{x:.5f}" for x in bpos),
            "quat": " ".join(f"{x:.6f}" for x in gquat),
            "size": " ".join(f"{x:.5f}" for x in half),
            "rgba": "0.8 0.4 0.1 0.35",
        })
        nboxes += 1

    # structural excludes
    structural = set()
    for suffix in ("_1", "_Mirror__1"):
        chain = [l + suffix for l in ARM_LINKS]
        for i in range(len(chain)):
            for j in range(i + 1, len(chain)):
                structural.add(tuple(sorted((chain[i], chain[j]))))
        for al in chain:
            for lb in LOWER_BODY:
                for lsuf in ("_1", "_Mirror__1"):
                    structural.add(tuple(sorted((al, lb + lsuf))))

    out = os.path.join(model_dir, "skt_v3_collision.xml")
    ET.ElementTree(root).write(out)

    # residual home-pose overlaps
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

    all_pairs = structural | pairs
    contact_el = ET.SubElement(root, "contact")
    for n1, n2 in sorted(all_pairs):
        ET.SubElement(contact_el, "exclude", {"body1": n1, "body2": n2})
    ET.ElementTree(root).write(out)
    print(f"wrote {out}: {nboxes} boxes, {len(pairs)} home-pose + {len(structural)} structural excludes")
    return out


if __name__ == "__main__":
    make(sys.argv[1] if len(sys.argv) > 1 else ".")

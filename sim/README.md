# sim — MuJoCo digital twin

Phase 0 result: the official `skt_v3` model loads, poses and renders headlessly.

## Setup

```bash
git clone https://github.com/Rbotic/skate_teleop.git
pip install mujoco numpy imageio
python render_skate.py --model skate_teleop/skt_v3 --out renders
```

On a headless Linux box use `MUJOCO_GL=egl` (GPU) or `MUJOCO_GL=osmesa` (CPU, `apt install libosmesa6`).

## What we know about the model (verified 11.06.2026, MuJoCo 3.9)

- `skt_v3_converted.xml`: 26 named hinge joints + free base joint (nq=33)
  - `a0…a7` — lower chain (hips/legs/wheels of the full Skate)
  - `a0_armL_a8 … a7_armL_a15` — left arm (8 DoF)
  - `a0_armR_a16 … a7_armR_a23` — right arm (8 DoF)
  - `a0_head_a24`, `a1_head_a25` — head pan/tilt
- Mirrored arm chains take the **same sign** for a symmetric pose.
- **Respect joint ranges** — they are asymmetric: e.g. `a1` (abduction) −0.79…2.36, `a3` (elbow) 0…2.64 — the elbow can't bend backwards. Out-of-range qpos silently interpenetrates meshes and looks broken; `render_skate.py` clamps as a guard.
- **No actuators (`nu=0`), no sensors** — it is a visualization/teleop model. A control-ready MJCF (position servos + sensors) is the next sim task and a community-tool candidate.
- Base free joint origin is mid-body: lift `qpos[2] ≈ 0.95` to put wheels on a floor plane.

## Control-ready model

`make_control_model.py` turns the visualization-only MJCF into a controllable one:

- 26 **position actuators** (kp=100), ctrlrange = joint ranges, forcerange ±28 N·m from the URDF
- joint **damping 2.0 + armature 0.05** (the original has none — undamped servos oscillate)
- **fixed base** (freejoint removed) — work-cell configuration
- **contacts disabled**: the raw converted meshes interpenetrate at the shoulder mounts and jam the joints — the shoulder servo saturates at its 28 N·m force limit just fighting the contact. Free-space control is correct without contacts; real collision geometry is a roadmap task.

Verified: holds RELAXED and WORK poses with **max error < 0.03 rad (~1.5°)**, settles to zero velocity, no divergence (MuJoCo 3.9, 2 ms timestep).

`demo_wave.py` runs a closed-loop sequence (independent left/right arm trajectories + head pan) and writes the GIF in the main README.

## Collision model

`make_collision_model.py` builds `skt_v3_collision.xml` on top of the control model:

- mesh geoms become **visual-only**; each body gets an auto-generated **box collision geom** from the compiled model's AABB (`m.geom_aabb` — using compiled values respects MuJoCo's mesh re-centering; raw mesh vertices + XML offsets double-count it and produce giant boxes)
- residual home-pose overlaps (link mounts: torso↔shoulders/hips, wrists↔hips at hanging pose) are **auto-excluded** — 11 pairs
- contacts **re-enabled**: poses still hold at < 0.03 rad; commanded arm-crossing now *blocks on the hips* instead of tunneling; a staged out→up→together trajectory ends in a stable wrist↔wrist contact

Known limitation: AABB boxes overestimate the L-shaped wrist links, so hands "touch" a bit early and the direct path to a hands-together pose snags on the hip boxes (the demo routes around: OUT → UP → MEET). Capsules/convex decomposition are the next refinement.

## Files

- `render_skate.py` — patch scene (floor/light/framebuffer), set bimanual pose, render PNGs. Outputs in [../docs/img/](../docs/img/).
- `make_control_model.py` — generate `skt_v3_control.xml` (actuators + damping + fixed base, contacts off).
- `make_collision_model.py` — generate `skt_v3_collision.xml` (box collision layer, contacts on).
- `demo_wave.py` — physics demo: arm trajectories under position control → GIF or MP4 (format follows the `--out` extension; MP4 needs `pip install imageio-ffmpeg`).
- `demo_selfcollision.py` — hands-meet demo with the collision layer revealed mid-clip → GIF or MP4.
- `telemetry_demo.py` — log sensors during the wave trajectory → tracking/torque/EE plot (+ optional CSV).

## Sensors

Both generated models carry 82 sensors: `qpos_<joint>`, `qvel_<joint>`,
`tau_<joint>` for all 26 joints, plus `ee_left`/`ee_right` wrist sites with
`framepos`/`framequat`. The naming is the telemetry schema for everything
downstream (dashboard, datasets, the real Skate's state stream).

## Media convention

Every milestone gets **three kinds of media**: stills (`docs/img/*.png`), a small GIF for inline README preview, and an HD MP4 (`docs/video/*.mp4`, 1280x960/30fps).

To get an *embedded video player* in the GitHub README (instead of a file link): open README.md in the GitHub web editor and **drag the .mp4 into the edit area** — GitHub uploads it to user-attachments and inserts a URL that renders as a player. Repo-stored .mp4 files only render a player on their own file page, not inside README.

# SkateArm — Roadmap

## Phase 0 — Foundations (now, no hardware)
- [x] Get the official skt_v3 twin loading + rendering in MuJoCo
- [x] Repo skeleton, architecture, roadmap
- [x] Control-ready MJCF: 26 position actuators + damping, fixed base, contacts disabled (converted meshes jam at shoulder mounts); holds poses < 0.03 rad error, closed-loop demo GIF
- [x] Primitive collision geometry: auto-generated boxes from compiled AABBs + home-pose excludes; contacts re-enabled, self-collision verified (hands meet & stop, arms block on hips instead of tunneling)
- [ ] Tighter collision shapes (capsules / convex decomposition) — AABB boxes overestimate the wrists
- [x] Sensors: jointpos/jointvel/actuatorfrc ×26 + EE sites with framepos/framequat (82 sensors); telemetry demo plot
- [x] Demonstrator task spec v1: bimanual peg-in-hole Ø20 H9/d9 (→H7/g6), GRAFCET cycle, takt ≤60 s, QC characteristics, success metrics (4 DECISIONs open for Daniels)
- [ ] Decide thesis/capstone registration with RTU supervisor

## Phase 1 — Sim work-cell (MuJoCo)
- [x] Work-cell scene: table, base part (spec masses 45 g/12 g), peg, bins as free bodies (`make_cell_scene.py`); v1 pocket is square/blind — round H9 bore arrives with the QC package
- [x] REACH primitive: closed-loop weighted-DLS IK via position actuators, bimanual, ≤ 2.5 cm under physics; smoothstep target gliding + task-space step clamp for human-smooth motion (`primitives.py`, lessons in sim/README)
- [x] Joint-space MOVE primitive + collision-aware fold→raise route past the table edge (`move_joints`)
- [x] GRASP/RELEASE + PICK & PLACE: weld-constraint grasp stand-in engaged at the part's current pose (no snap); both parts carried off the table and placed back (`demo_cell_pick.py`). Real gripper geometry replaces the stand-in when hardware arrives
- [x] INSERT: relative servoing (peg→pocket from body poses — the future QC camera's job), force-guarded descent with τ watchdog, live xy correction; depth 18.5 mm, tilt ≤2°, assembled unit survives placement (`demo_cell_assemble.py`). Key enablers: lateral-offset grasps (hands don't collide at the meet point) and orientation-locked 6-DOF carry (prevention beats correction — fixing an accumulated tilt runs the wrist into its limits)
- [ ] GRAFCET sequencer driving sim cell (soft-PLC)
- [ ] Camera rendering in sim → first QC pipeline (classical CV)
- [ ] Dashboard skeleton (Flask + SQL) logging sim cycles
- [ ] First community tool shipped: `skate_ros2` bridge (sim side) **or** control-ready MJCF release

## Phase 2 — Real Skate bring-up (hardware in Riga)
- [ ] Unboxing → teleop → joint-by-joint validation (document as handbook chapters)
- [ ] `skate_ros2` against real UDP endpoint
- [ ] Teleop dataset collection (LeRobot format) → dataset hub
- [ ] ACT policy for the hold/insert subtask

## Phase 3 — Integrated demonstrator
- [ ] Full cycle: feeder → bimanual assembly → GD&T inspection → dashboard log
- [ ] Benchmark suite release
- [ ] Technical report / thesis text assembled from repo docs

*Sequencing rule: every phase ships at least one standalone community tool.*

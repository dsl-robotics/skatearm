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
- [ ] Dual-arm reach/hold/insert primitives in sim
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

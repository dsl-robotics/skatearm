# SkateArm — Roadmap

## Phase 0 — Foundations (now, no hardware)
- [x] Get the official skt_v3 twin loading + rendering in MuJoCo
- [x] Repo skeleton, architecture, roadmap
- [x] Control-ready MJCF: 26 position actuators + damping, fixed base, contacts disabled (converted meshes jam at shoulder mounts); holds poses < 0.03 rad error, closed-loop demo GIF
- [x] Primitive collision geometry: auto-generated boxes from compiled AABBs + home-pose excludes; contacts re-enabled, self-collision verified (hands meet & stop, arms block on hips instead of tunneling)
- [x] Tighter collision shapes: capsules auto-fitted from compiled AABBs (longest axis + covering radius, near-isotropic links become spheres) — the wrists stop reading as bricks; `--boxes` keeps the old layer; guard e2e re-verified on the capsule model
- [x] Sensors: jointpos/jointvel/actuatorfrc ×26 + EE sites with framepos/framequat (82 sensors); telemetry demo plot
- [x] Demonstrator task spec v1: bimanual peg-in-hole Ø20 H9/d9 (→H7/g6), GRAFCET cycle, takt ≤60 s, QC characteristics, success metrics (4 DECISIONs open for Daniels)
- [ ] Decide thesis/capstone registration with RTU supervisor

## Phase 1 — Sim work-cell (MuJoCo)
- [x] Work-cell scene: table, base part (spec masses 45 g/12 g), peg, bins as free bodies (`make_cell_scene.py`); v1 pocket is square/blind — round H9 bore arrives with the QC package
- [x] REACH primitive: closed-loop weighted-DLS IK via position actuators, bimanual, ≤ 2.5 cm under physics; smoothstep target gliding + task-space step clamp for human-smooth motion (`primitives.py`, lessons in sim/README)
- [x] Joint-space MOVE primitive + collision-aware fold→raise route past the table edge (`move_joints`)
- [x] GRASP/RELEASE + PICK & PLACE: weld-constraint grasp stand-in engaged at the part's current pose (no snap); both parts carried off the table and placed back (`demo_cell_pick.py`). Real gripper geometry replaces the stand-in when hardware arrives
- [x] INSERT: relative servoing (peg→pocket from body poses — the future QC camera's job), force-guarded descent with τ watchdog, live xy correction; depth 18.5 mm, tilt ≤2°, assembled unit survives placement (`demo_cell_assemble.py`). Key enablers: lateral-offset grasps (hands don't collide at the meet point) and orientation-locked 6-DOF carry (prevention beats correction — fixing an accumulated tilt runs the wrist into its limits)
- [x] GRAFCET sequencer driving the sim cell: step engine with sensor receptivities (S0–S7), force-guard divert to reject branch, full cycle 42.4 s ≤ 60 s takt, JSON cycle log (`sequencer.py`, `demo_cell_cycle.py`, `logs/cycle_001.json`)
- [x] QC camera pipeline: qc_top/qc_side fixed cameras, classical CV (color segmentation + fixed inspection window + pocket-rim reference), camera verdict in the sequencer with oracle cross-check; residuals align ±1.3 mm / depth ±3.4 mm (`qc.py`; lessons in sim/README — camera roll, lighting biases, part presentation)
- [x] Dashboard (Flask + SQLite): KPI cards, cycle-time trend vs takt, camera/oracle residuals, GRAFCET step timeline per cycle; ingest schema = sequencer event stream (real-cell ready) (`dashboard/app.py`)
- [x] First community tool shipped: **`skate_ros2`** (`tools/skate_ros2/`) — documented UDP wire protocol, pure-Python client, MuJoCo sim endpoint speaking the real protocol, rclpy driver with firmware-mirrored safety (arm-at-pose, deadman freshness, overtemp latch); 17 ROS-free unit tests + e2e over real sockets (60 Hz cmds, ~190 pkt/s telemetry, 0.015 rad tracking, watchdog < 0.3 s)
- [ ] `ros2_control` hardware interface + MoveIt 2 config over the bridge
- [x] **`skate_commander` v0.1** (`tools/skate_commander/`) — web cockpit over the same UDP wire: in-browser URDF twin (FK validated vs MuJoCo < 0.001 mm), joint jog with live angle/vel/temp, SIM/REAL toggle, estop-first safety (starts dampened, arm-at-measured-pose, legs locked in REAL); FastAPI+WS backend, ws→UDP→MuJoCo e2e tested; functional reference: Waldo Commander (PAROL6), own design
- [x] `skate_commander` v0.2–v0.4: cartesian drag-gizmo (pure-numpy DLS IK, FK = MuJoCo ±0; `lower or -π` on the elbow's 0.0 limit was the bug of the week), draggable command sliders, waypoint sequencer (glide/dwell/loop, save/load, manual input overrides), TCP traces, **collision guard** (guard-specific model re-enables 136 hand↔leg pairs the physics model excludes; interpolated-path check kills tunneling; verified on plain Windows/Py3.13), full design pass (graphite + azure/amber command semantics)
- [x] **`skate_commander` v0.5** — the rest of the Waldo feature catalog, bimanual-first: cartesian XYZ step-jog with live TCP readout (auto-clearing IK targets), jump-to-limit, **mirror mode** (sign map measured numerically from FK — turned out axis=x, all +1), **Python programs** (sandboxed `rbt` API, Click-to-Step with line tracking, E-STOP/manual-input abort, save/load), tool/TCP-offset manager (FK/IK/gizmo/traces follow the active tool), capsule collision layer, bridge-level REAL leg lock; +3 test suites (kinematics tool offsets, cart/mirror e2e, program runner e2e)
- [x] **`skate_commander` v0.6** — teach-in: ● REC watches the commanded pose, every settle becomes a line of `rbt` code (`movej` / coordinated `pose({...})` — new bulk API with a single guard check, no mirror), generated program appends to the PROG editor and replays through the same safe bridge; null-space comfort objective in the IK (v0.5.1/2: the redundant arm picks its own elbow, no winding on out-and-back cartesian jogs)
- [x] **`skate_commander` v0.7** — product hardening: one-command launcher (`python -m skate_commander` — auto-detects the model, builds the sim/guard models on first run, opens the browser); connection robustness (browser↔server offline + stalled-telemetry detection, WS auto-reconnect, backend ignores bad commands); program-editor UX (`rbt` autocomplete, example library, error/step line highlighting); docs site + `rbt` API reference (dsl-robotics.github.io/skatearm/commander.html)
- [ ] `skate_commander` v0.8+ (camera passthrough, real-gripper presets) — detailed in **Road to Commander v1.0** below

## Road to Commander v1.0 — "the cockpit safely drives the real Skate"

**v1.0 scope (locked):** Skate Commander reliably and safely drives the **real** R.Botic Skate from the browser — jog, drag-IK, mirror, teach-in, programs — calibrated, hardened, documented and demoed on hardware. Autonomous work-cell + learned policies are a separate post-1.0 track (Phases 2–3, targeted at v1.x).

### Pre-hardware prep (now, while the Skate ships — lands as v0.7.x)
- [ ] MoveIt 2 + `ros2_control` over the bridge, validated in sim (then re-pointed at the real robot)
- [ ] LeRobot dataset-recorder: teleop / teach-in → `LeRobotDataset` (so demos can be captured day-one of hardware)
- [ ] Cockpit polish: mobile/responsive layout, persisted settings & panel state, keyboard shortcuts
- [ ] Handbook skeleton in the docs (unboxing → first teleop)

### v0.8 — "First contact" (hardware bring-up)
- [ ] Unbox → power → network (RPi 5, UDP `r.local`) → first real telemetry in the cockpit (REAL, dampened)
- [ ] `skate_ros2` against the **real** UDP endpoint (`127.0.0.1` → `r.local`): verify packet layout, rates, deadman on hardware
- [ ] Joint-by-joint validation — every commanded channel moves the right real joint (the SO-101 lesson)
- [ ] Camera passthrough (real cameras) + real-gripper tool presets
- [ ] Safety on hardware: E-STOP / deadman / overtemp / collision-guard verified on the real arm; define a known-safe startup pose
- [ ] 🎯 the cockpit safely jogs/drags the real Skate → first real-robot video

### v0.9 — "Calibrated & trustworthy"
- [ ] Calibration reconciliation: servo offsets ↔ URDF ↔ cockpit twin ↔ real pose — the on-screen twin matches the real arm (the central SO-101 problem)
- [ ] IK / limits / collision-guard tuned to real geometry (convex collision if the capsule layer is too coarse)
- [ ] Repeatable teach-in → program → replay on the real robot, plus an on-robot validation checklist
- [ ] 🎯 poses and programs run repeatably; the twin is faithful

### v1.0 — "Production-ready Commander + real Skate"
- [ ] End-to-end demo on hardware: a real pick-place (or the Phase-1 cycle) on the live Skate, recorded
- [ ] Hardening: error handling & reconnection on the real network, persisted settings, mobile polish
- [ ] Docs: real-hardware handbook with **real** numbers replacing the sim-endpoint figures; updated API & safety
- [ ] Media + release: real-robot video, the deferred dev.to write-up, `v1.0.0` release notes
- [ ] 🎯 **v1.0 — Commander runs on the real Skate, documented and demoed**

*Watch-outs: hardware-arrival timing is the main unknown; calibration is the hardest part (budget time for it); solo-dev — keep v1.0 focused on cockpit + bridge + safety, push AI/autonomy to v1.x.*

## Phase 2 — Real Skate bring-up (hardware in Riga)  *(→ v1.x, post-1.0: autonomous cell + learned policies)*
- [ ] Unboxing → teleop → joint-by-joint validation (document as handbook chapters)
- [ ] `skate_ros2` against real UDP endpoint
- [ ] Teleop dataset collection (LeRobot format) → dataset hub
- [ ] ACT policy for the hold/insert subtask

## Phase 3 — Integrated demonstrator
- [ ] Full cycle: feeder → bimanual assembly → GD&T inspection → dashboard log
- [ ] Benchmark suite release
- [ ] Technical report / thesis text assembled from repo docs

*Sequencing rule: every phase ships at least one standalone community tool.*

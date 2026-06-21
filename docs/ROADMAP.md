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
- [x] **`skate_commander` v0.7.1–v0.7.5** — Alloy-inspired feature run + UI redesign: dual-arm **CARRY** (v0.7.1), **singularity / SING** awareness (v0.7.2), **jerk-limited motion** profiles (v0.7.3), **closed-loop IBVS visual servoing** / SERVO pick (v0.7.4 — open-loop ~43 mm vs IBVS ~5 mm in sim), and a **cockpit UI redesign** retoned to the landing-site visual language: slim status topbar + floating control dock + CARRY popover (v0.7.5). Per-feature notes in the backlog below
- [x] **`skate_commander` v0.7.6** — motion-quality + safety: a jerk-limited **`home()`** glide (eased like the waypoint moves, with a graceful give-up when the straight path is guard-blocked) and a **contact reflex** — a torque spike on a *stalled* arm joint (loaded but not moving) latches a soft-stop, cleared from the CONTACT chip
- [x] **`skate_commander` v0.7.7** — **collision-free planning** (RRT-Connect): when the straight path to **Home** would clip a self-collision, the planner routes the arms *around* it instead of stalling; the legs / balance chain are left untouched (a stray ~0.01 rad of leg-sensor noise was what made a naive full-body plan infeasible). A **ROUTING** chip shows while a route runs; backed by a pure, headless-tested `planner.py`
- [x] **`skate_commander` v0.7.8** — **planned routing for waypoint moves**: `wp_goto` / `wp_play` now route each leg *around* a self-collision the straight path can't pass (same RRT planner as Home), instead of stalling. Plus a robustness fix — a planner-verified route is no longer re-checked by the guard per-tick (that was false-stalling the glide on grazing corners), so routes follow cleanly
- [x] **`skate_commander` v0.7.9** — **manipulability heat-map**: a `DEX` toggle renders each arm's reachable workspace as a colour-graded point cloud in the twin (warm = dexterous / isotropic, blue = near the singular reach limits), sampled server-side from a fast geometric (axis × lever) Jacobian (~15× the central-difference one) and cached. The Alloy singularity reel's "manipulability heat-volume in the twin", delivered
- [x] **`skate_commander` v0.7.10** — **work-camera point cloud**: a `PCL` toggle back-projects the work camera's rendered depth into the twin as a colour-graded point cloud (each point takes its RGB pixel's colour) — a live 3-D reconstruction of what the camera sees (table, target), reconstructing the magenta cube to ~2 mm of ground truth (`vision.depth_cloud`). A step toward the smarter-pick backlog item. Plus a camera robustness fix — the renderer forward-kinematics every frame, so it draws the scene even before any telemetry
- [x] **`skate_commander` v0.7.11** — **smart-pick: grasp synthesis on the point cloud**: the v0.7.10 cloud is turned into a grasp — a RANSAC plane fit removes the table, voxel connected-components cluster the rest, and a top-down parallel-jaw grasp is fit to the object's OWN geometry: centre, a MEASURED grasp height (mid plane↔top, not the hard-coded `GRASP_Z`), footprint + yaw (the jaws close across the minor axis) and a gripper-width feasibility check. A `GRASP` toggle draws it in the twin (footprint + jaw line + top-down approach); a `SMART` button executes it through the guarded runner (refusing an object too wide for the jaws). **Finding:** the cloud also holds the robot's OWN legs — two big elongated clusters that out-number a small part — so "largest cluster" grabbed a leg; fixed by selecting the flat, compact top face a resting object presents to the overhead camera (caught on the live cross-check, not in the unit tests). 8 headless tests + the live sim cross-check (cube recovered to ~3 mm, legs rejected). `grasp.py`
- [x] **`skate_commander` v0.7.12** — **smarter pick: multi-object pluggable detector**: smart-pick now finds EVERY graspable object on the work surface (`grasp.plan_grasps`), and a pluggable detector (`detect.py`) labels each by colour + shape — so a multi-object scene can be picked BY NAME. The `GRASP` toggle draws all candidate grasps (the selected one azure, the rest dimmed); an object selector in the camera bar + `SMART` `?target=` pick the chosen one. The detector is the deterministic-core + optional-heavy-backend pattern of `nl.py`: built-in colour/shape always, with an opt-in **YOLO** backend (`SKATE_YOLO` + ultralytics) that overlays class names on real / COCO objects and falls back to the built-in labels otherwise (no torch unless opted in). 6 detector tests + a multi-object grasp test (45 total); live: a magenta cube + a cyan box both detected & labelled, the robot's legs rejected. `detect.py`, `grasp.plan_grasps`
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

## Research & integration backlog (inspiration: @alloyrobotics)
Techniques worth folding into the cockpit, from a survey of @alloyrobotics' MuJoCo explainers. Shipped items landed in v0.7.x; the rest are post-1.0.

- [x] **Dual-arm carry** — *shipped in the cockpit* (v0.7.1: a `CARRY` mode where both wrists hold one object and move together via the topbar X/Y/Z pad, guard-protected) + *sim co-lift demo* (`sim/demo_dual_carry.py`, `docs/img/dual_carry.gif`): both wrists weld-grasp one bar at the natural ~17 cm separation and co-lift it with a load-sharing term + gravity feed-forward. **Finding:** the arms can't squeeze in x (they self-collide), so "co-lift + balance the load" is the feasible bimanual primitive. Next: rotation + load-sharing on the held object; soft-weld A/B.
- [x] **Gravity feed-forward** (`mj_rne`, qvel=0 → gravity-only, stable) — *shipped* in `primitives.reach(grav_ff=True)`: gravity-only feed-forward on the hinge joints via `qfrc_applied` (qvel=0 sidesteps the Coriolis instability a `qfrc_bias` feedforward has), cleared on exit so it never leaks into a force-guarded descent; free bodies (carried parts) still fall. Measured: 30 mm reach sag → 1 mm, servo holding offset 0.033 rad → 0 (`sim/test_gravity_ff.py`).
- [x] **Visual servoing (IBVS)** — *shipped* (v0.7.4: a `SERVO` pick that detects the static target once, then drives the gripper's image feature onto it while descending — closed-loop, so it stays on target despite camera mis-calibration where the one-shot `PICK` misses. Sim test: 3 cm / 12% camera error → open-loop misses 43 mm, IBVS converges to 5 mm. `ibvs.py` + `/api/servo_pick`).
- [x] **Singularity / manipulability** awareness in drag-IK — *shipped* (v0.7.2: per-arm manipulability = reciprocal Jacobian condition number, streamed in telemetry; a `SING` topbar chip warns when either active arm drops below 0.06, near a wrist singularity). Plus a **manipulability heat-map** (`DEX` toggle: a colour-graded dexterity cloud of the reachable workspace in the twin) — *shipped* (v0.7.9). Next: throttle the IK step near the warn line.
- [x] **Contact reflex** (torque-spike stop) — *shipped* (v0.7.6: a torque spike on a *stalled* arm joint — loaded but barely moving — latches a soft-stop, cleared from the CONTACT chip; grippers excluded so a grasp doesn't trip it). **RRT\*/A\*** collision-free planning — *shipped* (v0.7.7: RRT-Connect + shortcut smoothing in `planner.py`; `home()` routes the arms *around* the self-collision it used to give up on, leaving the legs to the firmware). Waypoint moves (`wp_goto` / `wp_play`) route around collisions too — *shipped* (v0.7.8).
- [ ] **Smarter pick**: learned detector (YOLO) + point-cloud + shape-completion (vs colour + plane). *Shipped:* the work-camera depth is back-projected to a coloured cloud (v0.7.10, `PCL`), and **grasp synthesis on that cloud** (v0.7.11, `grasp.py` / `GRASP` / `SMART`) — table removed, object clustered, a top-down parallel-jaw grasp fit to its geometry with a width-feasibility check. and a **multi-object pluggable detector** (v0.7.12, `detect.py`) — built-in colour/shape labelling of every graspable cluster + pick-by-name, with an opt-in YOLO backend (`SKATE_YOLO`) for real / COCO objects. Remaining: trained YOLO weights on the sim / real objects, and shape-completion for partially-seen parts.
- [x] **Trajectory smoothing + S-curve motion profiles** — *shipped* (v0.7.3: acceleration-limited jog — held jog eases in, eases out on release; trapezoidal sqrt-decel profile on waypoint/replay glides. Safety stops stay immediate). Same easing now on `home()` too — *shipped* (v0.7.6).
- [ ] **Learning track (v1.x):** teach-in → LeRobot dataset → behaviour cloning (+DAgger) / diffusion policy / VLA (SmolVLA, MolmoAct), with domain randomization for sim2real.

*Sequencing rule: every phase ships at least one standalone community tool.*

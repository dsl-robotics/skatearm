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
- [x] GRAFCET sequencer driving the sim cell: step engine with sensor receptivities (S0–S7), force-guard divert to reject branch, full cycle 42.4 s ≤ 60 s takt, JSON cycle log (`sequencer.py`, `demo_cell_cycle.py`, `logs/cycle_001.json`)
- [x] QC camera pipeline: qc_top/qc_side fixed cameras, classical CV (color segmentation + fixed inspection window + pocket-rim reference), camera verdict in the sequencer with oracle cross-check; residuals align ±1.3 mm / depth ±3.4 mm (`qc.py`; lessons in sim/README — camera roll, lighting biases, part presentation)
- [x] Dashboard (Flask + SQLite): KPI cards, cycle-time trend vs takt, camera/oracle residuals, GRAFCET step timeline per cycle; ingest schema = sequencer event stream (real-cell ready) (`dashboard/app.py`)
- [x] First community tool shipped: **`skate_ros2`** (`tools/skate_ros2/`) — documented UDP wire protocol, pure-Python client, MuJoCo sim endpoint speaking the real protocol, rclpy driver with firmware-mirrored safety (arm-at-pose, deadman freshness, overtemp latch); 15 ROS-free unit tests + e2e over real sockets (60 Hz cmds, ~200 pkt/s telemetry, 0.015 rad tracking, watchdog < 0.3 s)
- [ ] `ros2_control` hardware interface + MoveIt 2 config over the bridge
- [x] **`skate_commander` v0.1** (`tools/skate_commander/`) — web cockpit over the same UDP wire: in-browser URDF twin (FK validated vs MuJoCo < 0.001 mm), joint jog with live angle/vel/temp, SIM/REAL toggle, estop-first safety (starts dampened, arm-at-measured-pose, legs locked in REAL); FastAPI+WS backend, ws→UDP→MuJoCo e2e tested; functional reference: Waldo Commander (PAROL6), own design
- [ ] `skate_commander` v0.2: cartesian drag-gizmo (DLS IK), waypoint sequencer + playback, TCP trace, tool/TCP offsets

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

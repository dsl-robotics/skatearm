# SkateArm

**A two-armed robot you can drive from your browser — first as a 3D simulation, then, with one switch, the real [R.Botic Skate](https://www.rboticlabs.com/shop/p/skate-upper-body-v2).**

*An open bimanual work-cell & tool ecosystem: two-handed assembly with in-cell quality inspection, built sim-first in MuJoCo, then deployed over the robot's native UDP wire.*

<div align="center">
  <a href="https://dsl-robotics.github.io/skatearm/"><img src="docs/img/cockpit_v0724_demo.gif" width="820" alt="Skate Commander cockpit — mirror mode drives both arms from one slider while live telemetry plots track the motion"></a>
</div>

<div align="center">

[![License: MIT](https://img.shields.io/badge/license-MIT-green?style=flat)](LICENSE)
[![MuJoCo](https://img.shields.io/badge/sim-MuJoCo%203.x-orange?style=flat)](sim/)
[![ROS 2](https://img.shields.io/badge/bridge-ROS%202-22314E?style=flat)](tools/skate_ros2/)
[![Python](https://img.shields.io/badge/python-3.x-3776AB?style=flat)](sim/)
[![tests](https://github.com/dsl-robotics/skatearm/actions/workflows/tests.yml/badge.svg)](https://github.com/dsl-robotics/skatearm/actions/workflows/tests.yml)

</div>

<div align="center"><sub>CI runs the hardware-free subset (wire protocol · joint map · driver safety · NL parser). The model-gated tests — collision guard, RRT planner, IK & URDF — run locally against the <code>skt_v3</code> model and <strong>skip</strong> without it; see <a href="CONTRIBUTING.md#running-the-tests">CONTRIBUTING</a>.</sub></div>

<div align="center">

**▶ Watch the 3:46 product film** — a full walkthrough of the current v0.8 cockpit: digital twin, drag-IK, RRT collision-routing, mirror mode, dual-arm carry, live telemetry and teach-in.

🌐 **Live demo & write-up → [dsl-robotics.github.io/skatearm](https://dsl-robotics.github.io/skatearm/)**

🕹 **Try the cockpit in your browser — no install → [live preview](https://raw.githack.com/dsl-robotics/skatearm/main/tools/skate_commander/preview.html)** *(runs on recorded telemetry)*

</div>

<div align="center">
  <a href="https://github.com/dsl-robotics/skatearm/blob/main/docs/video/commander_v08_product.mp4">
    <img src="docs/img/commander_v08_video_thumb.png" width="720" alt="Skate Commander v0.8 — product walkthrough (click to play)">
  </a>
</div>

<div align="center">
  <img src="docs/img/cell_cycle_demo.gif" width="420px" alt="Autonomous GRAFCET assembly cycle with camera QC and HMI overlay">
  <img src="docs/img/commander_v06_overview.gif" width="420px" alt="Skate Commander overview: mirror-mode bimanual jog raises both arms from one slider, then teach-in writes a program from hand-moved poses and replays it">
  <br>
  <em>Left: <strong>Phase 1 complete</strong> — the autonomous bimanual assembly cycle (GRAFCET sequencer, camera QC).
  Right: <strong>Skate Commander</strong> — mirror-mode bimanual jog, then teach-in: move the arms by hand and the cockpit writes the <code>rbt</code> program itself.</em>
</div>

## What are you here for?

| You want to… | Go to |
|---|---|
| Drive the robot (twin or real) from a browser | [🕹 Skate Commander](#-skate-commander--web-cockpit) |
| Connect a ROS 2 stack to a Skate | [🔌 skate_ros2](#-skate_ros2--the-wire) |
| See the autonomous assembly cell | [🏭 Work-cell](#-autonomous-work-cell-phase-1--complete) |
| Get the control-ready model & collision layer | [🦾 Sim foundations](#-sim-foundations-phase-0) |
| Run it yourself | [🚀 Quick start](#-quick-start-simulation) |

<details>
<summary><strong>New to the jargon?</strong> A 20-second glossary (click to expand)</summary>

- **MuJoCo** — a physics simulator; the robot "lives" here virtually before any real hardware exists.
- **ROS 2** — the standard open-source middleware (the robot's "operating system").
- **URDF** — the file describing the robot's links, joints and limits.
- **FK / IK** — forward kinematics ("where the hand is for these joint angles") / inverse kinematics ("which joint angles put the hand there").
- **TCP** — tool center point: the exact tip of the tool the robot controls.
- **Jog** — nudging a joint or the tool one small step at a time (hold a button or drag a slider).
- **Digital twin** — a 3D copy of the real robot, driven by the same commands.
- **Deadman / E-STOP** — safety: motion stops if the connection goes silent or you hit emergency-stop.

</details>

## 🕹 Skate Commander — web cockpit

<div align="center">
  <img src="docs/img/skate_commander_lockup.png" width="560" alt="Skate Commander — web cockpit, digital twin, real robot">
</div>

> 🚧 **Early access · under active development** — v0.8.0 is sim-first; drive the twin in your browser now, real-Skate support lands with the hardware.

A browser cockpit for the Skate: a 3D digital twin built from the official URDF, driven over the **same UDP wire** the real robot speaks. Starts E-stopped, arms at the robot's measured pose, deadman drops in 0.3 s if the tab closes.

<div align="center">
<table>
  <tr>
    <td width="50%"><img width="100%" src="docs/img/cockpit_dex.webp" alt="Manipulability dexterity cloud rendered around the robot"><br><sub><b>Manipulability cloud</b> — warm where dexterous, blue near singular reach</sub></td>
    <td width="50%"><img width="100%" src="docs/img/cockpit_plots.webp" alt="Live Foxglove-style telemetry strip charts under the 3D view"><br><sub><b>Live telemetry plots</b> — angle / velocity / temperature / TCP / RTT at 30 Hz</sub></td>
  </tr>
  <tr>
    <td width="50%"><img width="100%" src="docs/img/cockpit_v0724_cockpit.webp" alt="The v0.8.0 Isaac-Sim-style cockpit: menu bar, tool rail, 3D twin, STAGE/PROPERTY dock"><br><sub><b>Isaac-Sim-style workstation</b> — menu bar, tool rail, Stage / Property dock, timeline</sub></td>
    <td width="50%"><img width="100%" src="docs/img/cockpit_ghost.webp" alt="Translucent ghost-robot preview with an Approve / Cancel gate"><br><sub><b>Ghost preview</b> — risky moves wait behind an Approve / Cancel gate</sub></td>
  </tr>
</table>
</div>

**The cockpit packs a robotics-workstation's worth of tooling** — drag-IK and mirror-mode bimanual motion, RRT collision-routing, Python + teach-in programs, an Isaac-Sim-style Stage / Property shell, live telemetry plots, a TF tree, diagnostics, and scene markers with keep-out obstacles. The full catalogue:

<details>
<summary><strong>▸ Full cockpit feature catalogue</strong> — 30+ features across motion · programs · vision · safety · observability · scene tools <em>(click to expand)</em></summary>

### Motion, IK &amp; manipulability

| Feature | What it does |
|---|---|
| Jog + sliders | Hold −/+, drag the thumb, or jump straight to a limit; amber = your command, azure = actual position |
| Cartesian jog | Step the TCP along world X/Y/Z in mm — server-side IK, auto-stops on arrival |
| Drag-IK | Grab a wrist sphere in 3D — server-side DLS IK (damped least squares inverse kinematics) glides all 7 arm joints |
| Singularity awareness | Live manipulability readout; a **SING** chip warns near a wrist singularity, where a small cartesian move would need huge joint speeds |
| Manipulability map | A **DEX** toggle renders a coloured point cloud of the arm's reachable workspace — warm where the arm is dexterous, blue near its singular reach limits |
| Mirror mode | Bimanual: jog/slider/IK on one arm is reflected onto the other — the sign map is *measured* from the model's FK, not guessed |
| Dual-arm carry | **CARRY** — both wrists hold one object and move together via an X/Y/Z pad, preserving their separation (a true two-handed carry) |
| Jerk-limited motion | Jog, replay and **Home** use acceleration-limited / trapezoidal profiles — motion eases in and out instead of snapping (E-STOP still stops instantly) |

### Programs &amp; teaching

| Feature | What it does |
|---|---|
| Python programs | Built-in editor + `rbt` API (`movej`/`pose`/`movel`/`home`/waypoints); **Click-to-Step** runs one motion at a time; E-STOP or any manual input kills the program |
| Control flow | A **+ FLOW** snippet bar inserts indent-aware `repeat` / `while` / `if` / `wait` skeletons, with `rbt.ok()` / `blocked()` / `contact()` / `near()` condition helpers — loops and conditionals run on the same guarded bridge |
| Natural-language programs | Describe a task in plain English — a safe **offline** parser writes the `rbt` program into the editor (AST-validated; optional LLM fallback), which you then Click-to-Step through the same guarded bridge |
| Teach-in recording | Press **● REC**, move the robot by hand — every settled pose becomes a line of `rbt` code, ready to replay |
| Waypoint sequencer | Record poses, play with pause/loop, save/load named sequences |

### Tools &amp; traces

| Feature | What it does |
|---|---|
| Tool / TCP offsets | Named end-of-arm tools (mm offsets); FK, IK, traces and the gizmo all follow the active TCP |
| TCP traces | Colored tool-center-point trajectories drawn in the viewport |

### Vision &amp; autonomy — validated in simulation, returning with the depth camera

> These camera-derived tools were built and validated against the MuJoCo render, then **parked behind a "Camera tools — under development" stub** in v0.8.0 — the live camera must be a *real* connected depth sensor, not a rendered one, so they re-enable when the hardware arrives (the vision backend stays in the tree as reference). The sim numbers below are real: they are sim-validated, not live cockpit toggles today.

| Capability (sim-validated) | What it does |
|---|---|
| On-board camera | A camera view rendered from the model (MuJoCo) and streamed into the cockpit (MJPEG), switchable between viewpoints |
| Work-camera point cloud | A **PCL** toggle back-projects the work camera's depth into the twin — a coloured 3D point cloud of what it sees (table, target), the input the grasp planner consumes |
| Vision-guided pick | **DETECT** finds the workspace target and back-projects its centroid to a world pose (~2 mm vs ground truth); **PICK** drives the right arm to it through the same IK + collision guard and closes the gripper |
| Smart pick (multi-object) | A **GRASP** toggle synthesises a top-down parallel-jaw grasp on the point cloud for **every** object (RANSAC removes the table, clusters the rest, fits a grasp — centre, *measured* height, footprint, yaw, width check — to each object's own geometry, rejecting the robot's own limbs). A pluggable detector labels each by **colour + shape** (opt-in YOLO backend for real objects); an object selector + **SMART** pick the chosen one by name through the IK + guard |
| Closed-loop visual servoing | **SERVO** locks the gripper onto the target *in image space* as it descends — robust to camera-calibration error (open-loop misses ~43 mm, IBVS ~5 mm in sim) |

### Safety &amp; modes

| Feature | What it does |
|---|---|
| Collision guard | Every target checked for self-collision *before* it is sent — including along interpolated paths; capsule / box collision model |
| Contact reflex | A torque spike on a *stalled* arm joint (loaded but not moving — i.e. pushing into something) latches a soft-stop; clear it from the **CONTACT** chip |
| Planned routing | When a straight move (**Home** or a **waypoint** goto/play) would clip a self-collision, an RRT planner routes the arms *around* it (collision-free) instead of stalling — the legs / balance chain are left untouched |
| SIM / REAL toggle | Same protocol either way; switching always re-latches the E-STOP |

### Observability &amp; operator tools

| Feature | What it does |
|---|---|
| Live telemetry plots | Foxglove-style scrolling strip charts (joint angle / velocity / temperature / TCP / link RTT) at 30 Hz — colour-coded legend, click-to-toggle lines, pause, current-value markers |
| Live TF frame tree | RViz2-style transform tree (world ▸ base_link ▸ arm flanges) with world-mm readouts and eye-toggled RGB axis triads that track the kinematics |
| Diagnostics panel | RViz `robot_monitor`-style status tree (system link, E-STOP, overtemp, guard, contact, RTT + per-joint temp / vel / load) with OK / warn / error dots and a worst-status badge |
| Joint-limit meters | Each joint's slider edge and value tint amber near a limit (red at the hard stop), with an amber bounding box on the link in 3D |
| Collision-mesh display | A collision-mesh toggle (key **B**) renders the guard's actual capsule / box model in 3D and reddens any contacting pair — see exactly what the guard sees |
| TCP-force overlay | A TCP-force toggle (key **F**) draws a per-arm end-effector force arrow estimated from the joint torques (`(J·Jᵀ)⁻¹·J·τ`), low-pass filtered, amber when straining (> 12 N) |
| Trajectory replay + scrub | A 45 s rolling record of joint motion with a scrubber and Play — drag to freeze the twin at any past instant; an amber playhead tracks it on the strip charts |
| CSV export | One-click **↓ CSV** of the current plot signal or the full 26-DoF recorded trajectory (degrees, real timestamps) |
| Global speed override | A **SPD** slider scales all motion server-side — jog and every glide (home, sequences, RRT routes) |
| Sim transport &amp; inspection | Play / Pause / Step / Reset of the autonomous motion with a run clock; a two-point **measure** tool; a viewport **stats HUD** (FPS / draw-calls / triangles); **Stage search** + a 3D selection outline |

### Scene, markers &amp; planning

| Feature | What it does |
|---|---|
| Stage hierarchy &amp; inspector | An Isaac-Sim-style **STAGE** tree (World ▸ Skate ▸ arms ▸ joints + overlays / grid) with visibility eyes; click any node for a live **PROPERTY** inspector (name, type, world pose) |
| Viewport display settings | A gear popover toggles grid / axes, sets camera FOV, swaps the background, and flips render quality |
| Scene markers | Spawn a target in reachable space and drag its X/Y/Z gizmo; each marker shows live **reachability** (green / red), one-click **→L / →R** go-to (server-side IK), **→P** to append `rbt.moveto(…)` to a program, and **⇄ both** for a simultaneous **bimanual reach** |
| Virtual obstacles | Spawn keep-out boxes and place them freely with a 3D gizmo, sized to any W×D×H — the RRT planner and the collision guard route the arms *around* them |
| Planning preview | Before a **Home** or **waypoint** move, a translucent **ghost robot** shows the destination pose and a blue trail shows the planned collision-free **route**, gated behind **Approve / Cancel** |
| Save / load scene | Save the placed markers + obstacles to a JSON scene file and reload them later |

</details>

<div align="center">
  <img src="docs/img/cockpit_v0724_cockpit.webp" width="720px" alt="The Skate Commander cockpit (v0.8.0): an Isaac-Sim-style workstation — menu bar, tool rail, 3D twin, STAGE / PROPERTY dock and live telemetry plots">
  <br>
  <em><strong>v0.8.0 cockpit</strong> — an Isaac-Sim-style workstation: a menu bar, a left tool rail, the 3D MuJoCo twin, a STAGE / PROPERTY dock and live telemetry plots. Mirror mode, dual-arm carry, jerk-limited motion and teach-in all live here. <strong><a href="https://raw.githack.com/dsl-robotics/skatearm/main/tools/skate_commander/preview.html">▶ Live preview</a></strong> (recorded telemetry, no install) · full docs: <a href="tools/skate_commander/">tools/skate_commander/</a></em>
</div>

## 🔌 skate_ros2 — the wire

A ROS 2 driver over Skate's **native UDP protocol** (documented packet layout, deadman semantics, 26-DoF ordering) plus a **MuJoCo sim endpoint speaking the same protocol** — develop your stack before the robot arrives, then swap `127.0.0.1` for `r.local`. Safety mirrors the firmware: arm-at-measured-pose, command-freshness deadman, 58 °C overtemp latch. 17 unit tests run without ROS; end-to-end verified over real sockets.

<div align="center">
  <img src="docs/img/ros2_wire_demo.gif" width="560px" alt="skate_ros2 wire demo: client teleoperates the MuJoCo endpoint over real UDP; at t=11s the client goes silent and the watchdog dampens the robot">
  <br>
  <em>A scripted client drives the MuJoCo endpoint over <strong>real UDP packets</strong>. At t = 11 s it goes silent — the watchdog dampens the robot.
  HD video: <a href="docs/video/ros2_wire_demo.mp4">ros2_wire_demo.mp4</a></em>
</div>

| On the wire (sim endpoint) | Result |
|---|---|
| Command rate | 60 Hz sustained (configured target) |
| Telemetry | ~190 packets/s |
| Tracking error | 0.015 rad (vs the MuJoCo model) |
| Watchdog dampen after silence | < 0.3 s (configured timeout) |

*These are sim-endpoint figures: command rate and watchdog timeout are configured targets confirmed in simulation, and tracking error is against the MuJoCo model. Real-hardware numbers come once the Skate reaches Riga.*

<div align="center">
  <img src="docs/img/ros2_wire_stats.png" width="560px" alt="Wire statistics: packet rates and joint tracking during the demo">
  <br>
  <em>Rates & tracking from the demo run · full docs: <a href="tools/skate_ros2/">tools/skate_ros2/</a></em>
</div>

## 🏭 Autonomous work-cell (Phase 1 — complete)

The demonstrator task, end to end in simulation: the left arm fixtures a base part in the air, the right arm aligns a peg by relative servoing and inserts it with a force-guarded descent. A GRAFCET sequencer (the IEC step-sequencer standard used in industrial soft-PLCs) runs the full cycle on sensor-based transitions — no timers — and two fixed cameras with classical CV deliver the accept/reject verdict that drives it. Every transition is logged to JSON and fed into a Flask + SQLite SCADA dashboard.

<div align="center">
  <img src="docs/img/cell_assemble_demo.gif" width="420px" alt="Bimanual assembly: left arm fixtures the base, right arm inserts the peg with a force-guarded descent">
  <img src="docs/img/14_qc_top_annotated.png" width="420px" alt="Overhead QC camera view, annotated: inspection window, pocket-rim reference, measured alignment">
  <br>
  <em>Left: the bimanual insert (τ-watchdog guarded, depth 18.5 mm, peg tilt ≤ 2°). Right: the overhead QC camera's annotated verdict.
  HD video: <a href="docs/video/cell_cycle_demo.mp4">cell_cycle_demo.mp4</a> · <a href="docs/video/cell_assemble_demo.mp4">cell_assemble_demo.mp4</a></em>
</div>

| Key number | Result |
|---|---|
| Cycle time | **42.4 s** (takt target ≤ 60 s) |
| QC residual, alignment (camera vs sim oracle) | ±1.3 mm |
| QC residual, insertion depth | ±3.4 mm |
| Accept rate | functional — only 2 cycles logged so far (sample too small for a true rate; tracked live on the dashboard) |

Dashboard live previews: **[overview](https://raw.githack.com/dsl-robotics/skatearm/main/dashboard/preview_overview.html)** · **[cycle detail](https://raw.githack.com/dsl-robotics/skatearm/main/dashboard/preview_cycle.html)** — code in [dashboard/](dashboard/), sequencer in [sim/sequencer.py](sim/sequencer.py), QC in [sim/qc.py](sim/qc.py).

## 🦾 Sim foundations (Phase 0)

The converted official `skt_v3` model ships with no actuators — [sim/make_control_model.py](sim/make_control_model.py) adds 26 position servos and holds poses under physics with < 0.03 rad error; [sim/make_collision_model.py](sim/make_collision_model.py) replaces the jamming raw meshes with auto-fitted collision capsules (boxes via `--boxes`), so self-collision actually works. Joint/torque sensors and end-effector sites seed the telemetry schema ([tracking plot](docs/img/sensor_tracking.png)). Honest limitations documented in [sim/README.md](sim/README.md).

<div align="center">
  <img src="docs/img/control_demo.gif" width="360px" alt="Closed-loop control demo: independent arm trajectories under physics">
  <img src="docs/img/collision_demo.gif" width="360px" alt="Self-collision demo: hands meet and stop; orange boxes are the generated collision layer">
  <br>
  <em>Left: closed-loop control under physics. Right: hands meet and <strong>stop</strong> — orange boxes are the collision layer.
  HD video: <a href="docs/video/control_demo.mp4">control_demo.mp4</a> · <a href="docs/video/collision_demo.mp4">collision_demo.mp4</a></em>
</div>

## 🏗 Architecture

```mermaid
flowchart TB
    subgraph cell [SkateArm work-cell]
        direction TB
        SEQ[Sequencer\nGRAFCET / soft-PLC] --> MOT[Motion layer\nROS 2 + MoveIt, 2 arms]
        SEQ --> FEED[Feeder node\nAVR]
        MOT --> SKATE[Skate 16 DoF\nMuJoCo twin → real robot]
        POL[Manipulation policies\nACT / SmolVLA via LeRobot] --> MOT
        SKATE --> QC[QC station\nGD&T accept/reject]
        QC --> DASH[SCADA dashboard\nFlask + SQL]
        SEQ --> DASH
    end
    CAM[2x cameras] --> POL
    CAM --> QC
```

**Demonstrator task:** one arm holds/fixtures a part, the other inserts (peg-in-hole class), then in-cell measurement decides accept/reject and logs to the dashboard. The real Skate (16 DoF, span 1615 mm, RPi 5, UDP control) is en route to Riga — Phase 2 starts on arrival; `skate_ros2` is already waiting for it.

Full architecture & mapping of all 12 prior portfolio projects onto subsystems: [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md). Phased plan: [docs/ROADMAP.md](docs/ROADMAP.md).

## 🚀 Quick start (simulation)

```bash
git clone https://github.com/Rbotic/skate_teleop.git   # official model (skt_v3)
pip install mujoco numpy imageio
python sim/render_skate.py --model path/to/skate_teleop/skt_v3         # static renders
python sim/make_control_model.py path/to/skate_teleop/skt_v3           # + actuators & sensors
python sim/make_collision_model.py path/to/skate_teleop/skt_v3         # + collision capsules
python sim/demo_wave.py --model path/to/skate_teleop/skt_v3            # control demo (mp4/gif)
python sim/demo_selfcollision.py --model path/to/skate_teleop/skt_v3   # self-collision demo
python sim/telemetry_demo.py --model path/to/skate_teleop/skt_v3       # tracking/torque plot
```

> **Windows:** use `py` instead of `python`/`python3` (the bare names may open the Microsoft Store stub).

Each script is documented in [sim/README.md](sim/README.md). To drive the twin from a browser, follow the [Commander quick start](tools/skate_commander/#quick-start-no-hardware).

## 🧰 Community tools

Tools get built because SkateArm needs them — then released standalone:

| Tool | What it is | Status |
|---|---|---|
| [`skate_ros2`](tools/skate_ros2/) | ROS 2 bridge over Skate's native UDP + protocol-true MuJoCo sim endpoint | ✅ **shipped** (sim-verified; MoveIt config next) |
| [`skate_commander`](tools/skate_commander/) | Web cockpit — browser digital twin, jog/sliders, **drag-IK**, cartesian jog, **mirror mode**, **dual-arm carry**, **singularity (SING) warning**, **jerk-limited motion**, **manipulability heat-map (DEX)**, **Python programs with Click-to-Step + control flow**, **teach-in recording**, waypoint **sequencer**, TCP tools & traces, **contact reflex**, smooth **Home** + **waypoint moves** that **plan around self-collisions (RRT)**, **virtual keep-out obstacles**, **scene markers** with reachability + go-to + **bimanual reach**, **planning preview** (ghost robot + route trail), **collision-mesh display**, **TCP-force overlay**, **diagnostics panel**, **joint-limit meters**, **trajectory replay + scrub**, **CSV export**, **drive / motion tuning**, **save / load scene**, **ghost-robot move preview + approval gating**, **collision guard**, **keyboard / screen-reader a11y**, **operator hotkeys + legend**, **Isaac-Sim-style application shell** (menu bar, Stage hierarchy + inspector, Property panel, timeline, nav gizmo), **live telemetry plots** (Foxglove-style), **live TF frame tree + 3D axis triads** (RViz-style), **global speed override**, **sim transport**, **measure tool**, **viewport stats HUD + display settings**, **Stage search + selection outline** · _(sim-validated camera tools — point cloud, smart-pick, visual servoing — are parked under "Camera tools: under development" pending a real depth camera)_ · [live preview](https://raw.githack.com/dsl-robotics/skatearm/main/tools/skate_commander/preview.html) | ✅ **v0.8.0** (real-camera passthrough waits for hardware) |
| Control-ready MJCF | skt_v3 with actuators, ready for control work | ✅ first version in [sim/](sim/) |
| Teleop dataset hub | Bimanual datasets in LeRobot format | planned |
| MuJoCo benchmark suite | Repeatable bimanual tasks for policy comparison | planned |
| URDF/config validator | Sanity-check tool for Skate configs | planned |
| Getting-started handbook | From unboxing to first teleop | planned |

Ideas and requests from other Skate owners are welcome — open an issue.

**Why this project:**
1. **Level up in robotics** — from a single SO-101 arm ([previous project](https://github.com/Lavs-Daniels-Skots-231RMC173/so101-native-ubuntu-ros2-moveit)) to a bimanual humanoid: two-arm coordination, sim-to-real.
2. **Learn by building** — ROS 2, MuJoCo, policy learning (ACT/SmolVLA), classical control, embedded in one system.
3. **Give back to the Skate community** — first-mover window to publish open tools, datasets and guides others can build on.

## 🔗 Related projects

- **[SO-101 · ROS 2 + MoveIt real-hardware bring-up](https://github.com/Lavs-Daniels-Skots-231RMC173/so101-native-ubuntu-ros2-moveit)** — a real SO-101 / SO-ARM101 arm pair brought up on ROS 2 Jazzy + MoveIt + LeRobot (a 2-camera ACT policy trained and published to Hugging Face).
- **[Engineering Portfolio](https://github.com/Lavs-Daniels-Skots-231RMC173/engineering-portfolio)** — 11 academic & applied projects: industrial robotics, PLC, embedded systems, metrology, CNC, mechanical design.

## Author

**Daniels Skots Lavs** — mechatronics student (RTU), industrial electronics technician.
📍 Riga / EU · **open to junior robotics software roles**
[GitHub profile](https://github.com/Lavs-Daniels-Skots-231RMC173) · [Engineering portfolio](https://github.com/Lavs-Daniels-Skots-231RMC173/engineering-portfolio) · porche121004@gmail.com

## License

MIT — see [LICENSE](LICENSE). The `skt_v3` model and meshes belong to [Rbotic/skate_teleop](https://github.com/Rbotic/skate_teleop) and are **not** redistributed here.

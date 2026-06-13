# SkateArm

**An open bimanual work-cell and tool ecosystem for the [R.Botic Skate](https://www.rboticlabs.com/shop/p/skate-upper-body-v2) robot — two-handed assembly with in-cell quality inspection, built sim-first in MuJoCo, then deployed on the real Skate.**

<div align="center">

[![License: MIT](https://img.shields.io/badge/license-MIT-green?style=flat)](LICENSE)
[![MuJoCo](https://img.shields.io/badge/sim-MuJoCo%203.x-orange?style=flat)](sim/)
[![ROS 2](https://img.shields.io/badge/bridge-ROS%202-22314E?style=flat)](tools/skate_ros2/)
[![Python](https://img.shields.io/badge/python-3.x-3776AB?style=flat)](sim/)

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

## 🕹 Skate Commander — web cockpit

A browser cockpit for the Skate: a 3D digital twin built from the official URDF, driven over the **same UDP wire** the real robot speaks. Starts E-stopped, arms at the robot's measured pose, deadman drops in 0.3 s if the tab closes.

| Feature | What it does |
|---|---|
| Jog + sliders | Hold −/+, drag the thumb, or jump straight to a limit; amber = your command, azure = actual position |
| Cartesian jog | Step the TCP along world X/Y/Z in mm — server-side IK, auto-stops on arrival |
| Drag-IK | Grab a wrist sphere in 3D — server-side DLS IK (damped least squares inverse kinematics) glides all 7 arm joints |
| Mirror mode | Bimanual: jog/slider/IK on one arm is reflected onto the other — the sign map is *measured* from the model's FK, not guessed |
| Python programs | Built-in editor + `rbt` API (`movej`/`pose`/`movel`/`home`/waypoints); **Click-to-Step** runs one motion at a time; E-STOP or any manual input kills the program |
| Teach-in recording | Press **● REC**, move the robot by hand — every settled pose becomes a line of `rbt` code, ready to replay |
| Waypoint sequencer | Record poses, play with pause/loop, save/load named sequences |
| Tool / TCP offsets | Named end-of-arm tools (mm offsets); FK, IK, traces and the gizmo all follow the active TCP |
| TCP traces | Colored tool-center-point trajectories drawn in the viewport |
| Collision guard | Every target checked for self-collision *before* it is sent — including along interpolated paths; capsule-fitted collision model |
| SIM / REAL toggle | Same protocol either way; switching always re-latches the E-STOP |

<div align="center">
  <img src="docs/img/commander_v06_overview.gif" width="720px" alt="Skate Commander overview: mirror-mode bimanual jog raises both arms from one slider with TCP traces, then teach-in — move the arm by hand and every settled pose becomes a movej line; RUN replays the self-written program through the guarded bridge">
  <br>
  <em><strong>v0.6 overview</strong>: <strong>mirror mode</strong> drives both arms from one slider (note the symmetric TCP traces), then <strong>teach-in</strong> — press ● REC, move the arm by hand, and every settled pose becomes a <code>movej</code> line. ▶ RUN replays the self-written program through the same collision-guarded bridge.</em>
  <br><br>
  <img src="docs/img/commander_v05_live.gif" width="720px" alt="Skate Commander v0.5 in action: RESUME, mirror-mode bimanual jog, cartesian TCP steps, then a Python program executed with Click-to-Step — the collision guard blocks two of its moves">
  <br>
  <em>v0.5 live: mirror-mode jog, cartesian TCP steps, then a Python program stepped command-by-command — watch the guard veto two of its moves. <strong><a href="https://raw.githack.com/Lavs-Daniels-Skots-231RMC173/skatearm/main/tools/skate_commander/preview.html">▶ Live preview</a></strong> (recorded telemetry, no install) · full docs: <a href="tools/skate_commander/">tools/skate_commander/</a></em>
</div>

## 🔌 skate_ros2 — the wire

A ROS 2 driver over Skate's **native UDP protocol** (documented packet layout, deadman semantics, 26-DoF ordering) plus a **MuJoCo sim endpoint speaking the same protocol** — develop your stack before the robot arrives, then swap `127.0.0.1` for `r.local`. Safety mirrors the firmware: arm-at-measured-pose, command-freshness deadman, 58 °C overtemp latch. 17 unit tests run without ROS; end-to-end verified over real sockets.

<div align="center">
  <img src="docs/img/ros2_wire_demo.gif" width="560px" alt="skate_ros2 wire demo: client teleoperates the MuJoCo endpoint over real UDP; at t=11s the client goes silent and the watchdog dampens the robot">
  <br>
  <em>A scripted client drives the MuJoCo endpoint over <strong>real UDP packets</strong>. At t = 11 s it goes silent — the watchdog dampens the robot.
  HD video: <a href="docs/video/ros2_wire_demo.mp4">ros2_wire_demo.mp4</a></em>
</div>

| Verified on the wire (sim endpoint) | Result |
|---|---|
| Command rate | 60 Hz sustained |
| Telemetry | ~190 packets/s |
| Tracking error | 0.015 rad |
| Watchdog dampen after silence | < 0.3 s |

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
| Accept rate | 100 % so far (2 logged cycles — small sample, tracked live on the dashboard) |

Dashboard live previews: **[overview](https://raw.githack.com/Lavs-Daniels-Skots-231RMC173/skatearm/main/dashboard/preview_overview.html)** · **[cycle detail](https://raw.githack.com/Lavs-Daniels-Skots-231RMC173/skatearm/main/dashboard/preview_cycle.html)** — code in [dashboard/](dashboard/), sequencer in [sim/sequencer.py](sim/sequencer.py), QC in [sim/qc.py](sim/qc.py).

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
| [`skate_commander`](tools/skate_commander/) | Web cockpit — browser digital twin, jog/sliders, **drag-IK**, cartesian jog, **mirror mode**, **Python programs with Click-to-Step**, **teach-in recording**, waypoint **sequencer**, TCP tools & traces, **collision guard** · [live preview](https://raw.githack.com/Lavs-Daniels-Skots-231RMC173/skatearm/main/tools/skate_commander/preview.html) | ✅ **v0.6** (camera passthrough waits for hardware) |
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

## Author

**Daniels Skots-Lavs** — mechatronics student (RTU), industrial electronics technician.
[GitHub profile](https://github.com/Lavs-Daniels-Skots-231RMC173) · [Engineering portfolio](https://github.com/Lavs-Daniels-Skots-231RMC173/engineering-portfolio) · porche121004@gmail.com

## License

MIT — see [LICENSE](LICENSE). The `skt_v3` model and meshes belong to [Rbotic/skate_teleop](https://github.com/Rbotic/skate_teleop) and are **not** redistributed here.

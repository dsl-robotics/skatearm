<p align="center">
  <img src="../../docs/img/skate_commander_lockup.png" width="640" alt="Skate Commander — web cockpit, digital twin, real robot">
</p>

# Skate Commander — web cockpit for the Skate digital twin & robot

Drive the [SkateArm](../../README.md) digital twin from a browser — and, when
the hardware lands, flip one switch and drive the real Skate over the exact
same UDP wire. Functional reference: PAROL6's Waldo Commander; the design,
the bimanual specifics and the safety model are our own.

**[▶ Live preview](https://raw.githack.com/dsl-robotics/skatearm/main/tools/skate_commander/preview.html)** —
recorded telemetry playback, no install (simplified stick-figure twin there:
Rbotic's STL meshes are only loaded from your local clone, never
redistributed).

<div align="center">
  <img src="../../docs/img/commander_v06_overview.gif" width="680px" alt="Commander overview: mirror-mode bimanual jog, then teach-in writes a movej program from hand-moved poses and replays it through the guarded bridge">
  <br>
  <em><strong>v0.6 overview</strong> — mirror-mode bimanual jog (both arms from one slider), then <strong>teach-in</strong>: move the arm by hand and the program writes itself as <code>movej</code> lines; ▶ RUN replays it through the same collision-guarded bridge.</em>
</div>

<div align="center">
  <a href="https://github.com/dsl-robotics/skatearm/blob/main/docs/video/commander_v06_product.mp4">
    <img src="../../docs/img/commander_v06_video_poster.png" width="680px" alt="Watch the Skate Commander product video">
  </a>
  <br>
  <em>▶ <strong>Watch the 50-second product tour</strong> — captions, music, and zoom-to-control highlights (click to play on GitHub).</em>
</div>

<div align="center">
  <img src="../../docs/img/commander_v06_pick.gif" width="680px" alt="Vision-guided pick: the work camera detects the magenta target and the right arm is driven to it by IK">
  <br>
  <em><strong>Vision-guided pick</strong> — <strong>DETECT</strong> finds the target (back-projected to a world pose ~2&nbsp;mm from the simulator's ground truth), <strong>PICK</strong> drives the right arm to it through the IK + collision guard.</em>
</div>

## Features (v0.6)

* **3D digital twin** built in-browser from the official `skt_v3.urdf`
  (Three.js; kinematic math validated against MuJoCo to < 0.001 mm; URDF
  material colors)
* **Joint control four ways** — hold −/+ to jog, **drag the slider thumb**
  (amber thumb = your command, azure fill = actual position), jump straight
  to a joint limit (⇤ ⇥, guard permitting), or grab a wrist sphere and
  **drag in 3D**: server-side damped-least-squares IK glides all 7 arm
  joints (pure numpy, 0.15 ms/step)
* **Cartesian jog** — step the TCP along world X/Y/Z (1–50 mm, hold to
  repeat) with a live TCP readout; the IK target auto-clears on arrival, or
  when it stops improving (out of reach / guard-blocked). While any IK
  target is active the redundant arm **chooses its own elbow**: a null-space
  comfort objective (documented default pose blended toward joint-range
  centers) continuously relaxes the posture *without moving the TCP* — no
  winding on out-and-back jogs, no limit-hugging
* **Mirror mode** — bimanual jog: jog/slider/IK input on one arm is
  reflected onto the other. The per-joint sign map and the mirror axis are
  **measured numerically from the model's FK at startup**, not assumed from
  URDF conventions
* **Python programs** — in-browser editor over a sandboxed `rbt` API
  (`movej`, `movel`, `home`, `gripper`, `waypoint`, `wait`, `tcp`, `q`,
  `status`; `print` goes to the cockpit log). **Click-to-Step** executes one
  motion at a time, showing the next command and its source line; RUN
  releases the program. Every motion uses the same bridge paths as the UI —
  limits, collision guard, E-STOP — and any manual input kills the program.
  Save/load to `programs/*.py`
* **Natural-language programs** — describe a task in plain English (e.g.
  *"raise both arms, then home"*) and a safe parser writes the `rbt` program
  into the editor; it never moves the robot directly — you review it, then
  Click-to-Step or RUN it through the same guarded bridge. Runs fully offline
  (a deterministic intent parser + an AST validator that only ever emits known
  `rbt` calls); an optional LLM fallback engages only if an API key is set
* **On-board camera + vision-guided pick** — the server renders a workspace
  camera from the model (MuJoCo) and streams it into the cockpit (MJPEG,
  switchable views). **DETECT** finds the magenta target and back-projects its
  centroid to a world pose (camera intrinsics from `fovy`, extrinsics from
  `cam_xpos`/`cam_xmat`, intersected with the table plane — validated to ~2 mm
  against the simulator's true object position); **PICK** drives the right arm
  to it via the same DLS-IK + collision guard and closes the gripper
* **Teach-in recording** — press **● REC** and just move the robot
  (sliders, jog, gizmo, cartesian steps): every settled pose becomes a line
  of `rbt` code — `movej` for one joint, a coordinated `pose({...})` for
  several. Stop, and the generated program lands in the editor ready to
  RUN. A red REC chip in the top bar shows the pose count from any tab
* **Tool / TCP offsets** — named end-of-arm tools (mm offsets in the wrist
  frame, persisted in `tcp_tools.json`); FK, IK, the drag-gizmo, traces and
  the cartesian readout all follow the active TCP per arm
* **Waypoint sequencer** — record poses, glide through them with pause/loop,
  jump to any step, save/load named sequences (`sequences/*.json`); any
  manual input or E-STOP interrupts playback
* **TCP traces** — colored tool-point trajectories in the viewport
  (toggle/clear)
* **Collision guard** — every candidate target (jog, slider, IK, cartesian,
  sequencer, programs) is checked for self-collision *before it is sent*;
  large jumps are checked along the interpolated path (no tunneling). The
  guard sees hand↔leg pairs the physics model deliberately excludes. The
  collision model now fits **capsules** instead of AABB boxes — far fewer
  false positives on the slim wrist links (`--boxes` restores v0.4 behavior)
* **SIM / REAL toggle** — the same `skate_ros2` UDP protocol either way;
  switching always re-latches the E-STOP; the lower chain is locked in REAL
  **at the bridge**, not just greyed out in the UI

## Safety model

Starts **estopped**; RESUME is an explicit human action. Arms at the robot's
**measured pose** (no jump on connect; early commands are ignored). Close the
tab → deadman drops in 0.3 s (firmware watchdog semantics). Joint limits are
clamped at the bridge; self-colliding targets never leave the server; the
lower chain is locked in REAL mode. Overtemp (58 °C) latches a whole-body
dampen.

## Quick start (no hardware)

> **Windows:** use `py` wherever you see `python3` (bare `python`/`python3`
> may open the Microsoft Store stub).

```bash
# the official robot model, if you don't have it yet:
git clone https://github.com/Rbotic/skate_teleop.git

cd tools/skate_commander       # commands below run from THIS folder
pip install -r requirements.txt mujoco
python3 ../../sim/make_control_model.py   /path/to/skate_teleop/skt_v3
python3 ../../sim/make_collision_model.py /path/to/skate_teleop/skt_v3

python3 -m skate_commander.server \
    --model-dir       /path/to/skate_teleop/skt_v3 \
    --spawn-sim       /path/to/skate_teleop/skt_v3/skt_v3_collision.xml \
    --collision-model /path/to/skate_teleop/skt_v3/skt_v3_collision.xml
# open http://127.0.0.1:8088 → RESUME → jog / drag / record
```

With a real Skate: leave out `--spawn-sim`, flip the toggle to REAL
(`--real-host` overrides `r.local`). Keep `--collision-model` — it protects
the real robot too.

## Programming the robot

The **PROG** tab is a tiny Python environment over the same bridge (degrees
for joints, millimeters for cartesian, world axes):

```python
rbt.home()
rbt.movej("L4", 60)            # left elbow — "L1".."L8", "R…", "H…", or index
rbt.pose({"L2": 25, "R2": 25}) # several joints as ONE coordinated move
for d in (40, 80, 40):
    rbt.movej("R4", d)         # wave
rbt.movel("right", dz=60)      # TCP up 60 mm (server-side IK)
rbt.gripper("right", 30)
print("tcp:", rbt.tcp("right"), "mm")
```

Too lazy to type? Press **● REC**, drive the robot by hand, press it again —
the program writes itself from your settled poses and appends to the editor.

**⏭ STEP** (Click-to-Step) pauses before every motion command and shows the
next call + its source line; **▶ RUN** releases it. The runner is a worker
thread whose every motion goes through the bridge — collision guard, joint
limits, REAL-mode leg lock and E-STOP included; touching any manual control
stops the program. MIRROR mode applies to program moves too (it's a bridge
mode, not a UI gimmick). The sandbox has no imports or file access (`math` and
`rbt` are provided) — it's a convenience layer for a local tool, not a
security boundary.

## Architecture

```
browser (Three.js twin · sliders · gizmo · sequencer UI)
   │ WebSocket: telemetry ↓20 Hz · commands ↑
FastAPI server (skate_commander.server)
   │ RobotBridge: arming · jog/slider/IK/sequencer @60 Hz
   │              estop · overtemp · collision guard · SIM/REAL
   │ numpy kinematics (FK = MuJoCo ±0; DLS IK)
   │ skate_ros2.SkateLink — the native UDP wire
   ▼
MuJoCo sim endpoint (SIM)  /  real Skate (REAL)
```

## Tests

`test/` runs headless — every file is a plain script, **no pytest needed**:

```bash
# from tools/skate_commander; Windows: py instead of python3
SKT_DIR=/path/to/skt_v3 SKATE_MJCF=/path/to/skt_v3/skt_v3_control.xml \
    python3 test/test_kinematics.py     # FK vs MuJoCo + tool offsets
# likewise: test_urdf.py · test_bridge.py (cart-step & mirror e2e) ·
#           test_ws_e2e.py · test_guard.py · test_program.py
```

Covered: URDF parsing, the bridge safety cycle, FK/IK and tool offsets vs
MuJoCo, cartesian step + mirror reflection over real UDP, the full
WebSocket→UDP→MuJoCo loop, sequencer e2e, collision-guard e2e (leg coverage,
anti-tunneling), and the program runner (Click-to-Step, STOP, E-STOP abort,
sandbox). The whole suite runs on a plain Windows + Python 3.13 machine — no
ROS anywhere in the stack.

## Roadmap

Camera passthrough and real-gripper tool presets wait for the hardware
(Phase 2). Graduates to its own repo once it's daily-drivable.

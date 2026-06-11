# Skate Commander — web cockpit for the Skate digital twin & robot

Drive the [SkateArm](../../README.md) digital twin from a browser — and, when
the hardware lands, flip one switch and drive the real Skate over the exact
same UDP wire. Functional reference: PAROL6's Waldo Commander; the design,
the bimanual specifics and the safety model are our own.

**[▶ Live preview](https://raw.githack.com/Lavs-Daniels-Skots-231RMC173/skatearm/main/tools/skate_commander/preview.html)** —
recorded telemetry playback, no install (simplified stick-figure twin there:
Rbotic's STL meshes are only loaded from your local clone, never
redistributed).

## Features (v0.4)

* **3D digital twin** built in-browser from the official `skt_v3.urdf`
  (Three.js; kinematic math validated against MuJoCo to < 0.001 mm; URDF
  material colors)
* **Joint control three ways** — hold −/+ to jog, **drag the slider thumb**
  (amber thumb = your command, azure fill = actual position), or grab a
  wrist sphere and **drag in 3D**: server-side damped-least-squares IK glides
  all 7 arm joints (pure numpy, 0.15 ms/step)
* **Waypoint sequencer** — record poses, glide through them with pause/loop,
  jump to any step, save/load named sequences (`sequences/*.json`); any
  manual input or E-STOP interrupts playback
* **TCP traces** — colored wrist trajectories in the viewport (toggle/clear)
* **Collision guard** — every candidate target (jog, slider, IK, sequencer)
  is checked for self-collision *before it is sent*; large jumps are checked
  along the interpolated path (no tunneling). The guard sees hand↔leg pairs
  the physics model deliberately excludes, tolerating only the contacts that
  exist at the neutral pose
* **SIM / REAL toggle** — the same `skate_ros2` UDP protocol either way;
  switching always re-latches the E-STOP

## Safety model

Starts **estopped**; RESUME is an explicit human action. Arms at the robot's
**measured pose** (no jump on connect; early commands are ignored). Close the
tab → deadman drops in 0.3 s (firmware watchdog semantics). Joint limits are
clamped at the bridge; self-colliding targets never leave the server; the
lower chain is locked in REAL mode. Overtemp (58 °C) latches a whole-body
dampen.

## Quick start (no hardware)

```bash
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

`test/` runs headless: URDF parsing, bridge safety cycle, kinematics vs
MuJoCo, full WebSocket→UDP→MuJoCo e2e, sequencer e2e, collision-guard e2e
(including leg coverage and anti-tunneling). The guard suite was verified on
a plain Windows + Python 3.13 machine — no ROS anywhere in the stack.

## Roadmap

Tool/TCP-offset manager and camera passthrough wait for the real gripper and
hardware (Phase 2). Graduates to its own repo once it's daily-drivable.

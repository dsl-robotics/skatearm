# tools — community tools track

Standalone, reusable tools for Skate owners. Built because SkateArm needs them; released so nobody has to rebuild them.

| Tool | Description | Status |
|---|---|---|
| [`skate_ros2`](skate_ros2/) | ROS 2 bridge over Skate's native UDP protocol + MuJoCo sim endpoint speaking the same wire format | ✅ **shipped** — sim-verified; real-hardware validation on arrival (Commander v0.8) |
| [`skate_commander`](skate_commander/) | Web cockpit: browser twin + joint jog + SIM/REAL switch over the same wire ([live preview](https://raw.githack.com/dsl-robotics/skatearm/main/tools/skate_commander/preview.html)) | ✅ **v0.7.9** — one-command launch, connection-loss detection, jog/sliders, drag-IK, cartesian jog, mirror mode, dual-arm carry, singularity (SING) warning, manipulability heat-map, jerk-limited motion (incl. eased Home + waypoint moves that plan around self-collisions via RRT), Python programs (Click-to-Step + autocomplete), natural-language programs, teach-in recording, sequencer, TCP tools & traces, closed-loop visual servoing (SERVO), contact reflex, collision guard, redesigned cockpit UI |
| `control-ready MJCF` | skt_v3 with actuators + sensors for control/RL work | planned |
| `dataset-hub` | Bimanual teleop datasets, LeRobot format | planned |
| `mujoco-benchmarks` | Repeatable bimanual tasks for policy comparison | planned |
| `config-validator` | URDF/config sanity checker | planned |
| `handbook` | Getting started: unboxing → first teleop | planned |

Each tool graduates to its own repo once it's usable; this folder hosts early prototypes.

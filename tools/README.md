# tools — community tools track

Standalone, reusable tools for Skate owners. Built because SkateArm needs them; released so nobody has to rebuild them.

| Tool | Description | Status |
|---|---|---|
| [`skate_ros2`](skate_ros2/) | ROS 2 bridge over Skate's native UDP protocol + MuJoCo sim endpoint speaking the same wire format | ✅ **shipped** — sim-verified, real-hardware validation in Phase 2 |
| [`skate_commander`](skate_commander/) | Web cockpit: browser twin + joint jog + SIM/REAL switch over the same wire ([live preview](https://raw.githack.com/Lavs-Daniels-Skots-231RMC173/skatearm/main/tools/skate_commander/preview.html)) | ✅ **v0.1** — jog/safety shipped; drag-IK & sequencer next |
| `control-ready MJCF` | skt_v3 with actuators + sensors for control/RL work | planned |
| `dataset-hub` | Bimanual teleop datasets, LeRobot format | planned |
| `mujoco-benchmarks` | Repeatable bimanual tasks for policy comparison | planned |
| `config-validator` | URDF/config sanity checker | planned |
| `handbook` | Getting started: unboxing → first teleop | planned |

Each tool graduates to its own repo once it's usable; this folder hosts early prototypes.

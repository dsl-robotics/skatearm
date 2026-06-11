# skate_ros2 — ROS 2 bridge for the R.Botic Skate

ROS 2 driver for the [R.Botic Skate](https://www.rboticlabs.com/shop/p/skate-upper-body-v2)
bimanual robot over its **native UDP protocol** — plus a **MuJoCo sim endpoint
that speaks the exact same protocol**, so you can develop and test your whole
ROS 2 stack before the robot is even out of the box.

First community tool from the [SkateArm](../../README.md) project.

![wire demo](../../docs/img/ros2_wire_demo.gif)

*A scripted client teleoperates the MuJoCo endpoint over real UDP packets —
60 Hz commands in, ~190 telemetry pkt/s out. At t=11 s the client goes silent
and the firmware-style deadman watchdog dampens the robot in 0.3 s.*

## Why

The Skate ships with a clean Python teleop example
([Rbotic/skate_teleop](https://github.com/Rbotic/skate_teleop)) but no ROS 2
integration. This package bridges the gap with three thin layers:

| Layer | File | Needs |
|---|---|---|
| Wire protocol client | `skate_ros2/protocol.py` | numpy only |
| Sim endpoint (fake robot) | `skate_ros2/sim_endpoint.py` | + mujoco |
| ROS 2 driver node | `skate_ros2/driver_node.py` | + ROS 2 |

The protocol and safety logic are pure Python and fully unit-tested without
ROS; the rclpy node is a thin shell around them.

## The wire protocol (documented)

Reverse-engineered from the official client and confirmed against the official
Skate docs. Everything is UDP to/from port **2000** on the robot (`r.local`):

**Telemetry** (robot → you, streamed to whoever spoke last):
`pickle.dumps((id, obj))` with the classes from the official
`shared_classes_def.py` (vendored here, Apache-2.0):

| id | object | content | internal rate |
|---|---|---|---|
| 0 | `motor_command` | commands actually sent to motors | 200 Hz |
| 1 | `motor_state` | raw pos/vel/torque/error/**temp** per motor | 200 Hz |
| 2 | `state_est` | calibrated dof pos/vel/torque + base estimates | 200 Hz |
| 3 | `INS_fusion_state` | IMU fusion: quat, gyro, acc, global pose | 400 Hz |
| 4 | controller states | internal | 200 Hz |

**Command** (you → robot): `pickle.dumps((5, payload))` where payload is

```python
(targ_pos,        # np.float64[26], radians, see ordering below
 vel_cmd,         # np.float64[3]: x vel, y vel, angular vel (base)
 height_cmd,      # float: crouch height
 (wb, la, ra))    # deadman flags: 0 = dampen (whole body / left / right arm)
```

**Keepalive / watchdog:** the robot streams to the last address it heard from;
the official client pings `b"yo"` every 0.3 s. If the robot hears *nothing*
for 0.3 s it assumes deadman `(0,0,0)` and dampens itself. Any packet resets
the watchdog; flags come from the last command.

**26-DoF ordering** — the URDF joint names encode the protocol index directly
(`a3_armL_a11` = index 11), so the mapping in `skate_ros2/names.py` is exact:

| indices | group | URDF names |
|---|---|---|
| 0–7 | lower chain (legs) | `a0` … `a7` |
| 8–15 | left arm (15 = gripper) | `a0_armL_a8` … `a7_armL_a15` |
| 16–23 | right arm (23 = gripper) | `a0_armR_a16` … `a7_armR_a23` |
| 24–25 | head | `a0_head_a24`, `a1_head_a25` |

> ⚠️ The wire format is Python pickle — deserializing it can execute code.
> That's the firmware's design; use it on a trusted LAN only (the official
> stack makes the same assumption).

## Quick start — no hardware, no ROS

```bash
# 1. generate the control-ready MJCF over your skate_teleop clone
python3 sim/make_control_model.py /path/to/skate_teleop/skt_v3

# 2. terminal 1: a "robot" appears on UDP :2000
python3 -m skate_ros2.sim_endpoint --model .../skt_v3_control.xml

# 3. terminal 2: stream a bimanual wave to it
python3 examples/wave_no_ros.py --host 127.0.0.1
```

Anything written against `127.0.0.1` here talks to the real robot by swapping
the host for `r.local`. That's the whole point.

## Quick start — ROS 2 (target: Jazzy)

> **Honesty note:** the wire protocol, sim endpoint, and all driver safety
> logic are verified by tests that run without ROS (rclpy is stubbed). A
> `colcon build` on a live ROS 2 install has **not** been run yet — that's
> the first task once a ROS 2 machine enters the loop.

```bash
mkdir -p ~/skate_ws/src
cp -r tools/skate_ros2 ~/skate_ws/src/
cd ~/skate_ws && colcon build && source install/setup.bash

# real robot:
ros2 launch skate_ros2 skate_bridge.launch.py
# or against the sim endpoint:
ros2 launch skate_ros2 skate_bridge.launch.py robot_host:=127.0.0.1
```

### Topics

| topic | type | dir | notes |
|---|---|---|---|
| `joint_states` | sensor_msgs/JointState | pub | calibrated 26-DoF pos/vel/effort, URDF names — feed `robot_state_publisher` + `skt_v3.urdf` for RViz |
| `skate/imu` | sensor_msgs/Imu | pub | INS passthrough |
| `skate/temperatures` | std_msgs/Float32MultiArray | pub | per-motor °C, protocol order |
| `skate/connected` | std_msgs/Bool | pub | telemetry freshness |
| `skate/joint_position_cmd` | sensor_msgs/JointState | sub | by-name, partial messages fine |
| `skate/joint_position_cmd_raw` | std_msgs/Float64MultiArray | sub | full 26-vector |
| `skate/cmd_vel` | geometry_msgs/Twist | sub | base velocity |
| `skate/height_cmd` | std_msgs/Float64 | sub | crouch height |
| `skate/estop` | std_msgs/Bool | sub | `true` = dampen, latched |

### Safety model (mirrors the firmware)

* Nothing is commanded until telemetry arrives; the driver then **arms at the
  robot's own measured pose** — a fresh bridge can never jump the robot.
  Joint commands received before arming are ignored (with a warning), so a
  robot that comes online late can't jump to a guessed base pose either.
* A stale `skate/cmd_vel` decays to zero after `cmd_timeout` — joint commands
  can't keep an old base velocity alive.
* Deadman flags are `(1,1,1)` only while your commands are fresher than
  `cmd_timeout` (0.3 s default). Stop publishing → the robot dampens, exactly
  like releasing the deadman button in the official VR teleop.
* Any motor over `overtemp_c` (58 °C, the PETG limit from the official docs)
  latches a whole-body dampen with 5 °C release hysteresis.
* `skate/estop` `true` dampens immediately and stays latched until `false`.

Parameters: `robot_host`, `robot_port`, `tx_rate` (60), `rx_rate` (60),
`cmd_timeout` (0.3), `auto_deadman` (true), `overtemp_c` (58.0).

## Sim endpoint: honest approximations

The emulator is faithful on the wire (port, packet layout, watchdog timing,
telemetry classes) but approximates the physics-side behavior:

* "dampen" freezes position targets at the current pose instead of dropping
  to damping-only torques;
* motor temperatures are **synthetic** (warm with |τ|, cool to 25 °C) so the
  temperature plumbing can be exercised;
* `vel_cmd` / `height_cmd` are accepted but ignored (fixed-base model);
* INS reports a static upright pose.

## Tests

```bash
python3 test/test_names.py              # ordering / CAN layout invariants
python3 test/test_protocol_loopback.py  # wire contract over localhost UDP
python3 test/test_driver_logic.py       # arming/deadman/estop/overtemp logic
                                        # (runs WITHOUT ROS via stubbed rclpy)
SKATE_MJCF=.../skt_v3_control.xml python3 test/test_e2e_sim.py  # full e2e
```

Verified end-to-end in CI-like conditions: 60 Hz commands, ~190 telemetry
pkt/s, elbow tracking error 0.015 rad, watchdog dampen < 0.3 s
([stats plot](../../docs/img/ros2_wire_stats.png)).

## Roadmap

* `ros2_control` hardware interface + MoveIt 2 config for the bimanual chains
  (the `joint_states` + URDF path already works for RViz/planning-scene use);
* gripper action server;
* real-hardware validation when the Skate lands in Riga — wire numbers above
  are sim-endpoint numbers until then.

## Credits

Wire classes and protocol from [Rbotic/skate_teleop](https://github.com/Rbotic/skate_teleop)
(Apache-2.0, vendored with attribution in `skate_ros2/shared_classes_def.py`).
Everything else MIT, part of [SkateArm](https://github.com/Lavs-Daniels-Skots-231RMC173/skatearm).

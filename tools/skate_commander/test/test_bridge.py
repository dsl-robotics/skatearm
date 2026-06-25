"""RobotBridge vs the skate_ros2 MuJoCo sim endpoint over real UDP."""

import os
import socket
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "skate_ros2"))

from skate_commander.bridge import RobotBridge          # noqa: E402


def _free_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0)); p = s.getsockname()[1]; s.close()
    return p


def _spin(bridge, seconds, ui=True, hz=60):
    end = time.monotonic() + seconds
    snap = None
    while time.monotonic() < end:
        snap = bridge.tick(1.0 / hz, ui_attached=ui)
        time.sleep(1.0 / hz)
    return snap


def test_bridge_full_cycle():
    model = os.environ.get("SKATE_MJCF", "/tmp/skate_teleop/skt_v3/skt_v3_control.xml")
    if not Path(model).exists():
        print("SKIP: no control model"); return
    from skate_ros2.sim_endpoint import SkateSimEndpoint

    port = _free_port()
    ep = SkateSimEndpoint(model, port=port, bind="127.0.0.1", verbose=False)
    th = threading.Thread(target=ep.run, kwargs={"duration": 30.0}, daemon=True)
    th.start()

    br = RobotBridge(sim_host="127.0.0.1", sim_port=port, jog_rate=0.6)
    # starts estopped; arming happens from telemetry
    snap = _spin(br, 0.6)
    assert snap["connected"] and snap["armed"] and snap["estop"]
    assert not snap["live"]

    # jog while estopped must not move the target
    t0 = np.array(br.targ)
    br.jog_start(11, +1)
    _spin(br, 0.3)
    assert np.allclose(br.targ, t0)

    # resume -> jog moves the elbow, clamped to limits eventually
    assert br.resume()
    snap = _spin(br, 1.5)
    assert snap["live"]
    assert br.targ[11] > t0[11] + 0.4
    # the contact reflex must NOT false-trip during a normal smooth jog
    assert not snap["contact"]["tripped"], \
        "contact reflex false-tripped on a normal jog (threshold too low)"
    br.jog_stop(11)
    elbow_target = br.targ[11]
    snap = _spin(br, 1.0)
    assert abs(snap["q"][11] - elbow_target) < 0.08   # sim tracks the target

    # ui detached -> deadman drops, target freezes
    br.jog_start(11, +1)
    before = br.targ[11]
    snap = _spin(br, 0.4, ui=False)
    assert not snap["live"] and abs(br.targ[11] - before) < 1e-9

    # mode switch re-latches estop and disarms
    br.set_mode("real")
    assert br.estop and br.targ is None and br.mode == "real"
    br.set_mode("sim")
    snap = _spin(br, 0.6)
    assert snap["armed"] and snap["estop"]

    br.close(); ep.close()


def test_cart_step_and_mirror():
    """v0.5: cartesian step-jog auto-clears on arrival; mirror mode reflects
    jog / slider / IK input onto the other arm with the measured sign map."""
    model_xml = os.environ.get("SKATE_MJCF",
                               "/tmp/skate_teleop/skt_v3/skt_v3_control.xml")
    if not Path(model_xml).exists():
        print("SKIP: no control model"); return
    urdf = Path(model_xml).parent / "skt_v3.urdf"
    if not urdf.exists():
        print("SKIP: no URDF next to the control model"); return
    from skate_commander.kinematics import ArmKinematics
    from skate_commander.server import compute_mirror_map
    from skate_commander.urdf import joint_limits, parse_urdf
    from skate_ros2.sim_endpoint import SkateSimEndpoint

    model = parse_urdf(urdf)
    kin = {a: ArmKinematics(model, a) for a in ("left", "right")}
    port = _free_port()
    ep = SkateSimEndpoint(model_xml, port=port, bind="127.0.0.1",
                          verbose=False)
    th = threading.Thread(target=ep.run, kwargs={"duration": 40.0},
                          daemon=True)
    th.start()

    br = RobotBridge(sim_host="127.0.0.1", sim_port=port,
                     limits=joint_limits(model), kin=kin)
    br.mirror_signs, br.mirror_axis = compute_mirror_map(kin)
    _spin(br, 0.6)
    assert br.targ is not None
    br.resume()

    # -- cartesian step: TCP glides 5 cm up, target auto-clears -------------
    p0 = kin["right"].fk(br.targ)
    br.cart_step("right", [0.0, 0.0, 0.05])
    assert br.ik_targets["right"] is not None and br.ik_auto["right"]
    end = time.monotonic() + 4.0
    while br.ik_targets["right"] is not None and time.monotonic() < end:
        br.tick(1 / 60, ui_attached=True)
        time.sleep(1 / 60)
    assert br.ik_targets["right"] is None, "cart target must auto-clear"
    moved = kin["right"].fk(br.targ) - p0
    assert abs(moved[2] - 0.05) < 0.01, f"dz={moved[2]:.3f}, wanted 0.05"
    print(f"PASS cart_step: dz={moved[2]*1000:.1f} mm, auto-cleared")

    # -- mirror: slider input reflects with the measured sign ----------------
    br.home()                                  # glide to the symmetric default
    end = time.monotonic() + 5.0               # smooth home() eases in/out now
    while br.home_active and time.monotonic() < end:
        br.tick(1 / 60, ui_attached=True)
        time.sleep(1 / 60)
    assert not br.home_active, "home glide should settle to the default pose"
    br.mirror = True
    s2 = br.mirror_signs[2]                    # arm slot of joint 10/18
    br.set_joint(10, 0.5)
    assert abs(br.targ[18] - s2 * 0.5) < 1e-9, "mirrored slider value"
    # jog reflects too (and both stop together)
    q11, q19 = br.targ[11], br.targ[19]
    br.jog_start(11, +1)
    _spin(br, 0.5)
    br.jog_stop(11)
    d11, d19 = br.targ[11] - q11, br.targ[19] - q19
    s3 = br.mirror_signs[3]
    assert d11 > 0.05 and abs(d19 - s3 * d11) < 1e-9
    # wrists end up mirror-symmetric in the world
    pl, pr = kin["left"].fk(br.targ), kin["right"].fk(br.targ)
    mirr = pl.copy()
    mirr[br.mirror_axis] = -mirr[br.mirror_axis]
    assert np.linalg.norm(pr - mirr) < 5e-3, \
        f"wrists not mirror-symmetric: {np.linalg.norm(pr - mirr)*1000:.1f} mm"
    print(f"PASS mirror: jog/slider reflected (signs={br.mirror_signs[:4]}), "
          "wrists symmetric")

    # -- mirrored IK target + estop clears everything -------------------------
    br.set_ik_target("left", pl + [0.0, 0.0, 0.03])
    assert br.ik_targets["right"] is not None
    br.trigger_estop()
    assert br.ik_targets == {"left": None, "right": None}
    print("PASS mirrored IK target + estop clears both")
    br.close(); ep.close()


def test_home_glide_smooth():
    """home() eases to the default pose with a jerk-limited trapezoidal profile
    instead of snapping — pure logic, no sim model needed."""
    br = RobotBridge(sim_port=_free_port())
    br.estop = False                                   # pretend resumed
    n = br.home_pose.shape[0]
    start = np.zeros(n)
    br.targ = start.copy()
    br.home()
    assert br.home_active and br.home_vel == 0.0
    assert not np.allclose(br.targ, br.home_goal)       # has NOT snapped

    dt = 1.0 / 60
    vels, prev, ticks = [], br.targ.copy(), 0
    while br.home_active and ticks < 4000:
        br._home_tick(dt)
        vels.append(float(np.max(np.abs(br.targ - prev))) / dt)
        prev = br.targ.copy()
        ticks += 1
    assert not br.home_active
    assert np.allclose(br.targ, br.home_goal, atol=1e-2)   # converged
    assert ticks > 30, f"home snapped too fast ({ticks} ticks)"
    peak = max(vels)
    assert peak <= br.seq_rate + 1e-6, "exceeded the cruise rate"
    assert vels[0] < peak * 0.5, "no jerk-limited ease-in"
    assert vels[-1] < peak * 0.6, "no ease-out near the goal"
    print(f"PASS home glide: {ticks} ticks, peak {peak:.3f} rad/s, eased in+out")

    # any manual input / estop / dampening (all via seq_stop) cancels it
    br.targ = start.copy(); br.home(); br._home_tick(dt)
    assert br.home_active
    br.seq_stop()
    assert not br.home_active and br.home_vel == 0.0
    print("PASS home glide cancels on manual input")

    # a goal that can't be approached at all (joint pinned at its limit, target
    # beyond it) makes zero progress -> gives up promptly instead of hanging,
    # the same graceful exit as when the guard keeps reverting every step
    br.targ = start.copy(); br.targ[0] = br.hi[0]            # already at the limit
    br.home_goal = start.copy(); br.home_goal[0] = br.hi[0] + 5.0   # unreachable
    br.home_active = True; br.home_vel = 0.0
    br._home_prev = None; br._home_stall = 0.0
    ticks = 0
    while br.home_active and ticks < 4000:
        br._home_tick(dt); ticks += 1
    assert not br.home_active, "a blocked/unreachable home must give up, not hang"
    assert ticks < 120, f"gave up too slowly ({ticks} ticks)"   # ~0.8 s stall window
    print(f"PASS home glide gives up when blocked ({ticks} ticks)")
    br.close()


def test_contact_reflex():
    """An arm-joint torque spike trips a latched soft-stop; legs and grippers
    are ignored; reset re-baselines. Pure logic, no sim model needed."""
    from skate_ros2 import names
    br = RobotBridge(sim_port=_free_port())
    br.contact_tau = 5.0
    n = names.N_JOINTS

    stalled = np.zeros(n)                              # joint not moving
    base = np.full(n, 1.0)                             # steady holding torque
    assert br._contact_update(base, stalled) is False  # first sample = baseline
    assert br._contact_update(base.copy(), stalled) is False   # steady -> no trip

    leg = base.copy(); leg[2] += 50.0                  # leg spike (firmware's)
    assert br._contact_update(leg, stalled) is False
    grip = base.copy(); grip[15] += 50.0               # gripper spike (a grasp)
    assert br._contact_update(grip, stalled) is False

    # a fast COMMANDED move: big torque but the joint is slewing -> NOT a contact
    br._tau_ref = base.copy(); br._contact_run = 0
    fast = base.copy(); fast[19] += 50.0
    moving = np.zeros(n); moving[19] = 2.0             # 2 rad/s
    assert br._contact_update(fast, moving) is False
    assert br._contact_update(fast, moving) is False   # still slewing -> never trips

    # a BLOCKED joint: same spike but stalled -> trips once it persists (hold=2)
    br._tau_ref = base.copy(); br._contact_run = 0
    blocked = base.copy(); blocked[19] += 50.0
    assert br._contact_update(blocked, stalled) is False  # hold tick 1 of 2
    assert br._contact_update(blocked, stalled) is True   # persisted -> trip
    assert br.contact_joint == 19
    print("PASS contact detect: blocked arm joint trips; fast move/legs/grippers don't")

    # trip latches a soft-stop and stops any motion in progress
    br.targ = np.array(names.DEFAULT_POSE, dtype=float)
    br.estop = False
    br.jog_start(19, +1)
    br._trip_contact()
    assert br.contact_tripped and not br.jog_dir.any() and not br.carry
    snap = br.snapshot(ui_attached=True)
    assert snap["contact"]["tripped"] and snap["live"] is False

    br.clear_contact()                                 # operator reset
    assert not br.contact_tripped and br._tau_ref is None
    assert br.snapshot()["contact"]["tripped"] is False
    print("PASS contact latch: trip dampens, reset clears + re-baselines")
    br.close()


def test_seq_routes_around_collision():
    """A waypoint move whose straight leg is guard-blocked makes the sequencer
    plan a collision-free DETOUR to the waypoint (not a straight glide); a
    clear leg stays a direct move. Pure logic, no sim. (End-to-end glide-
    following through a real collision is covered by the home e2e + live.)"""
    from skate_commander import planner
    br = RobotBridge(sim_port=_free_port())
    br.estop = False
    n = br.home_pose.shape[0]
    start = np.zeros(n)
    br.targ = start.copy()
    # forbidden when joint 11 is mid-range AND joint 9 is near zero; swinging
    # joint 9 out opens a gap — mirrors the elbow/abduction self-collision
    br.guard = lambda q: (0.3 < q[11] < 1.1) and (abs(q[9]) < 0.7)
    cfree = lambda q: not br.guard(q)

    # 1) a BLOCKED leg -> the sequencer plans a multi-node detour to the wp
    wp = np.zeros(n); wp[11] = 1.5
    br.waypoints = [wp]; br.wp_names = ["WP1"]
    assert not planner._edge_clear(start, wp, cfree, 0.05), "straight leg must be blocked"
    br.wp_goto(0)
    br._seq_tick(1 / 60)                         # first tick plans the leg
    assert br.seq_route is not None and len(br.seq_route) > 1, \
        "a blocked leg must become a multi-node detour, not a straight glide"
    for q in br.seq_route:
        assert cfree(q), "every detour node must be collision-free"
    assert np.allclose(br.seq_route[-1], wp), "the detour ends at the waypoint"
    print(f"PASS sequencer plans a {len(br.seq_route)}-node detour for a blocked leg")

    # 2) a CLEAR leg -> a direct (single-node) move, no detour
    br.seq_stop()
    br.targ = start.copy()
    wp2 = np.zeros(n); wp2[8] = 0.5             # joint 8 is unobstructed
    br.waypoints = [wp2]; br.wp_names = ["WP2"]
    br.wp_goto(0)
    br._seq_tick(1 / 60)
    assert br.seq_route is not None and len(br.seq_route) == 1, \
        "a clear leg should be a direct move"
    print("PASS sequencer uses a direct move for a clear leg")
    br.close()


if __name__ == "__main__":
    test_home_glide_smooth()
    test_contact_reflex()
    test_seq_routes_around_collision()
    test_bridge_full_cycle(); print("PASS test_bridge_full_cycle")
    test_cart_step_and_mirror()


def test_set_speed_clamps_and_scales():
    """F4 global speed override: clamps to 0.1-1.0, ignores bad input,
    and feeds the jog + glide cruise scaling."""
    br = RobotBridge(sim_host="127.0.0.1", sim_port=_free_port(), jog_rate=0.6)
    assert br.speed_scale == 1.0                 # full speed by default
    assert br.set_speed(0.5) == 0.5 and br.speed_scale == 0.5
    assert br.set_speed(5.0) == 1.0              # clamp high
    assert br.set_speed(0.0) == 0.1              # clamp low
    br.set_speed("bad")                          # non-numeric ignored, no raise
    assert br.speed_scale == 0.1
    # snapshot exposes the scale so the UI can reflect it
    assert br.snapshot()["speed_scale"] == 0.1


def test_pause_and_step():
    """I1 sim-transport: pause freezes autonomous motion, step queues ticks,
    and unpause clears the pending step counter."""
    br = RobotBridge(sim_host="127.0.0.1", sim_port=_free_port(), jog_rate=0.6)
    assert br.paused is False
    assert br.set_paused(True) is True and br.paused is True
    assert br._step_ticks == 0
    assert br.step(3) == 3 and br._step_ticks == 3
    assert br.step() == 4                          # default n=1
    assert br.set_paused(False) is False and br._step_ticks == 0   # unpause clears steps
    assert br.snapshot()["paused"] is False

"""Skate Commander server — FastAPI + WebSocket bridge to a Skate (sim/real).

    python3 -m skate_commander.server --model-dir /path/to/skate_teleop/skt_v3 \
        [--real-host r.local] [--spawn-sim /path/to/skt_v3_control.xml] \
        [--port 8088]

Serves the single-page UI, the URDF model tree, the STL meshes (from YOUR
local Rbotic/skate_teleop clone — meshes are not redistributed), and a
WebSocket carrying telemetry down (~20 Hz) and commands up.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import (FileResponse, JSONResponse, PlainTextResponse,
                               StreamingResponse)
from fastapi.staticfiles import StaticFiles

from .bridge import RobotBridge
from .kinematics import ArmKinematics, reach_map
from .program import PoseRecorder, ProgramRunner
from .urdf import joint_limits, parse_urdf
from . import camera, ibvs, nl, vision

from skate_ros2 import names  # noqa: E402  (path set up by .bridge)

STATIC_DIR = Path(__file__).resolve().parents[1] / "static"
SEQ_DIR = Path(__file__).resolve().parents[1] / "sequences"
PROG_DIR = Path(__file__).resolve().parents[1] / "programs"
TOOLS_FILE = Path(__file__).resolve().parents[1] / "tcp_tools.json"


def _seq_name_ok(name):
    return (isinstance(name, str) and 0 < len(name) <= 40
            and all(c.isalnum() or c in "-_" for c in name))

TX_HZ = 60.0          # bridge tick / command rate
WS_HZ = 20.0          # UI telemetry rate

# manual UI inputs interrupt a running program (same rule as the sequencer)
_MANUAL = {"jog_start", "jog_step", "set_joint", "ik_target", "cart_step",
           "wp_goto", "wp_play", "home", "carry_grab", "carry_step",
           "carry_release"}


def compute_mirror_map(kin):
    """Derive the left<->right mirror convention numerically from FK.

    Returns ``(signs[8], axis)``: flipping world coordinate ``axis`` maps
    the left wrist onto the right at the neutral pose, and ``signs[k]``
    makes arm-slot ``k`` of one arm reproduce the mirrored motion of the
    other. Measured from the model, not guessed from the URDF convention.
    """
    q0 = np.zeros(names.N_JOINTS)
    pL, pR = kin["left"].fk(q0), kin["right"].fk(q0)
    axis = int(np.argmax(np.abs(pL - pR)))

    def mirr(v):
        v = v.copy()
        v[axis] = -v[axis]
        return v

    signs = np.ones(8)
    for k in range(7):                      # slot 7 = gripper, sign +1
        qa = q0.copy()
        qa[8 + k] = 0.3
        dl = kin["left"].fk(qa) - pL
        qb = q0.copy()
        qb[16 + k] = 0.3
        dr = kin["right"].fk(qb) - pR
        signs[k] = (1.0 if np.linalg.norm(dr - mirr(dl))
                    <= np.linalg.norm(dr + mirr(dl)) else -1.0)
    return signs, axis


def _load_tools():
    tools = {"flange": [0.0, 0.0, 0.0]}
    if TOOLS_FILE.exists():
        try:
            data = json.loads(TOOLS_FILE.read_text())
            for k, v in data.items():
                if (_seq_name_ok(k) and isinstance(v, list) and len(v) == 3
                        and all(abs(float(x)) <= 500.0 for x in v)):
                    tools[k] = [float(x) for x in v]
        except Exception as e:
            print(f"[commander] WARNING: bad {TOOLS_FILE.name}: {e}")
    return tools


def make_collision_guard(collision_xml):
    """Self-collision predicate over the SkateArm box-collision model.

    The physics collision model excludes pairs that touch at the neutral pose
    (hanging hands sit right next to the hips!) so the robot doesn't jam in
    sim. A *predictive* guard must still see those pairs — so we build a
    guard-specific variant: distant excludes (kinematically > 3 hops apart,
    e.g. hand<->thigh) are re-enabled, and instead of excluding them we
    tolerate exactly the contact depth they have at the neutral pose.
    q26 -> True if the pose penetrates >2 mm (or >4 mm past baseline).
    """
    import xml.etree.ElementTree as ET

    import mujoco
    collision_xml = Path(collision_xml)
    m0 = mujoco.MjModel.from_xml_path(str(collision_xml))

    def chain(b):
        out = []
        while b != 0:
            out.append(b)
            b = m0.body_parentid[b]
        out.append(0)
        return out

    def hops(a, b):                       # kinematic distance between bodies
        ca, cb = chain(a), chain(b)
        return min(i + cb.index(x) for i, x in enumerate(ca) if x in cb)

    tree = ET.parse(collision_xml)
    cel = tree.getroot().find("contact")
    removed = 0
    if cel is not None:
        for ex in list(cel.findall("exclude")):
            try:
                b1 = m0.body(ex.get("body1")).id
                b2 = m0.body(ex.get("body2")).id
            except Exception:
                continue
            if hops(b1, b2) > 3:          # distant pair: the guard must see it
                cel.remove(ex)
                removed += 1
    guard_xml = collision_xml.parent / ".skate_guard.xml"
    tree.write(str(guard_xml))
    gm = mujoco.MjModel.from_xml_path(str(guard_xml))
    gd = mujoco.MjData(gm)

    # contacts already present at the neutral pose are tolerated up to their
    # baseline depth — anything deeper (or any NEW pair) is a violation
    gd.qpos[:] = 0
    mujoco.mj_forward(gm, gd)
    base = {}
    for i in range(gd.ncon):
        c = gd.contact[i]
        key = (min(c.geom1, c.geom2), max(c.geom1, c.geom2))
        base[key] = min(base.get(key, 0.0), float(c.dist))
    print(f"[commander] guard model: {removed} distant pairs re-enabled, "
          f"{len(base)} neutral-pose contacts baselined")

    def guard(q):
        gd.qpos[:26] = q
        mujoco.mj_forward(gm, gd)
        for i in range(gd.ncon):
            c = gd.contact[i]
            key = (min(c.geom1, c.geom2), max(c.geom1, c.geom2))
            thr = base[key] - 0.004 if key in base else -0.002
            if float(c.dist) < thr:
                return True
        return False
    return guard


def build_app(model_dir, real_host="r.local", sim_port=2000,
              collision_model=None):
    model_dir = Path(model_dir)
    urdf_path = model_dir / "skt_v3.urdf"
    mesh_dir = model_dir / "skt_v3_meshes" / "scaled_stl_files"
    if not urdf_path.exists():
        raise FileNotFoundError(f"{urdf_path} not found — point --model-dir "
                                "at the skt_v3 folder of your skate_teleop clone")
    model = parse_urdf(urdf_path)
    # resolve every mesh against both known layouts of the official clone
    mesh_dirs = [model_dir / "skt_v3_meshes" / "scaled_stl_files",
                 model_dir / "skt_v3_meshes"]
    mesh_paths = {}
    for name in model["mesh_files"]:
        for d in mesh_dirs:
            if (d / name).exists():
                mesh_paths[name] = d / name
                break
    missing = set(model["mesh_files"]) - set(mesh_paths)
    if missing:
        print(f"[commander] WARNING: {len(missing)} URDF meshes not found "
              f"under {model_dir} — the viewer will fall back to a stick "
              f"figure for those links: {sorted(missing)[:3]}...")

    kin = {arm: ArmKinematics(model, arm) for arm in ("left", "right")}
    bridge = RobotBridge(real_host=real_host, sim_port=sim_port,
                         limits=joint_limits(model), kin=kin)
    bridge.mirror_signs, bridge.mirror_axis = compute_mirror_map(kin)
    print(f"[commander] mirror map: axis={'xyz'[bridge.mirror_axis]} "
          f"signs={[int(s) for s in bridge.mirror_signs]}")
    if collision_model:
        bridge.guard = make_collision_guard(collision_model)
        print("[commander] collision guard ON — self-colliding targets are "
              "rejected before they reach the robot")
    runner = ProgramRunner(bridge)
    bridge.recorder = PoseRecorder()      # teach-in: observed in tick()
    tools = _load_tools()

    def save_tools():
        TOOLS_FILE.write_text(json.dumps(tools, indent=1))

    app = FastAPI(title="Skate Commander")
    app.state.bridge = bridge
    app.state.runner = runner
    app.state.tools = tools
    app.state.clients = 0
    app.state.reachmap = {}                 # cached dexterity clouds per arm

    def _cam_qpos():
        try:
            q = bridge.link.state.dof_pos()
        except Exception:
            q = None
        if q is None and bridge.targ is not None:
            q = list(bridge.targ)
        return q

    try:
        scene_path = camera.build_scene_xml(model_dir)
        app.state.camera = camera.CameraStreamer(scene_path, _cam_qpos)
    except Exception as exc:                       # camera is optional
        print(f"[commander] camera disabled: {exc}")
        app.state.camera = None

    @app.get("/")
    async def index():
        return FileResponse(STATIC_DIR / "index.html")

    @app.get("/api/model")
    async def api_model():
        return JSONResponse(model)

    @app.get("/api/reachmap")
    def api_reachmap(arm: str = "right", n: int = 3000):
        """Manipulability heat-map: a dexterity point cloud over the arm's
        reachable workspace — [[x, y, z, manip], ...], manip in [0, 1] (0 =
        singular, 1 = isotropic). Computed once per arm and cached. Sync `def`
        so the ~1.5 s sample runs in FastAPI's threadpool, not the event loop."""
        n = max(200, min(int(n), 6000))
        key = f"{arm}:{n}"
        if arm in kin and key not in app.state.reachmap:
            app.state.reachmap[key] = reach_map(
                kin[arm], np.array(names.DEFAULT_POSE, dtype=float), n=n)
        return JSONResponse({"arm": arm, "points": app.state.reachmap.get(key, [])})

    @app.get("/api/pointcloud")
    def api_pointcloud(stride: int = 5):
        """Work-camera depth back-projected to a coloured world point cloud
        ([[x, y, z, r, g, b], ...]) — what the camera sees, in 3D. Serviced on
        the camera's GL render thread (sync def -> FastAPI threadpool)."""
        cam = app.state.camera
        if cam is None:
            return JSONResponse({"error": "no camera"})
        res = cam.cloud(stride=max(2, min(int(stride), 12)))
        return JSONResponse({"points": res} if isinstance(res, list) else res)

    @app.get("/api/sequences")
    async def api_sequences():
        if not SEQ_DIR.exists():
            return JSONResponse([])
        return JSONResponse(sorted(p.stem for p in SEQ_DIR.glob("*.json")))

    @app.get("/api/tools")
    async def api_tools():
        return JSONResponse(tools)

    @app.get("/api/programs")
    async def api_programs():
        if not PROG_DIR.exists():
            return JSONResponse([])
        return JSONResponse(sorted(p.stem for p in PROG_DIR.glob("*.py")))

    @app.get("/api/programs/{name}")
    async def api_program(name: str):
        path = PROG_DIR / f"{name}.py"
        if not _seq_name_ok(name) or not path.exists():
            return JSONResponse({"error": "unknown program"}, status_code=404)
        return PlainTextResponse(path.read_text(encoding="utf-8"))

    @app.get("/api/recording")
    async def api_recording():
        return PlainTextResponse(bridge.recorder.result)

    @app.post("/api/nl")
    async def api_nl(req: Request):
        """Natural language -> rbt program. Returns code for the editor only;
        it never moves the robot (the editor still runs through the safe bridge)."""
        try:
            data = await req.json()
        except Exception:
            data = {}
        return JSONResponse(nl.generate((data or {}).get("text", "")))

    @app.get("/api/cameras")
    async def api_cameras():
        cam = app.state.camera
        if cam is None:
            return JSONResponse({"cameras": [], "current": None})
        return JSONResponse({"cameras": cam.cams, "current": cam.cam})

    @app.get("/camstream")
    async def camstream(cam: str = None):
        streamer = app.state.camera
        if streamer is None:
            return JSONResponse({"error": "camera unavailable"}, status_code=503)
        if cam:
            streamer.set_cam(cam)

        async def gen():
            boundary = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
            while True:
                yield boundary + streamer.jpeg() + b"\r\n"
                await asyncio.sleep(1 / 15)

        return StreamingResponse(
            gen(), media_type="multipart/x-mixed-replace; boundary=frame")

    @app.get("/api/detect")
    async def api_detect():
        cam = app.state.camera
        if cam is None:
            return JSONResponse({"found": False, "error": "camera unavailable"})
        return JSONResponse(cam.detect())

    @app.post("/api/pick")
    async def api_pick():
        """Detect the workspace target and run a pick program (right arm) through
        the same guarded runner. Moves nothing unless armed + resumed."""
        cam = app.state.camera
        if cam is None:
            return JSONResponse({"found": False, "error": "camera unavailable"})
        det = cam.detect()
        if not det.get("found"):
            return JSONResponse({"found": False,
                                 "error": det.get("error", "no target seen")})
        x, y, z = det["world_mm"]
        code = (
            f"# pick: magenta target detected at ({x:.0f}, {y:.0f}, {z:.0f}) mm\n"
            f"rbt.moveto('right', {x:.0f}, {y:.0f}, {z + 90:.0f})\n"
            f"rbt.moveto('right', {x:.0f}, {y:.0f}, {z + 12:.0f})\n"
            f"rbt.gripper('right', 0)\n"
            f"rbt.moveto('right', {x:.0f}, {y:.0f}, {z + 130:.0f})\n"
        )
        ok = runner.run(code)
        return JSONResponse({"found": True, "ran": bool(ok),
                             "world_mm": det["world_mm"],
                             "pixel": det.get("pixel"), "code": code})

    @app.post("/api/servo_pick")
    async def api_servo_pick():
        """Closed-loop image-based visual servo pick (right arm). Detects the
        static target once, then drives the gripper's image feature onto it
        while descending — robust to camera-calibration error, unlike the
        one-shot /api/pick. Moves nothing unless armed + resumed."""
        cam = app.state.camera
        if cam is None:
            return JSONResponse({"found": False, "error": "camera unavailable"})
        if runner.running:
            return JSONResponse({"found": False, "error": "a program is running"})
        kr = bridge.kin.get("right")
        if bridge.targ is None or bridge.estop or kr is None:
            return JSONResponse({"found": False,
                                 "error": "right arm not armed / resumed"})
        obs = cam.detect(ee_world=list(kr.fk(bridge.targ)))
        if not obs.get("found") or "cam" not in obs:
            return JSONResponse({"found": False,
                                 "error": obs.get("error", "no target seen")})
        s_cube = np.asarray(obs["pixel"], float)
        C = obs["cam"]
        f0, cx0, cy0 = vision.intrinsics(C["fovy"], C["W"], C["H"])
        pos0 = np.asarray(C["pos"], float)
        mat0 = np.asarray(C["mat"], float).reshape(3, 3)
        servo = ibvs.ImageServo(pos0, mat0, C["fovy"], C["W"], C["H"],
                                gain=0.7, max_step=0.03)
        grasp_z = camera.GRASP_Z
        z = grasp_z + 0.08
        err_px = 999.0
        for _ in range(20):                     # image servo, descending
            ee = kr.fk(bridge.targ)
            ee_pix = np.asarray(
                vision.project(ee, pos0, mat0, f0, cx0, cy0)[:2], float)
            dxy, err_px = servo.step(s_cube, ee_pix, ee)
            z = max(grasp_z + 0.03, z - 0.008)
            bridge.set_ik_target(
                "right", [ee[0] + dxy[0], ee[1] + dxy[1], z], auto=True)
            for _ in range(18):                 # let the tick loop track it
                await asyncio.sleep(1 / 60)
            if err_px < 3.0 and z <= grasp_z + 0.035:
                break
        ee = kr.fk(bridge.targ)
        x, y, gz = ee[0] * 1000, ee[1] * 1000, grasp_z * 1000
        code = (
            f"# servo-pick: IBVS-aligned over target (image err {err_px:.1f}px)\n"
            f"rbt.moveto('right', {x:.0f}, {y:.0f}, {gz + 12:.0f})\n"
            f"rbt.gripper('right', 0)\n"
            f"rbt.moveto('right', {x:.0f}, {y:.0f}, {gz + 130:.0f})\n"
        )
        ok = runner.run(code)
        return JSONResponse({"found": True, "ran": bool(ok),
                             "image_err_px": round(float(err_px), 1),
                             "world_mm": [round(x, 1), round(y, 1)]})

    @app.get("/meshes/{name}")
    async def mesh(name: str):
        path = mesh_paths.get(name)           # whitelist, no traversal
        if path is None:
            return JSONResponse({"error": "unknown mesh"}, status_code=404)
        return FileResponse(path, media_type="application/octet-stream")

    @app.websocket("/ws")
    async def ws(sock: WebSocket):
        await sock.accept()
        app.state.clients += 1
        try:
            while True:
                try:
                    raw = await asyncio.wait_for(sock.receive_text(),
                                                 timeout=1.0 / WS_HZ)
                except asyncio.TimeoutError:
                    raw = None
                if raw is not None:                # one bad command must not
                    try:                           # drop the whole connection
                        handle_command(bridge, json.loads(raw), runner=runner,
                                       tools=tools, save_tools=save_tools)
                    except Exception as e:
                        print(f"[commander] ignored bad command: {e}")
                snap = bridge.snapshot(ui_attached=app.state.clients > 0)
                snap["prog"] = runner.snapshot()
                snap["prog"]["rec"] = bridge.recorder.snapshot()
                await sock.send_text(json.dumps(snap))
        except WebSocketDisconnect:
            pass
        finally:
            app.state.clients -= 1
            if app.state.clients <= 0:
                bridge.jog_stop()             # nobody at the controls

    @app.on_event("startup")
    async def start_tick():
        async def loop():
            dt = 1.0 / TX_HZ
            while True:
                bridge.tick(dt, ui_attached=app.state.clients > 0)
                await asyncio.sleep(dt)
        app.state.tick_task = asyncio.create_task(loop())

    @app.on_event("shutdown")
    async def stop_tick():
        app.state.tick_task.cancel()
        bridge.close()
        if getattr(app.state, "camera", None):
            app.state.camera.close()

    app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")
    return app


def handle_command(bridge: RobotBridge, cmd: dict, runner=None, tools=None,
                   save_tools=None):
    t = cmd.get("type")
    if runner is not None and runner.running and t in _MANUAL:
        runner.stop("manual input")        # the human at the panel wins
    if t == "cart_step":
        delta = cmd.get("delta", ())
        if isinstance(delta, (list, tuple)) and len(delta) == 3:
            d = [max(-0.2, min(0.2, float(x))) for x in delta]
            bridge.cart_step(cmd.get("arm"), d)
    elif t == "carry_grab":
        bridge.carry_grab()
    elif t == "carry_step":
        delta = cmd.get("delta", ())
        if isinstance(delta, (list, tuple)) and len(delta) == 3:
            d = [max(-0.2, min(0.2, float(x))) for x in delta]
            bridge.carry_step(d)
    elif t == "carry_release":
        bridge.carry_release()
    elif t == "mirror":
        bridge.mirror = bool(cmd.get("on"))
        if not bridge.mirror:
            bridge.jog_stop()              # don't leave a ghost jog running
    elif t == "tool_set":
        if tools and cmd.get("name") in tools:
            off_m = [v / 1000.0 for v in tools[cmd["name"]]]
            bridge.set_tool(cmd.get("arm"), cmd["name"], off_m)
    elif t == "tool_def":
        name, xyz = cmd.get("name", ""), cmd.get("xyz_mm", ())
        if (tools is not None and _seq_name_ok(name) and name != "flange"
                and isinstance(xyz, (list, tuple)) and len(xyz) == 3):
            try:
                vals = [float(x) for x in xyz]
            except (TypeError, ValueError):
                return
            if all(abs(v) <= 500.0 for v in vals):
                tools[name] = vals
                if save_tools:
                    save_tools()
                for arm, nm in list(bridge.tool_names.items()):
                    if nm == name:         # live-update an attached tool
                        bridge.set_tool(arm, name, [v / 1000 for v in vals])
    elif t == "tool_del":
        name = cmd.get("name", "")
        if tools and name in tools and name != "flange":
            del tools[name]
            if save_tools:
                save_tools()
            for arm, nm in list(bridge.tool_names.items()):
                if nm == name:
                    bridge.set_tool(arm, "flange", [0.0, 0.0, 0.0])
    elif t == "prog_run":
        if runner:
            runner.run(cmd.get("code"))
    elif t == "prog_step":
        if runner:
            runner.step(cmd.get("code"))
    elif t == "prog_stop":
        if runner:
            runner.stop()
    elif t == "prog_save":
        name = cmd.get("name", "")
        if _seq_name_ok(name) and isinstance(cmd.get("code"), str):
            PROG_DIR.mkdir(exist_ok=True)
            (PROG_DIR / f"{name}.py").write_text(cmd["code"],
                                                 encoding="utf-8")
    elif t == "rec_start":
        if bridge.recorder is not None:
            bridge.recorder.start(bridge.targ)
    elif t == "rec_stop":
        if bridge.recorder is not None:
            bridge.recorder.stop()
    elif t == "jog_start":
        bridge.jog_start(int(cmd["idx"]), int(cmd["dir"]))
    elif t == "jog_stop":
        bridge.jog_stop(cmd.get("idx"))
    elif t == "jog_step":
        bridge.jog_step(int(cmd["idx"]), float(cmd["delta"]))
    elif t == "set_joint":
        bridge.set_joint(int(cmd["idx"]), float(cmd["value"]))
    elif t == "ik_target":
        bridge.set_ik_target(cmd.get("arm"), cmd.get("pos", ()))
    elif t == "ik_clear":
        bridge.clear_ik_target(cmd.get("arm"))
    elif t == "wp_add":
        bridge.wp_add()
    elif t == "wp_delete":
        bridge.wp_delete(int(cmd.get("idx", -1)))
    elif t == "wp_clear":
        bridge.wp_clear()
    elif t == "wp_goto":
        bridge.wp_goto(int(cmd.get("idx", -1)))
    elif t == "wp_play":
        bridge.wp_play(loop=bool(cmd.get("loop", False)))
    elif t == "wp_stop":
        bridge.seq_stop()
    elif t == "wp_save":
        name = cmd.get("name", "")
        if _seq_name_ok(name) and bridge.waypoints:
            SEQ_DIR.mkdir(exist_ok=True)
            (SEQ_DIR / f"{name}.json").write_text(json.dumps(
                {"names": bridge.wp_names,
                 "q": [w.tolist() for w in bridge.waypoints]}))
    elif t == "wp_load":
        name = cmd.get("name", "")
        path = SEQ_DIR / f"{name}.json"
        if _seq_name_ok(name) and path.exists():
            import numpy as np
            data = json.loads(path.read_text())
            bridge.wp_clear()
            bridge.waypoints = [np.asarray(q, dtype=float)
                                for q in data["q"]]
            bridge.wp_names = list(data["names"])
    elif t == "estop":
        bridge.trigger_estop()
    elif t == "resume":
        bridge.resume()
    elif t == "home":
        bridge.home()
    elif t == "reset_contact":
        bridge.clear_contact()
    elif t == "set_mode":
        bridge.set_mode(cmd["mode"])


SIM_DIR = Path(__file__).resolve().parents[3] / "sim"
SKT_TELEOP_URL = "https://github.com/Rbotic/skate_teleop.git"


def _find_model_dir():
    """Locate the skt_v3 model folder: $SKT_DIR, then common clone locations
    near the current dir / repo / home. Returns a Path or None."""
    import os
    cands = []
    if os.environ.get("SKT_DIR"):
        cands.append(Path(os.environ["SKT_DIR"]))
    here = Path.cwd()
    root = Path(__file__).resolve().parents[3]
    for base in (here, here.parent, here.parent.parent, root, root.parent,
                 Path.home()):
        cands += [base / "skt_v3", base / "skate_teleop" / "skt_v3"]
    for c in cands:
        if (c / "skt_v3.urdf").exists() or (c / "skt_v3_converted.xml").exists():
            return c
    return None


def _ensure_collision_model(model_dir):
    """Make sure skt_v3_collision.xml exists — generating the control then the
    collision model from the official converted model on first run. The
    collision model is used both as the sim-endpoint model and the guard model."""
    md = Path(model_dir)
    collision = md / "skt_v3_collision.xml"
    if collision.exists():
        return collision
    if not (md / "skt_v3_control.xml").exists():
        if not (md / "skt_v3_converted.xml").exists():
            raise FileNotFoundError(
                f"{md} has no skt_v3_converted.xml — point --model-dir at the "
                "skt_v3 folder of your Rbotic/skate_teleop clone")
        print("[commander] one-time setup: building control model…")
        subprocess.run([sys.executable, str(SIM_DIR / "make_control_model.py"),
                        str(md)], check=True)
    print("[commander] one-time setup: building collision model…")
    subprocess.run([sys.executable, str(SIM_DIR / "make_collision_model.py"),
                    str(md)], check=True)
    if not collision.exists():
        raise RuntimeError(f"collision model generation did not produce {collision}")
    return collision


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Skate Commander — web cockpit for the Skate twin/robot")
    ap.add_argument("--model-dir", default=None,
                    help="skt_v3 folder of your Rbotic/skate_teleop clone "
                         "(auto-detected from $SKT_DIR or a nearby clone if omitted)")
    ap.add_argument("--real", action="store_true",
                    help="drive a real Skate instead of the local sim endpoint")
    ap.add_argument("--real-host", default="r.local")
    ap.add_argument("--port", type=int, default=8088)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--no-browser", action="store_true",
                    help="don't open the cockpit in a browser on startup")
    ap.add_argument("--spawn-sim", metavar="MJCF_XML", default=None,
                    help="advanced: explicit sim-endpoint model "
                         "(default: the auto-generated skt_v3_collision.xml)")
    ap.add_argument("--collision-model", metavar="COLLISION_XML", default=None,
                    help="advanced: explicit collision-guard model "
                         "(default: the auto-generated skt_v3_collision.xml)")
    args = ap.parse_args(argv)

    model_dir = Path(args.model_dir) if args.model_dir else _find_model_dir()
    if not model_dir or not Path(model_dir).exists():
        sys.exit("[commander] couldn't find the skt_v3 model.\n"
                 f"  clone it once:  git clone {SKT_TELEOP_URL}\n"
                 "  then re-run from that folder, or pass --model-dir <…>/skt_v3")

    sim_mode = not args.real
    collision_xml = None
    if (sim_mode and not args.spawn_sim) or args.collision_model is None:
        try:
            collision_xml = _ensure_collision_model(model_dir)
        except Exception as e:
            if sim_mode and not args.spawn_sim:
                sys.exit(f"[commander] setup failed (the sim needs the model): {e}")
            print(f"[commander] WARNING: collision guard unavailable: {e}")

    spawn = args.spawn_sim or (str(collision_xml) if sim_mode else None)
    guard = args.collision_model or (str(collision_xml) if collision_xml else None)

    sim_proc = None
    if spawn:
        sim_proc = subprocess.Popen(
            [sys.executable, "-m", "skate_ros2.sim_endpoint",
             "--model", spawn, "--quiet"],
            cwd=str(Path(__file__).resolve().parents[2] / "skate_ros2"))
        print(f"[commander] sim endpoint spawned (pid {sim_proc.pid})")

    import uvicorn
    app = build_app(str(model_dir), real_host=args.real_host,
                    collision_model=guard)
    url = f"http://{args.host}:{args.port}"
    mode = "REAL" if args.real else "SIM"
    print(f"[commander] {url}  ({mode} — starts dampened, press RESUME in the UI)")
    if not args.no_browser:
        import threading
        import webbrowser
        threading.Timer(1.3, lambda: webbrowser.open(url)).start()
    try:
        uvicorn.run(app, host=args.host, port=args.port, log_level="warning")
    finally:
        if sim_proc:
            sim_proc.terminate()


if __name__ == "__main__":
    main()

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

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

from .bridge import RobotBridge
from .urdf import joint_limits, parse_urdf

STATIC_DIR = Path(__file__).resolve().parents[1] / "static"

TX_HZ = 60.0          # bridge tick / command rate
WS_HZ = 20.0          # UI telemetry rate


def build_app(model_dir, real_host="r.local", sim_port=2000):
    model_dir = Path(model_dir)
    urdf_path = model_dir / "skt_v3.urdf"
    mesh_dir = model_dir / "skt_v3_meshes" / "scaled_stl_files"
    if not urdf_path.exists():
        raise FileNotFoundError(f"{urdf_path} not found — point --model-dir "
                                "at the skt_v3 folder of your skate_teleop clone")
    model = parse_urdf(urdf_path)
    allowed_meshes = set(model["mesh_files"])

    bridge = RobotBridge(real_host=real_host, sim_port=sim_port,
                         limits=joint_limits(model))
    app = FastAPI(title="Skate Commander")
    app.state.bridge = bridge
    app.state.clients = 0

    @app.get("/")
    async def index():
        return FileResponse(STATIC_DIR / "index.html")

    @app.get("/api/model")
    async def api_model():
        return JSONResponse(model)

    @app.get("/meshes/{name}")
    async def mesh(name: str):
        if name not in allowed_meshes:        # whitelist, no traversal
            return JSONResponse({"error": "unknown mesh"}, status_code=404)
        return FileResponse(mesh_dir / name,
                            media_type="application/octet-stream")

    @app.websocket("/ws")
    async def ws(sock: WebSocket):
        await sock.accept()
        app.state.clients += 1
        try:
            while True:
                try:
                    raw = await asyncio.wait_for(sock.receive_text(),
                                                 timeout=1.0 / WS_HZ)
                    handle_command(bridge, json.loads(raw))
                except asyncio.TimeoutError:
                    pass
                await sock.send_text(json.dumps(bridge.snapshot(
                    ui_attached=app.state.clients > 0)))
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

    app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")
    return app


def handle_command(bridge: RobotBridge, cmd: dict):
    t = cmd.get("type")
    if t == "jog_start":
        bridge.jog_start(int(cmd["idx"]), int(cmd["dir"]))
    elif t == "jog_stop":
        bridge.jog_stop(cmd.get("idx"))
    elif t == "jog_step":
        bridge.jog_step(int(cmd["idx"]), float(cmd["delta"]))
    elif t == "set_joint":
        bridge.set_joint(int(cmd["idx"]), float(cmd["value"]))
    elif t == "estop":
        bridge.trigger_estop()
    elif t == "resume":
        bridge.resume()
    elif t == "home":
        bridge.home()
    elif t == "set_mode":
        bridge.set_mode(cmd["mode"])


def main(argv=None):
    ap = argparse.ArgumentParser(description="Skate Commander server")
    ap.add_argument("--model-dir", required=True,
                    help="skt_v3 folder of your Rbotic/skate_teleop clone")
    ap.add_argument("--real-host", default="r.local")
    ap.add_argument("--port", type=int, default=8088)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--spawn-sim", metavar="CONTROL_XML", default=None,
                    help="also launch the skate_ros2 sim endpoint on UDP :2000")
    args = ap.parse_args(argv)

    sim_proc = None
    if args.spawn_sim:
        sim_proc = subprocess.Popen(
            [sys.executable, "-m", "skate_ros2.sim_endpoint",
             "--model", args.spawn_sim, "--quiet"],
            cwd=str(Path(__file__).resolve().parents[2] / "skate_ros2"))
        print(f"[commander] sim endpoint spawned (pid {sim_proc.pid})")

    import uvicorn
    app = build_app(args.model_dir, real_host=args.real_host)
    print(f"[commander] http://{args.host}:{args.port}  (mode starts in SIM, "
          "dampened — press RESUME in the UI)")
    try:
        uvicorn.run(app, host=args.host, port=args.port, log_level="warning")
    finally:
        if sim_proc:
            sim_proc.terminate()


if __name__ == "__main__":
    main()

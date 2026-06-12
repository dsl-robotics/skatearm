"""Regenerate preview.html: record REAL telemetry from a scripted run against
the MuJoCo sim endpoint, then bake it into the current static/index.html
markup (so the preview always matches the live design).

    python3 scripts/make_preview.py /path/to/skate_teleop/skt_v3
"""

import json
import socket
import sys
import threading
import time
from pathlib import Path

import numpy as np

TOOL = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL))
sys.path.insert(0, str(TOOL.parent / "skate_ros2"))

from skate_commander.bridge import RobotBridge          # noqa: E402
from skate_ros2.sim_endpoint import SkateSimEndpoint    # noqa: E402


def record(model_xml):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0)); port = s.getsockname()[1]; s.close()
    ep = SkateSimEndpoint(str(model_xml), port=port, bind="127.0.0.1",
                          verbose=False)
    th = threading.Thread(target=ep.run, kwargs={"duration": 60.0},
                          daemon=True)
    th.start()
    br = RobotBridge(sim_host="127.0.0.1", sim_port=port, jog_rate=0.5)

    frames = []
    t0 = time.monotonic()
    next_f = 0.0

    def run(dur, fn=None):
        nonlocal next_f
        end = time.monotonic() - t0 + dur
        while (t := time.monotonic() - t0) < end:
            if fn:
                fn(t)
            br.tick(1 / 60, ui_attached=True)
            if t >= next_f:
                s = br.snapshot()
                f = {k: ([round(x, 3) for x in v] if isinstance(v, list)
                         else v)
                     for k, v in s.items() if k not in ("tau",)}
                f["tau"] = None
                frames.append(f)
                next_f = t + 0.1
            time.sleep(1 / 60)

    run(1.2)                                   # dampened arming
    br.resume()

    def wave(t):
        br.jog_dir[:] = 0
        ph = t - 1.2
        br.jog_dir[11] = 1 if np.sin(1.4 * ph) > 0 else -1
        br.jog_dir[19] = -br.jog_dir[11]
        br.jog_dir[9] = br.jog_dir[17] = 1 if ph < 1.6 else 0
        br.jog_dir[24] = 1 if np.sin(0.7 * ph) > 0 else -1
    run(12.0, wave)
    br.jog_stop()
    run(1.0)
    br.trigger_estop()
    run(1.5)
    br.close(); ep.close(); th.join(timeout=5)
    return frames


def main():
    skt = Path(sys.argv[1])
    model_xml = skt / "skt_v3_collision.xml"
    if not model_xml.exists():
        model_xml = skt / "skt_v3_control.xml"
    frames = record(model_xml)

    sys.path.insert(0, str(TOOL / "skate_commander"))
    from skate_commander.urdf import parse_urdf
    model = parse_urdf(skt / "skt_v3.urdf")
    data = json.dumps({"model": model, "frames": frames},
                      separators=(",", ":"))

    html = (TOOL / "static" / "index.html").read_text(encoding="utf-8")
    html = html.replace('href="/static/', 'href="./static/')
    html = html.replace('src="/static/', 'src="./static/')
    html = html.replace("<title>Skate Commander</title>",
                        "<title>Skate Commander — preview (recorded "
                        "telemetry)</title>")
    inject = f"<script>window.PREVIEW_DATA = {data};</script>\n"
    html = html.replace('<script type="module"', inject
                        + '<script type="module"')
    out = TOOL / "preview.html"
    out.write_text(html, encoding="utf-8")
    print(f"frames={len(frames)}  preview.html={out.stat().st_size // 1024} KB")


if __name__ == "__main__":
    main()

"""Server-side camera — render a MuJoCo camera from the live joint state, hand
out JPEG frames for the cockpit MJPEG panel, and run the vision pipeline
(detect the workspace target) on the same render thread.

Rendering runs in one daemon thread (MuJoCo's renderer is thread-affine: the GL
context is created and used on that thread). The latest JPEG is published behind
a lock; detection requests are serviced on the same thread so all GL work stays
single-threaded.
"""
from __future__ import annotations

import io
import threading
import time
from pathlib import Path

import numpy as np

from . import vision

WORK_CAMERA = "cam_work"
GRASP_Z = -0.055          # magenta cube centre height in the generated scene (m)

_VISUAL = """  <visual>
    <headlight ambient="0.45 0.45 0.45" diffuse="0.6 0.6 0.6" specular="0.1 0.1 0.1"/>
    <global offwidth="1280" offheight="960"/>
  </visual>
"""
# A lit tabletop + magenta cube + top-down work camera, placed in the right
# arm's reachable band (validated in skate_capture/make_scene.py).
_SCENE = """    <light name="worklight" pos="0.05 0.34 1.2" dir="0 0 -1" diffuse="0.8 0.8 0.8" specular="0.2 0.2 0.2"/>
    <geom name="floor" type="plane" pos="0 0 -0.95" size="3 3 0.1" rgba="0.17 0.19 0.22 1"/>
    <body name="table" pos="0.05 0.34 -0.10">
      <geom name="table_top" type="box" size="0.18 0.15 0.02" rgba="0.42 0.32 0.24 1"/>
    </body>
    <body name="target" pos="0.13 0.35 -0.055">
      <geom name="target_geom" type="box" size="0.025 0.025 0.025" rgba="0.9 0.12 0.86 1"/>
    </body>
    <camera name="cam_work" pos="0.05 0.34 0.42" xyaxes="1 0 0 0 1 0" fovy="55"/>
"""


def build_scene_xml(model_dir):
    """Generate <model_dir>/.skate_scene.xml = control.xml + a lit tabletop with
    a magenta cube and a top-down work camera. Returns its path, or the plain
    control.xml path if the workspace can't be added."""
    md = Path(model_dir)
    ctrl = md / "skt_v3_control.xml"
    out = md / ".skate_scene.xml"
    try:
        xml = ctrl.read_text(encoding="utf-8")
        if "cam_work" not in xml and "</asset>" in xml and "</worldbody>" in xml:
            xml = xml.replace("</asset>", "</asset>\n" + _VISUAL, 1)
            xml = xml.replace("</worldbody>", _SCENE + "  </worldbody>", 1)
        out.write_text(xml, encoding="utf-8")
        return str(out)
    except Exception:
        return str(ctrl)


class CameraStreamer:
    def __init__(self, model_path, get_qpos, width=640, height=480, fps=12):
        import mujoco
        self._mj = mujoco
        self.m = mujoco.MjModel.from_xml_path(str(model_path))
        self.cams = [self.m.camera(i).name for i in range(self.m.ncam)]
        self.has_work = WORK_CAMERA in self.cams
        self.cam = WORK_CAMERA if self.has_work else (self.cams[-1] if self.cams else None)
        self.get_qpos = get_qpos
        self.width, self.height, self.fps = width, height, fps
        self._jpeg = _solid_jpeg(width, height)
        self._lock = threading.Lock()
        self._stop = False
        self._det_req = None
        self._det_res = {"found": False}
        self._det_evt = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def set_cam(self, name):
        if name in self.cams:
            self.cam = name
            return True
        return False

    def jpeg(self):
        with self._lock:
            return self._jpeg

    def detect(self, grasp_z=GRASP_Z, timeout=2.5):
        """Render the work camera and find the target. Serviced on the render
        thread (GL-safe). Returns vision.detect()'s dict."""
        if not self.has_work:
            return {"found": False, "error": "no work camera"}
        self._det_evt.clear()
        self._det_req = grasp_z
        if self._det_evt.wait(timeout):
            return self._det_res
        return {"found": False, "error": "timeout"}

    def close(self):
        self._stop = True

    def _loop(self):
        from PIL import Image
        d = self._mj.MjData(self.m)
        renderer = self._mj.Renderer(self.m, self.height, self.width)
        n = min(26, self.m.nq)
        period = 1.0 / self.fps
        while not self._stop:
            t0 = time.time()
            try:
                q = self.get_qpos()
                if q is not None:
                    d.qpos[:n] = np.asarray(q, dtype=float)[:n]
                    self._mj.mj_forward(self.m, d)
                renderer.update_scene(d, camera=(self.cam if self.cam else -1))
                img = renderer.render()
                buf = io.BytesIO()
                Image.fromarray(img).save(buf, "JPEG", quality=70)
                with self._lock:
                    self._jpeg = buf.getvalue()
                if self._det_req is not None:                  # serve detection
                    gz = self._det_req
                    self._det_req = None
                    try:
                        renderer.update_scene(d, camera=WORK_CAMERA)
                        wimg = renderer.render()
                        cid = self.m.camera(WORK_CAMERA).id
                        self._det_res = vision.detect(
                            wimg, d.cam_xpos[cid], d.cam_xmat[cid],
                            float(self.m.cam_fovy[cid]), gz)
                    except Exception as exc:
                        self._det_res = {"found": False, "error": str(exc)}
                    self._det_evt.set()
            except Exception:
                pass
            dt = period - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)


def _solid_jpeg(w, h, rgb=(14, 15, 18)):
    from PIL import Image
    buf = io.BytesIO()
    Image.new("RGB", (w, h), rgb).save(buf, "JPEG")
    return buf.getvalue()

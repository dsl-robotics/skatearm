"""Server-side camera — render a MuJoCo camera from the live joint state and
hand out JPEG frames for an MJPEG panel in the cockpit.

The cockpit server already knows every joint angle and already depends on
MuJoCo (collision guard), so it can render any camera defined in the model
without touching the sim process. Rendering runs in its own daemon thread
(MuJoCo's renderer is thread-affine: the GL context is created and used on
that one thread) and publishes the latest JPEG behind a lock, so any number
of MJPEG clients just read the most recent frame — render cost is independent
of the client count.
"""
from __future__ import annotations

import io
import threading
import time

import numpy as np


class CameraStreamer:
    def __init__(self, model_path, get_qpos, width=480, height=360, fps=12):
        import mujoco
        self._mj = mujoco
        self.m = mujoco.MjModel.from_xml_path(str(model_path))
        self.cams = [self.m.camera(i).name for i in range(self.m.ncam)]
        self.cam = self.cams[-1] if self.cams else None
        self.get_qpos = get_qpos
        self.width, self.height, self.fps = width, height, fps
        self._jpeg = _solid_jpeg(width, height)
        self._lock = threading.Lock()
        self._stop = False
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

    def close(self):
        self._stop = True

    def _loop(self):
        from PIL import Image
        # Renderer + MjData are created HERE so the GL context lives on this thread.
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

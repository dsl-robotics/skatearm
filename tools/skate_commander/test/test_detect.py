"""Pluggable object detector (v0.7.12) — headless.

Built-in colour/shape labeling + target selection are pure; the YOLO backend is
env-gated and only active when ultralytics is importable, so here we check it
stays OFF (and falls back to the built-in labels) when not opted in.
"""
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from skate_commander import detect          # noqa: E402


def _obj(rgb, length_mm, width_mm, oid=0):
    return {"id": oid, "mean_rgb": rgb, "length_mm": length_mm,
            "width_mm": width_mm, "center": [0.1, 0.3, -0.05],
            "feasible": True}


def test_colour_name():
    assert detect.colour_name([0.9, 0.12, 0.86]) == "magenta"
    assert detect.colour_name([0.12, 0.75, 0.85]) == "cyan"
    assert detect.colour_name([0.2, 0.7, 0.3]) == "green"
    assert detect.colour_name([0.82, 0.15, 0.15]) == "red"
    print("PASS colour_name: magenta/cyan/green/red resolved")


def test_shape_name():
    assert detect.shape_name({"length_mm": 50, "width_mm": 47}) == "cube"
    assert detect.shape_name({"length_mm": 90, "width_mm": 55}) == "box"
    assert detect.shape_name({"length_mm": 120, "width_mm": 25}) == "bar"
    print("PASS shape_name: cube/box/bar by aspect")


def test_label_objects():
    objs = [_obj([0.9, 0.12, 0.86], 50, 47, 0),
            _obj([0.12, 0.75, 0.85], 40, 39, 1)]
    detect.label_objects(objs)
    assert objs[0]["label"] == "magenta cube", objs[0]
    assert objs[1]["colour"] == "cyan" and objs[1]["detector"] == "builtin"
    print(f"PASS label_objects: {[o['label'] for o in objs]}")


def test_pick_target():
    objs = [_obj([0.9, 0.12, 0.86], 50, 47, 0),
            _obj([0.12, 0.75, 0.85], 40, 39, 1)]
    detect.label_objects(objs)
    assert detect.pick_target(objs, None)["id"] == 0          # default = best
    assert detect.pick_target(objs, 1)["id"] == 1             # by id
    assert detect.pick_target(objs, "cyan")["id"] == 1        # by colour
    assert detect.pick_target(objs, "magenta cube")["id"] == 0  # by label
    assert detect.pick_target(objs, "banana") is None         # no match
    print("PASS pick_target: default / id / colour / label / no-match")


def test_yolo_gated_off_by_default():
    os.environ.pop("SKATE_YOLO", None)
    assert not detect.yolo_available()
    objs = [_obj([0.9, 0.12, 0.86], 50, 47, 0)]
    detect.detect(objs)                       # no image, not opted in
    assert objs[0]["detector"] == "builtin" and objs[0]["label"] == "magenta cube"
    print("PASS yolo gated OFF by default -> built-in labels")


def test_yolo_gracefully_falls_back():
    # opted in but ultralytics not installed -> still built-in, no crash
    os.environ["SKATE_YOLO"] = "1"
    try:
        import importlib.util
        have = importlib.util.find_spec("ultralytics") is not None
        assert detect.yolo_available() == have
        objs = [_obj([0.12, 0.75, 0.85], 40, 39, 0)]
        detect.detect(objs, rgb_image=None, cam=None)   # no frame -> built-in
        assert objs[0]["detector"] == "builtin"
    finally:
        os.environ.pop("SKATE_YOLO", None)
    print("PASS yolo opt-in falls back to built-in when unavailable")


if __name__ == "__main__":
    test_colour_name()
    test_shape_name()
    test_label_objects()
    test_pick_target()
    test_yolo_gated_off_by_default()
    test_yolo_gracefully_falls_back()
    print("DETECT OK")

#!/usr/bin/env python3
"""One-shot SkateArm sim setup.

Builds the control + collision models from an official ``skt_v3`` model directory
(the upstream Rbotic/skate_teleop model), so the demos, the cockpit and the
model-gated tests all have what they need.

Examples
--------
    py sim/make.py --skt-dir path/to/skt_v3      # build from a model you already have
    py sim/make.py --clone                        # git clone the model first, then build
    py sim/make.py --skt-dir path/to/skt_v3 --boxes   # legacy box collision geometry

On Windows use ``py``; on Linux/macOS ``python3``.
"""
import argparse
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent          # sim/
REPO = HERE.parent
UPSTREAM = "https://github.com/Rbotic/skate_teleop.git"


def run(cmd):
    print("  >>", " ".join(str(c) for c in cmd))
    subprocess.check_call(cmd)


def main():
    ap = argparse.ArgumentParser(
        description="Build the SkateArm control + collision models in one shot.")
    ap.add_argument("--skt-dir", type=Path,
                    help="path to the skt_v3 model directory")
    ap.add_argument("--clone", action="store_true",
                    help=f"git clone {UPSTREAM} into ./skate_teleop first, then build")
    ap.add_argument("--boxes", action="store_true",
                    help="legacy box collision geometry (default: capsules)")
    a = ap.parse_args()

    py = sys.executable or "python"
    skt = a.skt_dir

    if a.clone:
        dest = REPO / "skate_teleop"
        if not dest.exists():
            run(["git", "clone", "--depth", "1", UPSTREAM, str(dest)])
        else:
            print(f"  (reusing existing {dest})")
        skt = dest / "skt_v3"

    if skt is None:
        ap.error("give --skt-dir PATH (or --clone to fetch the model)")
    skt = skt.resolve()
    if not skt.exists():
        ap.error(f"{skt} does not exist — wrong --skt-dir?")

    print(f"[1/2] control model  (actuators + sensors)  <- {skt}")
    run([py, str(HERE / "make_control_model.py"), str(skt)])

    print(f"[2/2] collision model ({'boxes' if a.boxes else 'capsules'})  <- {skt}")
    coll = [py, str(HERE / "make_collision_model.py"), str(skt)]
    if a.boxes:
        coll.append("--boxes")
    run(coll)

    print("\nDone. Next steps:")
    print(f"  PowerShell:  $env:SKT_DIR = \"{skt}\"")
    print(f"  bash:        export SKT_DIR=\"{skt}\"")
    print(f"  {Path(py).name} sim/demo_wave.py --model {skt}")
    print(f"  {Path(py).name} -m pytest -q tools/skate_commander/test"
          "    # now runs the model-gated tests too")


if __name__ == "__main__":
    main()
